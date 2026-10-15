#include "companion/vision_worker.hpp"
#include <Eigen/Geometry>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>
#ifdef __linux__
#include <sys/resource.h>
#endif

namespace companion {
namespace {
using Clock = std::chrono::steady_clock;
double elapsed(Clock::time_point start) {
  return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}
struct Completed {
  VisionResult result;
  std::array<double, 3> milliseconds{{-1, -1, -1}};
  std::array<bool, 3> accepted{};
  double compute_ms{0};
};
Completed process(const VisionRequest& request) {
  Completed done;
  const auto start = Clock::now();
  auto& out = done.result;
  out.stamp = request.stamp; out.sequence = request.sequence;
  try {
    auto image = request.image;
    if (!image) {
      auto loaded = readPpm(request.image_path);
      if (loaded) image = std::make_shared<const PpmImage>(std::move(*loaded));
    }
    if (!image || (request.verify_metadata &&
        (image->sim_time_us != request.stamp || image->frame_seq != request.sequence))) {
      out.error = "camera_frame_mismatch";
    } else {
      out.frame_valid = true;
      const auto measure = [&](int stage, auto&& call) {
        const auto begin = Clock::now(); done.accepted[stage] = call();
        done.milliseconds[stage] = elapsed(begin);
      };
      if (request.pixel) measure(0, [&] {
        out.pixel = detectArmorStrips(*image, request.camera, request.detection);
        return out.pixel.valid;
      });
      if (request.pnp) measure(1, [&] {
#ifdef COMPANION_WITH_OPENCV
        out.pnp = detectRmArmor(*image, request.camera, request.strip_height_m, request.bottom_separation_m);
        if (out.pnp.valid && request.pose_valid) {
          const auto projection=cameraPointProjection(out.pnp.position_camera_m,request.exposure_pose,request.camera_forward_m);
          const auto covariance=projection?worldObservationCovariance(projection->pose_jacobian,
              request.exposure_covariance,request.pnp_variance_m2):std::nullopt;
          if(projection&&covariance){
            out.world=projection->position;out.world_covariance=*covariance;out.world_valid=true;
          }else out.error="invalid_pnp_world_uncertainty";
        }
        return out.pnp.valid;
#else
        throw std::runtime_error("OpenCV armor detection unavailable");
        return false;
#endif
      });
      if (request.guide_mode == GuideMode::Always ||
          (request.guide_mode == GuideMode::Fallback && !out.world_valid)) measure(2, [&] {
        out.green_ran = true;
        const auto& guide_image = request.guide_image ? *request.guide_image : *image;
        out.guide = detectGreenGuide(guide_image, request.guide_x_px, request.guide_y_px, request.guide_radius_px);
        const auto projection = request.pose_valid ? guideArmorProjection(out.guide, request.camera,
            request.exposure_pose, request.camera_forward_m, request.guide_center.x(), request.guide_offset) : std::nullopt;
        const auto covariance=projection?worldObservationCovariance(projection->pose_jacobian,
            request.exposure_covariance,request.guide_variance_m2):std::nullopt;
        const bool accepted = projection && covariance && std::abs(projection->position.y() - request.guide_center.y()) <= request.guide_y_gate_m &&
            std::abs(projection->position.z() - request.guide_center.z() - request.guide_offset.z()) < request.guide_z_gate_m;
        if (accepted && !out.world_valid) {
          out.world = projection->position; out.world_covariance=*covariance;out.world_valid = true; out.green = true;
        }
        if(projection&&!covariance)out.error="invalid_guide_world_uncertainty";
        return accepted;
      });
    }
  } catch (const std::exception& e) { out.error = e.what(); }
  done.compute_ms = elapsed(start);
  return done;
}
}

struct VisionWorker::Impl {
  mutable std::mutex mutex;
  std::condition_variable wake;
  std::optional<VisionRequest> pending;
  std::optional<VisionResult> result;
  VisionWorkerStats stats;
  bool stopping{false}, asynchronous, record_timings;
  std::thread thread;
  explicit Impl(bool async, bool record) : asynchronous(async), record_timings(record) {
    if (async) thread = std::thread([this] { run(); });
  }
  ~Impl() {
    if (!asynchronous) return;
    { std::lock_guard<std::mutex> lock(mutex); stopping = true; }
    wake.notify_one(); thread.join();
  }
  void publish(Completed done) {
    ++stats.completed;
    stats.failed += !done.result.error.empty();
    stats.last_compute_ms = done.compute_ms;
    stats.max_compute_ms = std::max(stats.max_compute_ms, done.compute_ms);
    for (int stage = 0; stage < 3; ++stage) if (done.milliseconds[stage] >= 0) {
      if(record_timings)stats.stages[stage].milliseconds.push_back(done.milliseconds[stage]);
      done.accepted[stage] ? ++stats.stages[stage].accepted : ++stats.stages[stage].rejected;
    }
    if (result) ++stats.result_replaced;
    result = std::move(done.result);
  }
  void run() {
#ifdef __linux__
    { std::lock_guard<std::mutex> lock(mutex);
      if (setpriority(PRIO_PROCESS, 0, 5) == 0) stats.worker_nice = 5; }
#endif
    for (;;) {
      std::optional<VisionRequest> job;
      {
        std::unique_lock<std::mutex> lock(mutex);
        wake.wait(lock, [&] { return stopping || pending.has_value(); });
        if (stopping) return;
        job = std::move(pending); pending.reset();
      }
      auto done = process(*job);
      { std::lock_guard<std::mutex> lock(mutex); publish(std::move(done)); }
    }
  }
};
VisionWorker::VisionWorker(bool asynchronous, bool record_timings) : impl_(std::make_unique<Impl>(asynchronous,record_timings)) {}
VisionWorker::~VisionWorker() = default;
bool VisionWorker::submit(VisionRequest request) {
  if (!request.stamp || request.stamp <= last_submitted_) return false;
  if (!impl_->asynchronous) {
    last_submitted_ = request.stamp;
    auto done = process(request);
    std::lock_guard<std::mutex> lock(impl_->mutex);
    ++impl_->stats.submitted; impl_->publish(std::move(done)); return true;
  }
  std::unique_lock<std::mutex> lock(impl_->mutex, std::try_to_lock);
  if (!lock.owns_lock()) return false;
  last_submitted_ = request.stamp; ++impl_->stats.submitted;
  if (impl_->pending) ++impl_->stats.pending_replaced;
  impl_->pending = std::move(request); impl_->wake.notify_one(); return true;
}
std::optional<VisionResult> VisionWorker::poll(uint64_t now, uint64_t timeout) {
  std::unique_lock<std::mutex> lock(impl_->mutex, std::try_to_lock);
  if (!lock.owns_lock() || !impl_->result) return std::nullopt;
  auto result = std::move(impl_->result); impl_->result.reset();
  if (result->stamp > now) { ++impl_->stats.future; return std::nullopt; }
  if (result->stamp <= last_published_ || now - result->stamp > timeout) {
    ++impl_->stats.stale; return std::nullopt;
  }
  last_published_ = result->stamp;
  return result;
}
VisionWorkerStats VisionWorker::stats() const {
  std::lock_guard<std::mutex> lock(impl_->mutex); return impl_->stats;
}
}  // namespace companion
