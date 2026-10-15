#include "companion/camera_metadata_worker.hpp"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <fstream>
#include <mutex>
#include <thread>
#include <utility>

namespace companion {
struct CameraMetadataWorker::Impl {
  const std::filesystem::path runtime;
  const std::string stem;
  mutable std::mutex mutex;
  std::condition_variable wake;
  bool pending{true}, stopping{false};
  std::optional<CameraMetadata> result;
  CameraMetadataStats stats;
  std::thread thread;
  Impl(std::filesystem::path path, int vehicle)
      : runtime(std::move(path)), stem("camera_" + std::to_string(vehicle)),
        thread([this] { run(); }) {}
  ~Impl() {
    { std::lock_guard<std::mutex> lock(mutex); stopping = true; }
    wake.notify_one(); thread.join();
  }
  void run() {
    for (;;) {
      {
        std::unique_lock<std::mutex> lock(mutex);
        wake.wait(lock, [&] { return stopping || pending; });
        if (stopping) return;
        pending = false;
      }
      const auto begin = std::chrono::steady_clock::now();
      std::optional<CameraMetadata> metadata;
      try {
        auto directory = runtime;
        if (std::filesystem::exists(runtime / "camera")) directory /= "camera";
        std::ifstream stream(directory / (stem + ".json"));
        nlohmann::json json; stream >> json;
        metadata = CameraMetadata{json.at("frame_seq").get<uint64_t>(),
            json.at("sim_time_us").get<uint64_t>(), (directory / (stem + ".ppm")).string()};
      } catch (const std::exception&) {}
      const auto duration = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now() - begin).count());
      std::optional<CameraMetadata> previous;
      {
        std::lock_guard<std::mutex> lock(mutex);
        ++stats.reads; stats.max_read_us = std::max(stats.max_read_us, duration);
        if (metadata) {
          if (result) ++stats.result_replaced;
          previous = std::move(result); result = std::move(metadata);
        } else ++stats.failures;
      }
    }
  }
};
CameraMetadataWorker::CameraMetadataWorker(std::filesystem::path runtime, int vehicle)
    : impl_(std::make_unique<Impl>(std::move(runtime), vehicle)) {}
CameraMetadataWorker::~CameraMetadataWorker() = default;
void CameraMetadataWorker::request() {
  std::unique_lock<std::mutex> lock(impl_->mutex, std::try_to_lock);
  if (!lock.owns_lock()) { ++request_busy_; return; }
  impl_->pending = true;
  lock.unlock(); impl_->wake.notify_one();
}
std::optional<CameraMetadata> CameraMetadataWorker::poll() {
  std::unique_lock<std::mutex> lock(impl_->mutex, std::try_to_lock);
  if (!lock.owns_lock()) { ++poll_busy_; return std::nullopt; }
  auto result = std::move(impl_->result); impl_->result.reset();
  return result;
}
CameraMetadataStats CameraMetadataWorker::stats() const {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  auto result = impl_->stats;
  result.request_busy = request_busy_; result.poll_busy = poll_busy_;
  return result;
}
}  // namespace companion
