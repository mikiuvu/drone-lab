/** @brief 定位、视觉、导航与撞击任务的计时和定频回放测试。 */
#include "companion/armor_tracker.hpp"
#include "companion/estimator.hpp"
#include "companion/intercept.hpp"
#include "companion/periodic_motion_worker.hpp"
#include "companion/vision_worker.hpp"
#include "companion/latest_tof.hpp"
#include "companion/rm_auto_aim.hpp"
#include "companion/team_mission.hpp"
#include "companion/terrain.hpp"
#include "companion/vision.hpp"
#include <opencv2/core.hpp>
#include <nlohmann/json.hpp>
#include <sys/resource.h>
#include <sys/utsname.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <numeric>
#include <thread>

using Json = nlohmann::json;
using Clock = std::chrono::steady_clock;
using namespace companion;
namespace {
volatile std::sig_atomic_t running = 1;
volatile double sink = 0;
void stop(int) { running = 0; }

struct Stats {
  std::vector<double> milliseconds;
  uint64_t accepted = 0, rejected = 0;
  Json json() const {
    if (milliseconds.empty()) return {{"samples", 0}};
    auto v = milliseconds;
    std::sort(v.begin(), v.end());
    auto percentile = [&](double p) { return v[static_cast<size_t>(p * (v.size() - 1))]; };
    return {{"samples", v.size()}, {"mean_ms", std::accumulate(v.begin(), v.end(), 0.0) / v.size()},
      {"p50_ms", percentile(.50)}, {"p95_ms", percentile(.95)}, {"p99_ms", percentile(.99)},
      {"max_ms", v.back()}, {"over_10ms", std::count_if(v.begin(), v.end(), [](double ms) { return ms > 10; })},
      {"over_20ms", std::count_if(v.begin(), v.end(), [](double ms) { return ms > 20; })},
      {"accepted", accepted}, {"rejected", rejected}};
  }
};

struct Options {
  std::string image, guide_image, terrain, poses, output, mode = "paced";
  double seconds = 60;
  int camera_hz = 15, tof_hz = 15;
  bool uwb_fallback = false;
  bool periodic_async = true;
  bool vision_async = true, tof_bounded = true;
  bool tof_navigation_slot = true;
  GuideMode guide_mode = GuideMode::Always;
};

Options options(int argc, char** argv) {
  Options o;
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    if (++i >= argc) throw std::runtime_error("Missing option value");
    const std::string value = argv[i];
    if (key == "--image") o.image = value;
    else if (key == "--guide-image") o.guide_image = value;
    else if (key == "--terrain") o.terrain = value;
    else if (key == "--poses") o.poses = value;
    else if (key == "--output") o.output = value;
    else if (key == "--mode") o.mode = value;
    else if (key == "--seconds") o.seconds = std::stod(value);
    else if (key == "--camera-hz") o.camera_hz = std::stoi(value);
    else if (key == "--tof-hz") o.tof_hz = std::stoi(value);
    else if (key == "--uwb-fallback") {
      if (value != "0" && value != "1") throw std::runtime_error("Invalid UWB fallback value");
      o.uwb_fallback = value == "1";
    }
    else if (key == "--periodic-async") {
      if (value != "0" && value != "1") throw std::runtime_error("Invalid periodic async value");
      o.periodic_async = value == "1";
    }
    else if (key == "--vision-async" || key == "--tof-bounded" || key == "--tof-navigation-slot") {
      if (value != "0" && value != "1") throw std::runtime_error("Invalid scheduling value");
      if (key == "--vision-async") o.vision_async = value == "1";
      else if(key=="--tof-bounded")o.tof_bounded = value == "1";
      else o.tof_navigation_slot=value=="1";
    }
    else if (key == "--vision-branches") {
      if (value == "all") o.guide_mode = GuideMode::Always;
      else if (value == "fallback") o.guide_mode = GuideMode::Fallback;
      else throw std::runtime_error("Invalid vision branch policy");
    }
    else throw std::runtime_error("Unknown option: " + key);
  }
  if (o.image.empty() || o.terrain.empty() || o.poses.empty() || o.output.empty() ||
      !std::isfinite(o.seconds) || o.seconds <= 0 || o.seconds > 600 ||
      o.camera_hz < 1 || o.camera_hz > 60 || o.tof_hz < 1 || o.tof_hz > 30 ||
      (o.mode != "paced" && o.mode != "micro")) throw std::runtime_error("Invalid load-test options");
  return o;
}

Json readJson(const std::string& path) {
  std::ifstream stream(path);
  if (!stream) throw std::runtime_error("Cannot open " + path);
  Json j; stream >> j; return j;
}

double cpuSeconds() {
  rusage r{};
  if (getrusage(RUSAGE_SELF, &r)) throw std::runtime_error("getrusage failed");
  return r.ru_utime.tv_sec + r.ru_stime.tv_sec + (r.ru_utime.tv_usec + r.ru_stime.tv_usec) * 1e-6;
}

Json resources(double wall_s) {
  Json row{{"elapsed_s", wall_s}, {"process_cpu_seconds", cpuSeconds()}};
  std::ifstream status("/proc/self/status");
  std::string line;
  while (std::getline(status, line)) {
    if (line.rfind("VmRSS:", 0) == 0) row["rss_kib"] = std::stoll(line.substr(6));
    if (line.rfind("VmHWM:", 0) == 0) row["peak_rss_kib"] = std::stoll(line.substr(6));
  }
  std::ifstream memory("/proc/meminfo");
  while (std::getline(memory, line))
    if (line.rfind("MemAvailable:", 0) == 0) row["available_memory_kib"] = std::stoll(line.substr(13));
  const std::filesystem::path thermal("/sys/class/thermal");
  if (std::filesystem::exists(thermal)) for (const auto& entry : std::filesystem::directory_iterator(thermal)) {
    if (entry.path().filename().string().rfind("thermal_zone", 0) != 0) continue;
    std::ifstream temperature(entry.path() / "temp");
    double raw = 0;
    if (temperature >> raw) row["temperatures_c"][entry.path().filename().string()] = raw / 1000;
  }
  return row;
}

struct PoseInput { uint64_t us; Eigen::Vector3d position; int vehicle; };

class Workload {
 public:
  explicit Workload(const Options& o) : o_(o), terrain_(TerrainMap::load(o.terrain)),
      config_(configuration()), estimator_(config_), mission_(missionConfig()), periodic_(o.periodic_async), vision_(o.vision_async, true) {
    auto image = readPpm(o.image);
    if (!image) throw std::runtime_error("Invalid PPM fixture");
    image_ = std::make_shared<const PpmImage>(std::move(*image));
    auto guide_image = readPpm(o.guide_image.empty() ? o.image : o.guide_image);
    if (!guide_image || guide_image->width != image_->width || guide_image->height != image_->height)
      throw std::runtime_error("Invalid guide PPM fixture");
    guide_image_ = std::make_shared<const PpmImage>(std::move(*guide_image));
    periodic_.setHorizonCap(.6);
    periodic_.setHorizonFraction(.25);
    if (o_.uwb_fallback) {
      auto config = config_; config.terrain.reset(); config.map_lines.clear();
      fallback_estimator_ = std::make_unique<Estimator>(config);
    }
    camera_.width = image_->width; camera_.height = image_->height;
    camera_.fx_px = camera_.fy_px = image_->width * .5;
    camera_.cx_px = (image_->width - 1) * .5; camera_.cy_px = (image_->height - 1) * .5;
    camera_.strip_center_separation_m = .18409;
    const auto recorded = readJson(o.poses);
    for (const auto& j : recorded.at("poses"))
      poses_.push_back({j.at("sim_time_us").get<uint64_t>(),
          Eigen::Vector3d(j.at("x_m").get<double>(), j.at("y_m").get<double>(), j.at("z_m").get<double>()),
          j.at("vehicle").get<int>()});
    if (poses_.size() < 200 || poses_.back().us <= poses_.front().us)
      throw std::runtime_error("Recorded visual pose fixture is too short");
    for (int sensor = 0; sensor < 3; ++sensor) {
      frames_[sensor].sensor_index = sensor;
      for (int cell = 0; cell < 64; ++cell) {
        TerrainMap::Hit hit;
        const auto ray = Estimator::tofRayBody(config_.tof_mounts[sensor], cell / 8, cell % 8);
        if (terrain_->raycast({10.6, 0, 1.44}, ray, 4, hit)) {
          frames_[sensor].valid[cell] = 1; frames_[sensor].distance_m[cell] = hit.depth_m;
        }
      }
    }
    for (int id = 1; id < 4; ++id) {
      PeerEstimate peer; peer.vehicle_id = id; peer.pose = config_.initial_pose;
      peer.pose.y_m = id * .7; peer.xy_structural_valid = true;
      peer.pose_covariance = Eigen::Matrix3d::Identity() * .01;
      peer.reference_origin = id; peer.reference_path = {id}; peer.reference_seq = 1;
      peer.source = "ESTIMATE"; peers_.push_back(peer);
    }
  }

  template<class F> void measure(const std::string& name, F&& f) {
    const auto begin = Clock::now(); f();
    stats_[name].milliseconds.push_back(std::chrono::duration<double, std::milli>(Clock::now() - begin).count());
  }

  void tick(uint64_t tick, uint64_t previous, bool all = false) {
    const uint64_t now = 1000000 + tick * 10000;
    const auto due = [&](int rate, int phase = 0) {
      return all || tick == 0 || (tick + phase) * rate / 100 > (previous + phase) * rate / 100;
    };
    const bool navigation_due=due(50);
    measure("imu_predict", [&] {
      ImuSample imu; imu.sim_time_us = now; imu.attitude_valid = true;
      estimator_.predict(imu); sink = estimator_.estimate().pose.x_m;
    });
    if (due(50)) measure("flow_height", [&] {
      estimator_.fuseHeightAssist({now, 1.44, "SIM_HEIGHT_ASSIST"});
      auto r = estimator_.fuseOpticalFlow({now, 0, 0, .95, .0004});
      count("flow_height", r.accepted);
    });
    for (int sensor = 0; sensor < 3; ++sensor) if (due(o_.tof_hz, sensor * 2)) {
      frames_[sensor].sim_time_us = now;
      if (o_.tof_bounded) tof_queue_.submit(frames_[sensor]);
      else measure("tof_frame", [&] {
        auto r = estimator_.fuseTof(frames_[sensor]); count("tof_frame", r.accepted); sink = r.inliers;
      });
    }
    if (due(20)) measure("uwb_peers", [&] {
      for (auto& peer : peers_) {
        peer.sim_time_us = peer.reference_time_us = now; ++peer.reference_seq;
        auto r = estimator_.fuseUwb({now, peer.vehicle_id, peer.pose.y_m, kPi / 2, true, 1, false}, peer);
        count("uwb_peers", r.accepted); ++reasons_["uwb_peers"][r.reason];
      }
    });
    if (fallback_estimator_ && due(20)) measure("uwb_fallback", [&] {
      fallback_estimator_->fuseHeightAssist({now, 1.44, "SIM_HEIGHT_ASSIST"});
      for (const auto& peer : peers_) {
        auto r = fallback_estimator_->fuseUwb({now, peer.vehicle_id, peer.pose.y_m, kPi / 2, true, 1, false}, peer);
        count("uwb_fallback", r.accepted); ++reasons_["uwb_fallback"][r.reason];
      }
    });
    if (due(o_.camera_hz)) {
      measure("vision_submit", [&] {
        VisionRequest request; request.stamp = now; request.sequence = tick + 1;
        request.image = image_; request.guide_image = guide_image_; request.verify_metadata = false;
        request.camera = camera_; request.detection.enemy_color = ArmorColor::Blue; request.detection.max_pair_aspect = 6;
        request.strip_height_m = .093427; request.bottom_separation_m = .108873;
        request.exposure_pose = config_.initial_pose; request.pose_valid = true;
        request.guide_mode = o_.guide_mode; request.guide_x_px = camera_.cx_px; request.guide_y_px = camera_.cy_px;
        request.guide_radius_px = 100; request.guide_center.x() = 11.58; request.camera_forward_m = .05;
        count("vision_submit", vision_.submit(std::move(request)));
      });
    }
    measure("vision_poll", [&] {
      if (auto done = vision_.poll(now, camera_.timeout_us)) {
        observation_ = done->pixel; camera_stamp_ = done->stamp; ++vision_used_;
        frame_ages_.milliseconds.push_back((now - done->stamp) * .001);
      }
    });
    if (navigation_due) {
      measure("target_tracking", [&] {
        advanceRecordedPoses(tick * 10000, now);
        track_ = tracker_.at(now); count("target_tracking", track_.valid); sink = track_.position.y();
      });
      measure("periodic_service", [&] { periodic_.service(now); });
      measure("periodic_learning", [&] {
        if (track_.valid) periodic_.observeMeasurement(track_.measurement_time_us,track_.measurement_position);
        auto prediction = periodic_.predict(now); count("periodic_learning", prediction.valid); sink = prediction.position.y();
        if (prediction.valid && (prediction_trace_.empty() || now >= prediction_trace_.back().stamp + 100000))
          prediction_trace_.push_back({now, prediction.position.y(), prediction.confidence, periodic_.period_estimate()});
        const auto& learning=periodic_.learningStats();
        if(periodic_.ready()!=last_periodic_ready_||learning.motion_resets!=last_motion_resets_){
          periodic_events_.push_back({{"time_us",now},{"ready",periodic_.ready()},
            {"motion_resets",learning.motion_resets},{"reset_error_m",learning.last_motion_reset_error_m},
            {"reset_shared",learning.last_motion_reset_shared},{"reset_boundary_us",learning.last_motion_reset_us},
            {"samples",periodic_.sample_count()},{"reject",periodic_.learn_reject()}});
          last_periodic_ready_=periodic_.ready();last_motion_resets_=learning.motion_resets;
        }
      });
      measure("navigation", [&] {
        auto intention = makeControlIntention(now, estimator_.estimate(), {}, camera_, peers_, {{11.0, 0, 0}}, &waypoint_, .45, false);
        sink = intention.velocity_flu_mps.norm();
      });
      const auto navigation_time=Clock::now();
      if(last_navigation_!=Clock::time_point{})navigation_intervals_.milliseconds.push_back(
          std::chrono::duration<double,std::milli>(navigation_time-last_navigation_).count());
      last_navigation_=navigation_time;
      measure("intercept_guidance", [&] {
        auto e = estimator_.estimate();
        auto plan = planIntercept({e.pose.x_m, e.pose.y_m, e.pose.z_m}, e.velocity_enu_mps,
            track_.position, track_.velocity, 3, 1.5, .8);
        auto visual = visual_.update(now, observation_, camera_stamp_);
        count("intercept_guidance", plan.valid && track_.valid); sink = plan.velocity.norm() + visual.velocity_flu_mps.norm();
      });
    }
    if (due(10)) measure("team_impact_schedule", [&] {
      for (int id = 0; id < 4; ++id) {
        TeamVehicleStatus s; s.vehicle = id; s.sim_time_us = now; s.xy_valid = s.z_valid = s.ready = s.armed = true;
        s.visual_ready = true;
        if (commands_.empty()) { s.x_m = 0; s.y_m = id * .5; s.z_m = 1.5355; }
        else {
          const auto& c = commands_[id]; s.x_m = c.target_x_m; s.y_m = c.target_y_m; s.z_m = c.target_z_m;
          if (c.phase == TeamPhase::CONTACT && c.active_vehicle == id) {
            const auto target = missionConfig().target;
            s.x_m = target.x; s.y_m = target.y; s.z_m = target.z; ++impacts_[id];
          }
          if (c.phase == TeamPhase::LAND_ALL) { s.landed = true; s.armed = false; }
        }
        s.impact_seq = impacts_[id]; mission_.updateStatus(s, now);
      }
      commands_ = mission_.step(now, now);
      phase_counts_[teamPhaseName(mission_.phase())]++;
      if (mission_.phase() == TeamPhase::COMPLETE) { mission_ = TeamMission(missionConfig()); commands_.clear(); impacts_.fill(0); }
      sink = commands_.size();
    });
    if (o_.tof_bounded&&(!o_.tof_navigation_slot||navigation_due)) if (auto frame = tof_queue_.take()) measure("tof_frame", [&] {
      tof_start_.milliseconds.push_back(std::chrono::duration<double,std::milli>(Clock::now()-last_navigation_).count());
      tof_frame_age_.milliseconds.push_back((now-frame->sim_time_us)*.001);
      auto r = estimator_.fuseTof(*frame); count("tof_frame", r.accepted); sink = r.inliers;
      tof_end_.milliseconds.push_back(std::chrono::duration<double,std::milli>(Clock::now()-last_navigation_).count());
    });
  }

  Json results() const {
    Json j;
    for (const auto& [name, stat] : stats_) j["stages"][name] = stat.json();
    const auto vision = vision_.stats();
    const std::array<const char*, 3> names{{"vision_pixel", "vision_detect_pnp", "green_guide"}};
    for (int i = 0; i < 3; ++i) {
      Stats stat; stat.milliseconds = vision.stages[i].milliseconds;
      stat.accepted = vision.stages[i].accepted; stat.rejected = vision.stages[i].rejected;
      j["stages"][names[i]] = stat.json();
    }
    j["vision_pipeline"] = {{"asynchronous", o_.vision_async}, {"submitted", vision.submitted},
        {"completed", vision.completed}, {"used", vision_used_}, {"pending_replaced", vision.pending_replaced},
        {"result_replaced", vision.result_replaced}, {"stale", vision.stale}, {"future", vision.future},
        {"failed", vision.failed}, {"nice", vision.worker_nice}, {"max_compute_ms", vision.max_compute_ms},
        {"frame_age", frame_ages_.json()}, {"branches", o_.guide_mode == GuideMode::Always ? "all" : "fallback"}};
    j["tof_queue"] = {{"bounded", o_.tof_bounded}, {"submitted", tof_queue_.submitted},
        {"replaced", tof_queue_.replaced}, {"taken", tof_queue_.taken},
        {"navigation_slot",o_.tof_navigation_slot},{"start_since_navigation",tof_start_.json()},
        {"end_since_navigation",tof_end_.json()},{"frame_age",tof_frame_age_.json()}};
    j["navigation_intervals"] = navigation_intervals_.json();
    j["terrain_storage_bytes"] = terrain_->storageBytes(); j["image_bytes"] = image_->rgb.size();
    j["periodic_model_ready"] = periodic_.ready(); j["periodic_samples"] = periodic_.sample_count();
    j["periodic_period_s"] = periodic_.period_estimate(); j["periodic_reject"] = periodic_.learn_reject();
    const auto& learning = periodic_.learningStats();
    j["periodic_background"] = {{"submitted", learning.submitted}, {"completed", learning.completed},
        {"discarded", learning.discarded}, {"failed", learning.failed}, {"last_ms", learning.last_ms},
        {"publication_rejected", learning.publication_rejected},
        {"motion_suspensions", learning.motion_suspensions}, {"motion_resets", learning.motion_resets},
        {"last_motion_reset_us", learning.last_motion_reset_us},
        {"max_ms", learning.max_ms}, {"nice", learning.worker_nice}};
    j["periodic_async"] = o_.periodic_async;
    j["periodic_events"] = periodic_events_;
    j["periodic_prediction_trace"] = Json::array();
    for (const auto& p : prediction_trace_)
      j["periodic_prediction_trace"].push_back({{"time_us", p.stamp}, {"y_m", p.y},
          {"confidence", p.confidence}, {"period_s", p.period}});
    j["mission_phase_counts"] = phase_counts_;
    j["outcome_reasons"] = reasons_;
    for (int sensor = 0; sensor < 3; ++sensor)
      j["valid_tof_cells"].push_back(std::accumulate(frames_[sensor].valid.begin(), frames_[sensor].valid.end(), 0));
    j["recorded_pose_count"] = poses_.size();
    return j;
  }

 private:
  EstimatorConfig configuration() {
    EstimatorConfig cfg; cfg.terrain = terrain_; cfg.initial_pose = {10.6, 0, 1.44, 0, 0, 0};
    cfg.initial_covariance *= .01;
    for (int sensor = 0; sensor < 3; ++sensor) {
      cfg.tof_mounts[sensor].yaw_rad = sensor * 2 * kPi / 3;
      cfg.tof_mounts[sensor].horizontal_fov_rad = cfg.tof_mounts[sensor].vertical_fov_rad = kPi / 4;
    }
    return cfg;
  }
  static TeamMissionConfig missionConfig() {
    TeamMissionConfig c; c.homes.clear(); c.assembly.clear(); c.order.clear();
    for (int id = 0; id < 4; ++id) { c.homes.push_back({0, id * .5, .55, 0}); c.assembly.push_back({8.8, id * .5, 1.5355, 0}); c.order.push_back(id); }
    c.cruise_z_m = 1.5355;
    c.approach = {10.6, 0, 1.5355, 0}; c.target = {11.58, 0, 1.5355, 0}; return c;
  }
  void count(const std::string& stage, bool accepted) { accepted ? ++stats_[stage].accepted : ++stats_[stage].rejected; }
  void advanceRecordedPoses(uint64_t elapsed, uint64_t now) {
    const uint64_t span = poses_.back().us - poses_.front().us + 100000;
    const auto cycle = elapsed / span;
    if (cycle != replay_cycle_) { replay_index_ = 0; replay_cycle_ = cycle; }
    const uint64_t raw_time = poses_.front().us + elapsed % span;
    std::vector<PeriodicSample> shared;
    while (replay_index_ < poses_.size() && poses_[replay_index_].us <= raw_time) {
      const auto& p = poses_[replay_index_++];
      const uint64_t stamp = 1000000 + cycle * span + p.us - poses_.front().us;
      if (p.vehicle == 0) tracker_.observe(stamp, p.position, .0064);
      else shared.push_back({stamp, p.position.y(),p.vehicle});
    }
    if (!shared.empty()) periodic_.merge(shared);
    sink = now;
  }
  Options o_;
  std::shared_ptr<const TerrainMap> terrain_;
  EstimatorConfig config_;
  Estimator estimator_;
  std::unique_ptr<Estimator> fallback_estimator_;
  TeamMission mission_;
  CameraConfig camera_;
  std::shared_ptr<const PpmImage> image_, guide_image_;
  std::array<TofFrame, 3> frames_;
  std::vector<PeerEstimate> peers_;
  std::vector<PoseInput> poses_;
  size_t replay_index_ = 0;
  uint64_t replay_cycle_ = 0;
  ArmorTracker tracker_;
  ArmorTrack track_;
  PeriodicMotionWorker periodic_;
  VisionWorker vision_;
  LatestTof tof_queue_;
  uint64_t vision_used_{0};
  Stats frame_ages_;
  Stats navigation_intervals_;
  Stats tof_start_,tof_end_,tof_frame_age_;
  Clock::time_point last_navigation_{};
  struct PredictionTrace { uint64_t stamp; double y, confidence, period; };
  std::vector<PredictionTrace> prediction_trace_;
  Json periodic_events_=Json::array();
  bool last_periodic_ready_{false};
  uint64_t last_motion_resets_{0};
  VisualTargetTracker visual_;
  ArmorObservation observation_;
  uint64_t camera_stamp_ = 0;
  int waypoint_ = 0;
  std::map<std::string, Stats> stats_;
  std::map<std::string, uint64_t> phase_counts_;
  std::map<std::string, std::map<std::string, uint64_t>> reasons_;
  std::vector<TeamCommand> commands_;
  std::array<uint64_t, 4> impacts_{};
};
}  // namespace

int main(int argc, char** argv) {
  try {
    const auto o = options(argc, argv);
    cv::setNumThreads(1);
    std::signal(SIGINT, stop); std::signal(SIGTERM, stop);
    Workload work(o);
    Json trace = Json::array();
    const double cpu_start = cpuSeconds();
    const auto start = Clock::now();
    uint64_t tick = 0, previous = 0, completed = 0, skipped = 0, misses = 0;
    double next_resource = 0, next_progress = 10;
    Stats completion, work_times;
    if (o.mode == "micro") {
      for (tick = 0; tick < 3500 && running; ++tick) {
        const double elapsed = std::chrono::duration<double>(Clock::now() - start).count();
        if (elapsed >= o.seconds) break;
        work.tick(tick * 2, tick ? (tick - 1) * 2 : 0, true);
        ++completed;
        if (elapsed >= next_resource) { trace.push_back(resources(elapsed)); next_resource = elapsed + 1; }
        const auto temperature = trace.back().value("temperatures_c", Json::object());
        for (const auto& t : temperature.items())
          if (t.value().get<double>() >= 85) running = 0;
      }
    } else {
      while (running) {
        const auto scheduled = start + std::chrono::microseconds(tick * 10000);
        std::this_thread::sleep_until(scheduled);
        const auto begin = Clock::now();
        const double elapsed = std::chrono::duration<double>(begin - start).count();
        if (elapsed >= o.seconds) break;
        const uint64_t current = static_cast<uint64_t>(elapsed * 100);
        if (current > tick) { skipped += current - tick; tick = current; }
        work.tick(tick, previous);
        previous = tick; ++tick; ++completed;
        const auto end = Clock::now();
        const double duration = std::chrono::duration<double, std::milli>(end - begin).count();
        const double lag = std::chrono::duration<double, std::milli>(end - start).count() - previous * 10;
        work_times.milliseconds.push_back(duration); completion.milliseconds.push_back(lag);
        if (lag > 10) ++misses;
        if (elapsed >= next_resource) { trace.push_back(resources(elapsed)); next_resource = elapsed + 1; }
        if (elapsed >= next_progress) { std::cout << "elapsed=" << elapsed << "s ticks=" << completed << " skipped=" << skipped << std::endl; next_progress += 10; }
        const auto temperature = trace.back().value("temperatures_c", Json::object());
        for (const auto& t : temperature.items()) if (t.value().get<double>() >= 85) running = 0;
      }
    }
    const double elapsed = std::chrono::duration<double>(Clock::now() - start).count();
    auto output = work.results();
    utsname host{}; uname(&host); rusage usage{}; getrusage(RUSAGE_SELF, &usage);
    output["host_arch"] = host.machine; output["host_name"] = host.nodename;
    output["opencv_runtime"] = cv::getVersionString(); output["opencv_headers"] = CV_VERSION;
    output["opencv_threads"] = cv::getNumThreads(); output["peak_rss_kib"] = usage.ru_maxrss;
    output["elapsed_s"] = elapsed; output["process_cpu_seconds"] = cpuSeconds() - cpu_start;
    output["process_cpu_percent"] = 100 * output["process_cpu_seconds"].get<double>() / elapsed;
    output["mode"] = o.mode; output["camera_hz"] = o.camera_hz; output["tof_hz_each"] = o.tof_hz;
    output["uwb_fallback"] = o.uwb_fallback;
    output["requested_duration_s"] = o.seconds; output["completed_ticks"] = completed;
    output["skipped_ticks"] = skipped; output["deadline_misses"] = misses;
    output["tick_work"] = work_times.json(); output["tick_completion_lag"] = completion.json();
    output["resources"] = trace; output["stopped_early"] = !running;
    output["inputs"] = {{"image", o.image}, {"guide_image", o.guide_image}, {"terrain", o.terrain}, {"poses", o.poses}};
    output["workload"] = "Single-vehicle algorithm kernels, three ToF streams, three UWB peers, four-vehicle mission schedule, recorded visual pose replay";
    output["timing_scope"] = "Algorithm calls; camera capture, ToF hardware I/O, MAVLink transport and actuator execution are excluded";
    std::ofstream file(o.output); file << output.dump(2) << '\n';
    if (!file) throw std::runtime_error("Cannot write result");
    std::cout << output.dump(2) << std::endl;
    return running ? 0 : 2;
  } catch (const std::exception& e) { std::cerr << e.what() << std::endl; return 1; }
}
