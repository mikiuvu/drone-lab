/**
 * @file localization_study.cpp
 * @brief RMUC2027 实飞轨迹上的四机定位与通信消融实验。
 * @note 真值日志只含位置与 yaw，roll/pitch 固定为 0，属于仅 yaw 的姿态重建。
 *       传感器与估计器先验共用同一静态地形网格，结构定位精度不代表真实建图误差。
 *       高度辅助使用带噪真值高度，不是真实高度传感器。
 */
#include "companion/estimator.hpp"
#include "companion/terrain.hpp"
#include "tof_error_model.hpp"

#include <Eigen/Geometry>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using namespace companion;
using json = nlohmann::json;

namespace {

constexpr int kVehicles = 4;
constexpr double kLoopStepS = 0.01;
constexpr double kKinematicsHalfS = 0.05;
constexpr double kTofHz = 15.0;
constexpr int kHeightDivisor = 2;
constexpr int kFlowDivisor = 2;
constexpr int kUwbDivisor = 5;
constexpr int kTraceStride = 10;

constexpr double kUwbRangeSigmaM = 0.15;
constexpr double kUwbBearingSigmaRad = 0.08;
constexpr double kImuAccelBiasMps2 = 0.008;
constexpr double kImuAccelWhiteMps2 = 0.02;
constexpr double kImuYawBiasRad = 0.01;
constexpr double kImuYawWhiteRad = 0.002;
constexpr double kHeightWhiteM = 0.01;
constexpr double kFlowScale = 1.005;
constexpr double kFlowWhiteMps = 0.015;
constexpr double kInitOffsetX = 0.08;
constexpr double kInitOffsetY = -0.06;
constexpr double kInitYawOffset = 0.02;
constexpr double kInitCovariance = 0.02;
constexpr double kFalseValidErrorM = 0.1;
constexpr double kIntermittentDropProbability = 0.30;
constexpr double kIntermittentOutageStartS = 8.0;
constexpr double kIntermittentOutageEndS = 16.0;
constexpr double kVehicle0TofBlackoutStartS = 8.0;
constexpr double kVehicle0TofBlackoutEndS = 16.0;
constexpr double kRobotMotionPeriodS = 4.0;

constexpr const char* kHeightSource = "SIM_HEIGHT_ASSIST";

const std::array<const char*, 6> kRequiredScenarios{
    "ideal", "noise", "occluders", "combined", "adverse", "blackout"};
const std::array<const char*, 3> kRequiredProfiles{"ideal", "nominal", "adverse"};

Eigen::Matrix3d rotateZ(double yaw) {
  return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}

// ---------- 确定性指纹 ----------

class Hasher {
 public:
  void bytes(const void* data, size_t size) {
    const auto* raw = static_cast<const uint8_t*>(data);
    for (size_t i = 0; i < size; ++i) {
      hash_ ^= raw[i];
      hash_ *= 1099511628211ull;
    }
  }
  void add(double value) {
    const double finite = std::isfinite(value) ? value : 0.0;
    bytes(&finite, sizeof finite);
  }
  void add(uint64_t value) { bytes(&value, sizeof value); }
  void add(int value) {
    const int64_t wide = value;
    bytes(&wide, sizeof wide);
  }
  uint64_t value() const { return hash_; }

 private:
  uint64_t hash_{1469598103934665603ull};
};

std::string hex64(uint64_t value) {
  std::ostringstream out;
  out << "0x" << std::hex << value;
  return out.str();
}

double percentile(const std::vector<double>& sorted, double q) {
  if (sorted.empty()) return 0.0;
  const double index = q * static_cast<double>(sorted.size() - 1);
  const size_t low = static_cast<size_t>(std::floor(index));
  const size_t high = static_cast<size_t>(std::ceil(index));
  const double fraction = index - static_cast<double>(low);
  return sorted[low] * (1.0 - fraction) + sorted[high] * fraction;
}

// ---------- 轨迹 ----------

struct TrajSample {
  double t_s{0.0};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  double yaw_rad{0.0};
};

class Trajectory {
 public:
  static Trajectory load(const std::string& path) {
    std::ifstream file(path);
    if (!file) throw std::runtime_error("cannot open trajectory: " + path);
    Trajectory trajectory;
    std::string line;
    size_t line_number = 0;
    while (std::getline(file, line)) {
      ++line_number;
      if (line.empty()) continue;
      json record;
      try {
        record = json::parse(line);
      } catch (const std::exception& error) {
        throw std::runtime_error("trajectory line " + std::to_string(line_number) +
                                 " is not JSON: " + error.what());
      }
      if (!record.contains("vehicle")) continue;
      const int vehicle = record.at("vehicle").get<int>();
      if (vehicle < 0 || vehicle >= kVehicles) continue;
      TrajSample sample;
      sample.t_s = record.at("sim_time_us").get<double>() * 1e-6;
      sample.position = Eigen::Vector3d(record.at("x_m").get<double>(),
                                        record.at("y_m").get<double>(),
                                        record.at("z_m").get<double>());
      sample.yaw_rad = record.at("yaw_rad").get<double>();
      trajectory.samples_[vehicle].push_back(sample);
    }
    for (auto& vehicle_samples : trajectory.samples_) {
      if (vehicle_samples.size() < 2) {
        throw std::runtime_error("trajectory has too few samples for a vehicle");
      }
      std::sort(vehicle_samples.begin(), vehicle_samples.end(),
                [](const TrajSample& a, const TrajSample& b) { return a.t_s < b.t_s; });
      // 逐车解包 yaw，保证跨 ±π 的线性插值连续。
      for (size_t i = 1; i < vehicle_samples.size(); ++i) {
        const double previous = vehicle_samples[i - 1].yaw_rad;
        vehicle_samples[i].yaw_rad =
            previous + wrapAngle(vehicle_samples[i].yaw_rad - previous);
      }
    }
    trajectory.t_min_ = trajectory.samples_[0].front().t_s;
    trajectory.t_max_ = trajectory.samples_[0].back().t_s;
    for (const auto& vehicle_samples : trajectory.samples_) {
      trajectory.t_min_ = std::min(trajectory.t_min_, vehicle_samples.front().t_s);
      trajectory.t_max_ = std::max(trajectory.t_max_, vehicle_samples.back().t_s);
    }
    return trajectory;
  }

  double tMin() const { return t_min_; }
  double tMax() const { return t_max_; }

  Eigen::Vector3d position(int vehicle, double t_s) const {
    const auto& samples = samples_[vehicle];
    const double clamped = std::min(std::max(t_s, samples.front().t_s), samples.back().t_s);
    const auto upper = std::upper_bound(
        samples.begin(), samples.end(), clamped,
        [](double value, const TrajSample& sample) { return value < sample.t_s; });
    if (upper == samples.begin()) return samples.front().position;
    if (upper == samples.end()) return samples.back().position;
    const TrajSample& high = *upper;
    const TrajSample& low = *(upper - 1);
    const double fraction = (clamped - low.t_s) / (high.t_s - low.t_s);
    return low.position + fraction * (high.position - low.position);
  }

  double yaw(int vehicle, double t_s) const {
    const auto& samples = samples_[vehicle];
    const double clamped = std::min(std::max(t_s, samples.front().t_s), samples.back().t_s);
    const auto upper = std::upper_bound(
        samples.begin(), samples.end(), clamped,
        [](double value, const TrajSample& sample) { return value < sample.t_s; });
    if (upper == samples.begin()) return wrapAngle(samples.front().yaw_rad);
    if (upper == samples.end()) return wrapAngle(samples.back().yaw_rad);
    const TrajSample& high = *upper;
    const TrajSample& low = *(upper - 1);
    const double fraction = (clamped - low.t_s) / (high.t_s - low.t_s);
    return wrapAngle(low.yaw_rad + fraction * (high.yaw_rad - low.yaw_rad));
  }

  // 中心差分跨度为 2*kKinematicsHalfS，抑制真值日志间隔抖动带来的伪加速度。
  Eigen::Vector3d velocity(int vehicle, double t_s) const {
    return (position(vehicle, t_s + kKinematicsHalfS) -
            position(vehicle, t_s - kKinematicsHalfS)) /
           (2.0 * kKinematicsHalfS);
  }

  Eigen::Vector3d acceleration(int vehicle, double t_s) const {
    return (position(vehicle, t_s + kKinematicsHalfS) -
            2.0 * position(vehicle, t_s) +
            position(vehicle, t_s - kKinematicsHalfS)) /
           (kKinematicsHalfS * kKinematicsHalfS);
  }

  size_t sampleCount(int vehicle) const { return samples_[vehicle].size(); }

 private:
  std::array<std::vector<TrajSample>, kVehicles> samples_;
  double t_min_{0.0};
  double t_max_{0.0};
};

// ---------- 场景与噪声剖面 ----------

struct Profile {
  std::string name;
  double noise_std_m{0.0};
  double range_sigma_per_m{0.0};
  double bias_bound_m{0.0};
  double scale_bound{0.0};
  double frame_bias_std_m{0.0};
  double drop_probability{0.0};
};

struct Robot {
  std::string id;
  std::string role;
  Eigen::Vector3d center{Eigen::Vector3d::Zero()};
  Eigen::Vector3d size{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  bool moving{false};
};

// 共享 schema 的 ToF 几何：sensor 与 estimator 先验使用同一份外参。
struct TofGeometry {
  std::array<double, 3> yaw_rad{0.0, 2.0 * kPi / 3.0, -2.0 * kPi / 3.0};
  double radius_m{0.055};
  double horizontal_fov_rad{kPi / 4.0};
  double vertical_fov_rad{kPi / 4.0};
  double pitch_down_rad{0.0};
  double min_range_m{0.02};
  double max_range_m{4.0};

  TofMount mount(int sensor) const {
    TofMount mount;
    mount.yaw_rad = yaw_rad[static_cast<size_t>(sensor)];
    mount.roll_rad = 0.0;
    mount.pitch_down_rad = pitch_down_rad;
    mount.translation_body_m = rotateZ(mount.yaw_rad) * Eigen::Vector3d(radius_m, 0.0, 0.0);
    mount.horizontal_fov_rad = horizontal_fov_rad;
    mount.vertical_fov_rad = vertical_fov_rad;
    mount.min_range_m = min_range_m;
    mount.max_range_m = max_range_m;
    return mount;
  }
};

struct Scenario {
  std::string name;
  std::string profile_name;
  Profile profile;
  std::vector<Robot> robots;
  bool vehicle0_tof_blackout{false};
};

Profile defaultProfile(const std::string& name) {
  Profile profile;
  profile.name = name;
  if (name == "ideal") return profile;
  if (name == "nominal") {
    profile.noise_std_m = 0.02;
    profile.range_sigma_per_m = 0.01;
    profile.bias_bound_m = 0.02;
    profile.scale_bound = 0.01;
    profile.frame_bias_std_m = 0.01;
    profile.drop_probability = 0.02;
    return profile;
  }
  if (name == "adverse") {
    profile.noise_std_m = 0.08;
    profile.range_sigma_per_m = 0.04;
    profile.bias_bound_m = 0.08;
    profile.scale_bound = 0.05;
    profile.frame_bias_std_m = 0.05;
    profile.drop_probability = 0.15;
    return profile;
  }
  throw std::runtime_error("unknown profile: " + name);
}

Profile parseProfile(const std::string& name, const json& object) {
  Profile profile = defaultProfile(name);
  const auto read = [&object](const char* key, double& target) {
    if (object.contains(key)) target = object.at(key).get<double>();
  };
  read("noise_std_m", profile.noise_std_m);
  read("range_sigma_per_m", profile.range_sigma_per_m);
  read("bias_bound_m", profile.bias_bound_m);
  read("scale_bound", profile.scale_bound);
  read("frame_bias_std_m", profile.frame_bias_std_m);
  read("drop_probability", profile.drop_probability);
  return profile;
}

// 把共享 schema 的剖面转成 tof_error_model.hpp 的等价结构，保证离线与 gateway 同源。
drone_sim::research::TofNoiseProfile toSharedTofProfile(const Profile& profile) {
  drone_sim::research::TofNoiseProfile shared;
  shared.noise_std_m = profile.noise_std_m;
  shared.range_sigma_per_m = profile.range_sigma_per_m;
  shared.bias_bound_m = profile.bias_bound_m;
  shared.scale_bound = profile.scale_bound;
  shared.frame_bias_std_m = profile.frame_bias_std_m;
  shared.drop_probability = profile.drop_probability;
  return shared;
}

Robot parseRobot(const json& object) {
  Robot robot;
  if (object.contains("id")) {
    robot.id = object.at("id").is_string() ? object.at("id").get<std::string>()
                                           : std::to_string(object.at("id").get<int>());
  }
  robot.role = object.value("role", std::string("robot"));
  const auto readVector = [&object](const char* key, Eigen::Vector3d& target) {
    if (!object.contains(key)) return;
    const auto& values = object.at(key);
    if (values.size() != 3) throw std::runtime_error(std::string(key) + " needs 3 values");
    target = Eigen::Vector3d(values[0].get<double>(), values[1].get<double>(),
                             values[2].get<double>());
  };
  readVector("center_m", robot.center);
  readVector("size_m", robot.size);
  if (object.contains("velocity_mps")) {
    readVector("velocity_mps", robot.velocity);
    robot.moving = robot.velocity.squaredNorm() > 0.0;
  }
  if (robot.size.minCoeff() <= 0.0) throw std::runtime_error("robot size_m must be positive");
  return robot;
}

std::vector<Robot> defaultRobots(const std::string& set) {
  std::vector<Robot> robots;
  if (set == "route") {
    robots.push_back({"route_hero_0", "hero", {0.6, -2.4, 0.5371}, {0.6, 0.5, 0.6}, {}, false});
    robots.push_back(
        {"route_infantry_1", "infantry", {2.2, -2.4, 0.4631}, {0.45, 0.4, 0.45}, {}, false});
    robots.push_back(
        {"route_engineer_2", "engineer", {3.6, -2.4, 0.6971}, {0.65, 0.55, 0.8}, {}, false});
    robots.push_back(
        {"route_sentry_3", "sentry", {5.2, -2.4, 0.2371}, {0.6, 0.5, 0.6}, {}, false});
    robots.push_back(
        {"route_infantry_4", "infantry", {6.8, -2.4, 0.1621}, {0.45, 0.4, 0.45}, {}, false});
    robots.push_back({"route_hero_5", "hero", {8.2, -2.4, 0.2371}, {0.6, 0.5, 0.6}, {}, false});
    robots.push_back(
        {"route_engineer_6", "engineer", {9.0, -2.4, 0.3371}, {0.65, 0.55, 0.8}, {}, false});
  } else if (set == "tray_near") {
    robots.push_back(
        {"tray_infantry_7", "infantry", {-1.5, -1.0, 0.4988}, {0.45, 0.4, 0.45}, {}, false});
  } else if (set == "contact_near") {
    robots.push_back(
        {"contact_sentry_8", "sentry", {10.8, -0.2, 0.5}, {0.6, 0.5, 0.6}, {}, false});
  } else if (set == "all") {
    for (const char* part : {"route", "tray_near", "contact_near"}) {
      const auto more = defaultRobots(part);
      robots.insert(robots.end(), more.begin(), more.end());
    }
  }
  return robots;
}

std::vector<Scenario> defaultScenarios() {
  std::vector<Scenario> scenarios;
  const auto make = [](const std::string& name, const std::string& profile_name,
                       std::vector<Robot> robots, bool blackout) {
    Scenario scenario;
    scenario.name = name;
    scenario.profile_name = profile_name;
    scenario.profile = defaultProfile(profile_name);
    scenario.robots = std::move(robots);
    scenario.vehicle0_tof_blackout = blackout;
    return scenario;
  };
  scenarios.push_back(make("ideal", "ideal", {}, false));
  scenarios.push_back(make("noise", "nominal", {}, false));
  scenarios.push_back(make("occluders", "ideal", defaultRobots("all"), false));
  scenarios.push_back(make("combined", "nominal", defaultRobots("all"), false));
  scenarios.push_back(make("adverse", "adverse", defaultRobots("all"), false));
  scenarios.push_back(make("blackout", "nominal", defaultRobots("all"), true));
  return scenarios;
}

struct ScenarioLoad {
  std::vector<Scenario> scenarios;
  std::map<std::string, Profile> profiles;
  TofGeometry tof;
  std::vector<std::string> available_groups;
  std::vector<std::string> excluded_groups;
  std::string default_profile{"nominal"};
  bool used_defaults{false};
};

// 共享 schema：simulation/research/scenarios.json。缺失时仅在 --allow-default-scenarios 下回退。
ScenarioLoad loadScenarios(const std::string& path, bool allow_default) {
  std::ifstream file(path);
  if (!file) {
    if (!allow_default) {
      throw std::runtime_error(
          "scenarios file not found: " + path +
          " (pass --allow-default-scenarios only for smoke runs)");
    }
    ScenarioLoad load;
    load.scenarios = defaultScenarios();
    for (const Scenario& scenario : load.scenarios) {
      load.profiles[scenario.profile_name] = scenario.profile;
    }
    load.available_groups = {"route", "tray_near", "contact_near"};
    load.used_defaults = true;
    std::cerr << "warning: using built-in scenario defaults, " << path << " is absent\n";
    return load;
  }
  json document;
  file >> document;
  if (!document.is_object()) throw std::runtime_error("scenarios root must be an object");
  const int schema = document.contains("schema") ? document.at("schema").get<int>()
                                                 : document.value("schema_version", 0);
  if (schema != 1) {
    throw std::runtime_error("scenarios schema must be 1");
  }

  ScenarioLoad load;
  for (const char* name : kRequiredProfiles) {
    load.profiles[name] = defaultProfile(name);
  }
  if (document.contains("profiles")) {
    for (auto it = document.at("profiles").begin(); it != document.at("profiles").end(); ++it) {
      load.profiles[it.key()] = parseProfile(it.key(), it.value());
    }
  }

  std::map<std::string, Robot> robot_by_id;
  std::vector<Robot> declared_robots;
  if (document.contains("robots")) {
    for (const auto& entry : document.at("robots")) {
      Robot robot = parseRobot(entry);
      if (!robot.id.empty()) robot_by_id[robot.id] = robot;
      declared_robots.push_back(robot);
    }
  }
  const json research = document.contains("research") ? document.at("research") : json::object();
  load.default_profile = research.value("default_profile", std::string("nominal"));
  if (!load.profiles.count(load.default_profile)) load.default_profile = "nominal";

  std::map<std::string, std::vector<std::string>> groups;
  if (research.contains("scenario_groups")) {
    for (auto it = research.at("scenario_groups").begin();
         it != research.at("scenario_groups").end(); ++it) {
      std::vector<std::string> ids;
      for (const auto& id : it.value()) ids.push_back(id.get<std::string>());
      groups[it.key()] = std::move(ids);
    }
  }
  if (research.contains("pressure_robots")) {
    for (const auto& entry : research.at("pressure_robots")) {
      Robot robot = parseRobot(entry);
      if (!robot.id.empty()) robot_by_id[robot.id] = robot;
    }
  }
  if (research.contains("tof_geometry")) {
    const json& geometry = research.at("tof_geometry");
    const auto read = [&geometry](const char* key, double& target) {
      if (geometry.contains(key)) target = geometry.at(key).get<double>();
    };
    read("hfov_rad", load.tof.horizontal_fov_rad);
    read("vfov_rad", load.tof.vertical_fov_rad);
    read("pitch_down_rad", load.tof.pitch_down_rad);
    read("min_range_m", load.tof.min_range_m);
    read("max_range_m", load.tof.max_range_m);
  }

  const auto resolve = [&](const std::string& name) {
    std::vector<Robot> robots;
    const auto group = groups.find(name);
    if (group == groups.end()) return robots;
    for (const std::string& id : group->second) {
      const auto robot = robot_by_id.find(id);
      if (robot == robot_by_id.end()) throw std::runtime_error("robot id not found: " + id);
      robots.push_back(robot->second);
    }
    return robots;
  };
  for (const auto& group : groups) load.available_groups.push_back(group.first);
  load.excluded_groups = {"tray_pressure"};

  const auto make = [&](const std::string& name, const std::string& profile_name,
                        std::vector<Robot> robots, bool blackout) {
    Scenario scenario;
    scenario.name = name;
    scenario.profile_name = profile_name;
    scenario.profile = load.profiles.at(profile_name);
    scenario.robots = std::move(robots);
    scenario.vehicle0_tof_blackout = blackout;
    return scenario;
  };

  if (document.contains("scenarios")) {
    for (const auto& entry : document.at("scenarios")) {
      const std::string name = entry.at("name").get<std::string>();
      const std::string profile_name = entry.value("profile", load.default_profile);
      if (!load.profiles.count(profile_name)) {
        throw std::runtime_error("scenario " + name + " uses unknown profile " + profile_name);
      }
      std::vector<Robot> robots;
      if (entry.contains("robots")) {
        const auto& declared = entry.at("robots");
        if (declared.is_string()) {
          robots = resolve(declared.get<std::string>());
        } else if (declared.is_array()) {
          for (const auto& robot : declared) robots.push_back(parseRobot(robot));
        } else {
          throw std::runtime_error("scenario robots must be a group name or an array");
        }
      }
      load.scenarios.push_back(make(name, profile_name, std::move(robots),
                                    entry.value("vehicle0_tof_blackout", false)));
    }
  } else {
    // 共享契约：四个含机器人场景统一使用顶层 robots 全量静态占位，差异来自噪声剖面与黑障。
    // pressure_robots 是托盘平台上的非比赛站位，只参与显式分组，不进入六个场景。
    std::vector<Robot> all_robots = declared_robots;
    std::sort(all_robots.begin(), all_robots.end(),
              [](const Robot& first, const Robot& second) { return first.id < second.id; });
    load.scenarios.push_back(make("ideal", "ideal", {}, false));
    load.scenarios.push_back(make("noise", load.default_profile, {}, false));
    load.scenarios.push_back(make("occluders", "ideal", all_robots, false));
    load.scenarios.push_back(make("combined", load.default_profile, all_robots, false));
    load.scenarios.push_back(make("adverse", "adverse", all_robots, false));
    load.scenarios.push_back(make("blackout", load.default_profile, all_robots, true));
  }

  for (const char* required : kRequiredScenarios) {
    const bool present = std::any_of(
        load.scenarios.begin(), load.scenarios.end(),
        [required](const Scenario& scenario) { return scenario.name == required; });
    if (!present) throw std::runtime_error(std::string("scenario missing from file: ") + required);
  }
  return load;
}

// ---------- 传感器输入 ----------

struct Truth {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
  double yaw{0.0};
};

struct SensorBundle {
  double t_s{0.0};
  std::array<Truth, kVehicles> truth;
  std::array<ImuSample, kVehicles> imu;
  std::array<HeightAssistSample, kVehicles> height;
  std::array<FlowSample, kVehicles> flow;
  std::array<std::array<TofFrame, 3>, kVehicles> tof;
  std::array<std::array<UwbPeerSample, kVehicles>, kVehicles> uwb;
  std::array<std::array<uint8_t, kVehicles>, kVehicles> uwb_drop{};
  bool has_height{false};
  bool has_flow{false};
  bool has_tof{false};
  bool has_uwb{false};
};

struct SensorStats {
  long long rays{0};
  long long robot_occluded{0};
  long long no_return{0};
  long long dropped{0};
  long long out_of_range{0};
  long long blackout{0};
  long long valid_returns{0};
  std::vector<float> terrain_error;

  double occludedFraction() const {
    return rays ? static_cast<double>(robot_occluded) / static_cast<double>(rays) : 0.0;
  }
};

struct InputSet {
  std::vector<SensorBundle> bundles;
  SensorStats stats;
  uint64_t checksum{0};
};

struct DeviceConstants {
  std::array<Eigen::Vector3d, kVehicles> accel_bias;
  std::array<double, kVehicles> yaw_bias;
};

double triangleWave(double t_s) {
  double phase = std::fmod(t_s, kRobotMotionPeriodS) / kRobotMotionPeriodS;
  if (phase < 0.0) phase += 1.0;
  return 4.0 * std::abs(phase - 0.5) - 1.0;
}

constexpr double kNoRobotHit = std::numeric_limits<double>::infinity();

double robotDepth(const Robot& robot, double t_s, const Eigen::Vector3d& origin,
                  const Eigen::Vector3d& ray, double max_depth) {
  Eigen::Vector3d center = robot.center;
  if (robot.moving) center += robot.velocity * triangleWave(t_s);
  const Eigen::Vector3d low = center - 0.5 * robot.size;
  const Eigen::Vector3d high = center + 0.5 * robot.size;
  double near_depth = 1e-6;
  double far_depth = max_depth;
  for (int axis = 0; axis < 3; ++axis) {
    if (std::abs(ray(axis)) < 1e-12) {
      if (origin(axis) < low(axis) || origin(axis) > high(axis)) return kNoRobotHit;
      continue;
    }
    double first = (low(axis) - origin(axis)) / ray(axis);
    double second = (high(axis) - origin(axis)) / ray(axis);
    if (first > second) std::swap(first, second);
    near_depth = std::max(near_depth, first);
    far_depth = std::min(far_depth, second);
    if (near_depth > far_depth) return kNoRobotHit;
  }
  return near_depth;
}

DeviceConstants makeDeviceConstants(int seed) {
  std::mt19937 rng(static_cast<uint32_t>(seed) * 2654435761u + 12345u);
  std::uniform_real_distribution<double> unit(-1.0, 1.0);
  DeviceConstants constants;
  for (int vehicle = 0; vehicle < kVehicles; ++vehicle) {
    Eigen::Vector3d direction(unit(rng), unit(rng), 0.0);
    if (direction.norm() < 1e-6) direction = Eigen::Vector3d(1.0, 0.0, 0.0);
    constants.accel_bias[vehicle] = direction.normalized() * kImuAccelBiasMps2;
    constants.yaw_bias[vehicle] = unit(rng) * kImuYawBiasRad;
  }
  return constants;
}

struct Window {
  std::string name;
  double start_s{0.0};
  double seconds{0.0};
  bool clamped{false};
};

InputSet generateInputs(const Trajectory& trajectory, const Scenario& scenario,
                        const std::shared_ptr<const TerrainMap>& map, int seed,
                        const Window& window, int steps, const TofGeometry& tof) {
  const Profile& profile = scenario.profile;
  const DeviceConstants device = makeDeviceConstants(seed);
  std::mt19937 noise(static_cast<uint32_t>(seed) * 1000003u + 7919u);
  std::normal_distribution<double> normal(0.0, 1.0);
  std::uniform_real_distribution<double> uniform(0.0, 1.0);

  const std::array<TofMount, 3> mounts{tof.mount(0), tof.mount(1), tof.mount(2)};
  const drone_sim::research::TofNoiseProfile tof_profile = toSharedTofProfile(profile);
  if (!drone_sim::research::TofProfileIsValid(tof_profile)) {
    throw std::runtime_error("invalid ToF noise profile: " + profile.name);
  }
  // 设备固定偏差与比例由 (seed, vehicle, sensor) 派生，整段运行恒定。
  std::array<std::array<drone_sim::research::TofNoiseDevice, 3>, kVehicles> tof_devices;
  for (int vehicle = 0; vehicle < kVehicles; ++vehicle) {
    for (int sensor = 0; sensor < 3; ++sensor) {
      tof_devices[vehicle][sensor] = drone_sim::research::MakeTofNoiseDevice(
          tof_profile, static_cast<uint64_t>(seed), vehicle, sensor);
    }
  }

  InputSet input;
  input.bundles.reserve(static_cast<size_t>(steps) + 1);
  Hasher hasher;
  int tof_index = 0;

  for (int tick = 0; tick <= steps; ++tick) {
    SensorBundle bundle;
    bundle.t_s = window.start_s + tick * kLoopStepS;
    const double t_s = bundle.t_s;
    const uint64_t stamp = static_cast<uint64_t>(std::llround(t_s * 1e6));

    for (int vehicle = 0; vehicle < kVehicles; ++vehicle) {
      Truth& truth = bundle.truth[vehicle];
      truth.position = trajectory.position(vehicle, t_s);
      truth.velocity = trajectory.velocity(vehicle, t_s);
      truth.acceleration = trajectory.acceleration(vehicle, t_s);
      truth.yaw = trajectory.yaw(vehicle, t_s);
    }

    for (int vehicle = 0; vehicle < kVehicles; ++vehicle) {
      const Truth& truth = bundle.truth[vehicle];
      ImuSample& imu = bundle.imu[vehicle];
      imu.sim_time_us = stamp;
      imu.attitude_valid = true;
      imu.roll_flu_rad = 0.0;
      imu.pitch_flu_rad = 0.0;
      imu.yaw_enu_rad = truth.yaw + device.yaw_bias[vehicle] + kImuYawWhiteRad * normal(noise);
      imu.acceleration_valid = true;
      imu.acceleration_enu_mps2 =
          truth.acceleration + device.accel_bias[vehicle] +
          Eigen::Vector3d(kImuAccelWhiteMps2 * normal(noise), kImuAccelWhiteMps2 * normal(noise),
                          0.0);
      imu.pressure_valid = false;
    }

    if (tick % kHeightDivisor == 0) {
      bundle.has_height = true;
      for (int vehicle = 0; vehicle < kVehicles; ++vehicle) {
        HeightAssistSample& height = bundle.height[vehicle];
        height.sim_time_us = stamp;
        height.z_world_m = bundle.truth[vehicle].position.z() + kHeightWhiteM * normal(noise);
        height.source = kHeightSource;
      }
    }

    if (tick % kFlowDivisor == 0) {
      bundle.has_flow = true;
      for (int vehicle = 0; vehicle < kVehicles; ++vehicle) {
        const Truth& truth = bundle.truth[vehicle];
        const Eigen::Vector3d body = rotateZ(-truth.yaw) * truth.velocity;
        FlowSample& flow = bundle.flow[vehicle];
        flow.sim_time_us = stamp;
        flow.vx_body_mps = kFlowScale * body.x() + kFlowWhiteMps * normal(noise);
        flow.vy_body_mps = kFlowScale * body.y() + kFlowWhiteMps * normal(noise);
        flow.quality = 1.0;
        flow.variance_m2s2 = kFlowWhiteMps * kFlowWhiteMps;
      }
    }

    const double tof_period = 1.0 / kTofHz;
    while (static_cast<double>(tof_index) * tof_period <= tick * kLoopStepS + 1e-9) {
      const double tof_t = window.start_s + static_cast<double>(tof_index) * tof_period;
      if (tof_t > window.start_s + window.seconds + 1e-9) break;
      bundle.has_tof = true;
      const uint64_t tof_stamp = static_cast<uint64_t>(std::llround(tof_t * 1e6));
      for (int vehicle = 0; vehicle < kVehicles; ++vehicle) {
        const Truth& truth = bundle.truth[vehicle];
        const Eigen::Matrix3d rotation = rotateZ(truth.yaw);
        const bool blackout = scenario.vehicle0_tof_blackout && vehicle == 0 &&
                              (t_s >= kVehicle0TofBlackoutStartS) &&
                              (t_s < kVehicle0TofBlackoutEndS);
        for (int sensor = 0; sensor < 3; ++sensor) {
          TofFrame& frame = bundle.tof[vehicle][sensor];
          frame.sim_time_us = tof_stamp;
          frame.sensor_index = sensor;
          const TofMount& mount = mounts[sensor];
          const Eigen::Vector3d origin = truth.position + rotation * mount.translation_body_m;
          const drone_sim::research::TofFrameError frame_error =
              drone_sim::research::MakeTofFrameError(tof_profile, static_cast<uint64_t>(seed),
                                                     vehicle, sensor,
                                                     static_cast<int64_t>(tof_stamp));
          for (int cell = 0; cell < kTofCellCount; ++cell) {
            ++input.stats.rays;
            const int row = cell / 8;
            const int col = cell % 8;
            const Eigen::Vector3d body_ray = Estimator::tofRayBody(mount, row, col);
            const Eigen::Vector3d ray = rotation * body_ray;
            TerrainMap::Hit hit;
            const bool terrain_hit = map->raycast(origin, ray, mount.max_range_m, hit);
            const double terrain_depth = terrain_hit ? hit.depth_m : kNoRobotHit;
            double robot_depth = kNoRobotHit;
            for (const Robot& robot : scenario.robots) {
              robot_depth = std::min(robot_depth,
                                     robotDepth(robot, t_s, origin, ray, mount.max_range_m));
            }
            const bool blocked = robot_depth < terrain_depth;
            const double true_depth = blocked ? robot_depth : terrain_depth;
            if (blocked) ++input.stats.robot_occluded;
            if (!std::isfinite(true_depth)) {
              ++input.stats.no_return;
              frame.valid[cell] = 0;
              continue;
            }
            if (blackout) {
              ++input.stats.blackout;
              frame.valid[cell] = 0;
              continue;
            }
            const drone_sim::research::TofPixelResult pixel = drone_sim::research::ApplyTofPixel(
                tof_profile, tof_devices[vehicle][sensor], frame_error, true_depth, cell,
                mount.min_range_m, mount.max_range_m,
                drone_sim::research::TofRangePolicy::kMarkInvalid);
            if (!pixel.valid) {
              // 无效来源用共享模型自身的键区分，量测本身不改动。
              const uint64_t key = drone_sim::research::TofPixelKey(frame_error.frame_key, cell);
              if (drone_sim::research::TofUniform(key ^ 0x31ULL) < tof_profile.drop_probability) {
                ++input.stats.dropped;
              } else {
                ++input.stats.out_of_range;
              }
              frame.valid[cell] = 0;
              continue;
            }
            frame.distance_m[cell] = pixel.distance_m;
            frame.valid[cell] = 1;
            ++input.stats.valid_returns;
            if (!blocked) {
              input.stats.terrain_error.push_back(
                  static_cast<float>(pixel.distance_m - terrain_depth));
            }
          }
        }
      }
      ++tof_index;
    }

    if (tick % kUwbDivisor == 0) {
      bundle.has_uwb = true;
      for (int vehicle = 0; vehicle < kVehicles; ++vehicle) {
        for (int peer = 0; peer < kVehicles; ++peer) {
          if (peer == vehicle) continue;
          const Eigen::Vector3d delta =
              bundle.truth[peer].position - bundle.truth[vehicle].position;
          const Eigen::Vector3d body = rotateZ(-bundle.truth[vehicle].yaw) * delta;
          UwbPeerSample& sample = bundle.uwb[vehicle][peer];
          sample.sim_time_us = stamp;
          sample.peer_id = peer;
          sample.range_m = delta.norm() + kUwbRangeSigmaM * normal(noise);
          sample.bearing_rad = wrapAngle(std::atan2(body.y(), body.x()) +
                                         kUwbBearingSigmaRad * normal(noise));
          sample.valid = true;
          sample.quality = 1.0;
          sample.nlos = false;
          const bool outage = (t_s >= kIntermittentOutageStartS) && (t_s < kIntermittentOutageEndS) &&
                              (vehicle == 0 || peer == 0);
          bundle.uwb_drop[vehicle][peer] =
              (outage || uniform(noise) < kIntermittentDropProbability) ? 1 : 0;
        }
      }
    }

    // 指纹只覆盖量测本身，与通信模式、融合模式无关。
    for (int vehicle = 0; vehicle < kVehicles; ++vehicle) {
      const Truth& truth = bundle.truth[vehicle];
      hasher.add(truth.position.x());
      hasher.add(truth.position.y());
      hasher.add(truth.position.z());
      hasher.add(truth.yaw);
      const ImuSample& imu = bundle.imu[vehicle];
      hasher.add(imu.sim_time_us);
      hasher.add(imu.yaw_enu_rad);
      hasher.add(imu.acceleration_enu_mps2.x());
      hasher.add(imu.acceleration_enu_mps2.y());
      if (bundle.has_height) {
        hasher.add(bundle.height[vehicle].sim_time_us);
        hasher.add(bundle.height[vehicle].z_world_m);
      }
      if (bundle.has_flow) {
        hasher.add(bundle.flow[vehicle].sim_time_us);
        hasher.add(bundle.flow[vehicle].vx_body_mps);
        hasher.add(bundle.flow[vehicle].vy_body_mps);
      }
      if (bundle.has_tof) {
        for (int sensor = 0; sensor < 3; ++sensor) {
          const TofFrame& frame = bundle.tof[vehicle][sensor];
          hasher.add(frame.sim_time_us);
          hasher.add(frame.sensor_index);
          for (int cell = 0; cell < kTofCellCount; ++cell) {
            hasher.add(frame.distance_m[cell]);
            hasher.add(static_cast<int>(frame.valid[cell]));
          }
        }
      }
      if (bundle.has_uwb) {
        for (int peer = 0; peer < kVehicles; ++peer) {
          if (peer == vehicle) continue;
          const UwbPeerSample& sample = bundle.uwb[vehicle][peer];
          hasher.add(sample.sim_time_us);
          hasher.add(sample.peer_id);
          hasher.add(sample.range_m);
          hasher.add(sample.bearing_rad);
          hasher.add(static_cast<int>(sample.valid));
        }
      }
    }
    input.bundles.push_back(std::move(bundle));
  }
  input.checksum = hasher.value();
  return input;
}

// ---------- 估计器配置 ----------

EstimatorConfig makeConfig(int vehicle, const Eigen::Vector3d& initial_position,
                           double initial_yaw, const std::shared_ptr<const TerrainMap>& map,
                           const TofGeometry& tof) {
  EstimatorConfig config;
  config.vehicle_id = vehicle;
  config.initial_pose.x_m = initial_position.x();
  config.initial_pose.y_m = initial_position.y();
  config.initial_pose.z_m = initial_position.z();
  config.initial_pose.roll_rad = 0.0;
  config.initial_pose.pitch_rad = 0.0;
  config.initial_pose.yaw_rad = initial_yaw;
  config.initial_covariance = Eigen::Matrix<double, 7, 7>::Identity() * kInitCovariance;
  config.terrain = map;
  config.use_baro_fallback = false;
  for (int sensor = 0; sensor < 3; ++sensor) {
    config.tof_mounts[sensor] = tof.mount(sensor);
  }
  return config;
}

PeerEstimate snapshot(const Estimate& estimate) {
  PeerEstimate peer;
  peer.sim_time_us = estimate.sim_time_us;
  peer.vehicle_id = estimate.vehicle_id;
  peer.pose = estimate.pose;
  peer.velocity_enu_mps = estimate.velocity_enu_mps;
  peer.pose_covariance = estimate.covariance.topLeftCorner<3, 3>();
  peer.height_assisted = estimate.height_assisted;
  peer.xy_structural_valid = estimate.xy_structural_valid;
  peer.xy_peer_valid = estimate.xy_peer_valid;
  peer.reference_origin = estimate.reference_origin;
  peer.reference_seq = estimate.reference_seq;
  peer.reference_time_us = estimate.reference_time_us;
  peer.reference_hops = estimate.reference_hops;
  peer.reference_path = estimate.reference_path;
  peer.source = "ESTIMATE";
  return peer;
}

// ---------- 单次运行 ----------

struct RunSpec {
  std::string window;
  std::string scenario;
  std::string profile;
  int seed{0};
  std::string comm;
  std::string fusion;
  bool use_flow{false};
};

struct RunResult {
  json row;
  std::string trace;
};

struct VehicleAccumulator {
  std::vector<double> error;
  int count{0};
  int valid{0};
  int structural{0};
  int current_invalid{0};
  int longest_invalid{0};
};

RunResult runVariant(const InputSet& input, const RunSpec& spec,
                     const std::shared_ptr<const TerrainMap>& map,
                     const TofGeometry& tof,
                     const std::array<Eigen::Vector3d, kVehicles>& initial_position,
                     const std::array<double, kVehicles>& initial_yaw,
                     const std::array<Eigen::Vector3d, kVehicles>& initial_truth,
                     const std::array<double, kVehicles>& initial_truth_yaw,
                     double evaluation_start_s, int delay_ticks) {
  std::array<std::unique_ptr<Estimator>, kVehicles> estimators;
  for (int vehicle = 0; vehicle < kVehicles; ++vehicle) {
    estimators[vehicle] = std::make_unique<Estimator>(
        makeConfig(vehicle, initial_position[vehicle], initial_yaw[vehicle], map, tof));
  }

  std::array<VehicleAccumulator, kVehicles> vehicles;
  std::vector<double> error_all;
  int samples = 0;
  int valid_all = 0;
  int structural_all = 0;
  int peer_valid_all = 0;
  int false_valid = 0;
  double maximum_error = 0.0;
  double common_squared = 0.0;
  int common_count = 0;
  double relative_squared = 0.0;
  int relative_count = 0;
  long long tof_calls = 0;
  long long tof_accepted = 0;
  long long uwb_calls = 0;
  long long uwb_accepted = 0;
  std::map<std::string, long long> uwb_rejections;

  std::deque<std::pair<long long, std::array<PeerEstimate, kVehicles>>> pending;
  std::array<PeerEstimate, kVehicles> delivered{};
  bool has_delivered = false;

  std::ostringstream trace;
  const bool want_trace = true;

  for (size_t index = 0; index < input.bundles.size(); ++index) {
    const SensorBundle& bundle = input.bundles[index];
    const double t_s = bundle.t_s;

    for (int vehicle = 0; vehicle < kVehicles; ++vehicle) {
      estimators[vehicle]->predict(bundle.imu[vehicle]);
    }
    if (bundle.has_height) {
      for (int vehicle = 0; vehicle < kVehicles; ++vehicle) {
        estimators[vehicle]->fuseHeightAssist(bundle.height[vehicle]);
      }
    }
    if (bundle.has_tof) {
      for (int vehicle = 0; vehicle < kVehicles; ++vehicle) {
        for (int sensor = 0; sensor < 3; ++sensor) {
          ++tof_calls;
          const FusionReport report = estimators[vehicle]->fuseTof(bundle.tof[vehicle][sensor]);
          if (report.accepted) ++tof_accepted;
        }
      }
    }
    if (spec.use_flow && bundle.has_flow) {
      for (int vehicle = 0; vehicle < kVehicles; ++vehicle) {
        estimators[vehicle]->fuseOpticalFlow(bundle.flow[vehicle]);
      }
    }

    if (spec.comm != "off") {
      std::array<PeerEstimate, kVehicles> current;
      for (int vehicle = 0; vehicle < kVehicles; ++vehicle) {
        current[vehicle] = snapshot(estimators[vehicle]->estimate());
      }
      if (delay_ticks > 0) {
        pending.emplace_back(static_cast<long long>(index) + delay_ticks, current);
        while (!pending.empty() && pending.front().first <= static_cast<long long>(index)) {
          delivered = pending.front().second;
          has_delivered = true;
          pending.pop_front();
        }
      } else {
        delivered = current;
        has_delivered = true;
      }
    }

    if (spec.comm != "off" && bundle.has_uwb && has_delivered) {
      for (int vehicle = 0; vehicle < kVehicles; ++vehicle) {
        for (int peer = 0; peer < kVehicles; ++peer) {
          if (peer == vehicle) continue;
          if (spec.comm == "intermittent" && bundle.uwb_drop[vehicle][peer]) continue;
          ++uwb_calls;
          const FusionReport report =
              estimators[vehicle]->fuseUwb(bundle.uwb[vehicle][peer], delivered[peer]);
          if (report.accepted) {
            ++uwb_accepted;
          } else {
            ++uwb_rejections[report.reason];
          }
        }
      }
    }

    if (t_s + 1e-9 < evaluation_start_s) continue;

    std::array<Eigen::Vector2d, kVehicles> estimated{};
    Eigen::Vector2d mean_error = Eigen::Vector2d::Zero();
    for (int vehicle = 0; vehicle < kVehicles; ++vehicle) {
      const Estimate estimate = estimators[vehicle]->estimate();
      if (!std::isfinite(estimate.pose.x_m) || !std::isfinite(estimate.pose.y_m) ||
          !std::isfinite(estimate.pose.z_m) || !std::isfinite(estimate.pose.yaw_rad)) {
        throw std::runtime_error("non-finite estimate for vehicle " + std::to_string(vehicle));
      }
      estimated[vehicle] = Eigen::Vector2d(estimate.pose.x_m, estimate.pose.y_m);
      const Eigen::Vector2d truth(bundle.truth[vehicle].position.x(),
                                  bundle.truth[vehicle].position.y());
      const Eigen::Vector2d error = estimated[vehicle] - truth;
      const double norm = error.norm();
      error_all.push_back(norm);
      vehicles[vehicle].error.push_back(norm);
      ++vehicles[vehicle].count;
      ++samples;
      vehicles[vehicle].valid += estimate.xy_valid ? 1 : 0;
      valid_all += estimate.xy_valid ? 1 : 0;
      vehicles[vehicle].structural += estimate.xy_structural_valid ? 1 : 0;
      structural_all += estimate.xy_structural_valid ? 1 : 0;
      peer_valid_all += estimate.xy_peer_valid ? 1 : 0;
      if (estimate.xy_valid && norm > kFalseValidErrorM) ++false_valid;
      maximum_error = std::max(maximum_error, norm);
      if (estimate.xy_valid) {
        vehicles[vehicle].current_invalid = 0;
      } else {
        ++vehicles[vehicle].current_invalid;
        vehicles[vehicle].longest_invalid =
            std::max(vehicles[vehicle].longest_invalid, vehicles[vehicle].current_invalid);
      }
      mean_error += error / static_cast<double>(kVehicles);
      if (want_trace && index % kTraceStride == 0) {
        trace << "{\"w\":\"" << spec.window << "\",\"s\":\"" << spec.scenario
              << "\",\"k\":" << spec.seed << ",\"c\":\"" << spec.comm << "\",\"f\":\"" << spec.fusion
              << "\",\"t\":" << t_s << ",\"v\":" << vehicle << ",\"ex\":" << estimate.pose.x_m
              << ",\"ey\":" << estimate.pose.y_m << ",\"ez\":" << estimate.pose.z_m
              << ",\"tx\":" << truth.x() << ",\"ty\":" << truth.y()
              << ",\"e\":" << norm << ",\"xyv\":" << (estimate.xy_valid ? 1 : 0)
              << ",\"stv\":" << (estimate.xy_structural_valid ? 1 : 0)
              << ",\"pvv\":" << (estimate.xy_peer_valid ? 1 : 0) << ",\"m\":\""
              << estimate.localization_mode << "\"}\n";
      }
    }
    common_squared += mean_error.squaredNorm();
    ++common_count;
    for (int first = 0; first < kVehicles; ++first) {
      for (int second = first + 1; second < kVehicles; ++second) {
        const double estimate_distance = (estimated[first] - estimated[second]).norm();
        const Eigen::Vector2d truth_first(bundle.truth[first].position.x(),
                                          bundle.truth[first].position.y());
        const Eigen::Vector2d truth_second(bundle.truth[second].position.x(),
                                           bundle.truth[second].position.y());
        const double truth_distance = (truth_first - truth_second).norm();
        const double difference = estimate_distance - truth_distance;
        relative_squared += difference * difference;
        ++relative_count;
      }
    }
  }

  const auto metrics = [](std::vector<double>& values) {
    std::sort(values.begin(), values.end());
    const double p50 = percentile(values, 0.50);
    const double p95 = percentile(values, 0.95);
    return std::make_pair(p50, p95);
  };
  const auto [p50_all, p95_all] = metrics(error_all);

  const auto rmse = [](const std::vector<double>& values) {
    if (values.empty()) return 0.0;
    double sum = 0.0;
    for (double value : values) sum += value * value;
    return std::sqrt(sum / static_cast<double>(values.size()));
  };

  json vehicle_rows = json::array();
  for (int vehicle = 0; vehicle < kVehicles; ++vehicle) {
    std::vector<double> values = vehicles[vehicle].error;
    const auto [p50, p95] = metrics(values);
    vehicle_rows.push_back(
        {{"vehicle", vehicle},
         {"xy_rmse_m", rmse(vehicles[vehicle].error)},
         {"xy_p50_m", p50},
         {"xy_p95_m", p95},
         {"max_xy_error_m", vehicles[vehicle].error.empty()
                                ? 0.0
                                : *std::max_element(vehicles[vehicle].error.begin(),
                                                    vehicles[vehicle].error.end())},
         {"xy_valid_fraction", vehicles[vehicle].count
                                   ? static_cast<double>(vehicles[vehicle].valid) /
                                         static_cast<double>(vehicles[vehicle].count)
                                   : 0.0},
         {"xy_structural_fraction", vehicles[vehicle].count
                                        ? static_cast<double>(vehicles[vehicle].structural) /
                                              static_cast<double>(vehicles[vehicle].count)
                                        : 0.0},
         {"longest_invalid_s", vehicles[vehicle].longest_invalid * kLoopStepS},
         {"init_offset_m", (initial_position[vehicle] - initial_truth[vehicle]).norm()},
         {"init_yaw_offset_rad",
          wrapAngle(initial_yaw[vehicle] - initial_truth_yaw[vehicle])}});
  }

  json row;
  row["window"] = spec.window;
  row["scenario"] = spec.scenario;
  row["profile"] = spec.profile;
  row["seed"] = spec.seed;
  row["comm"] = spec.comm;
  row["fusion"] = spec.fusion;
  row["samples"] = samples;
  row["xy_rmse_m"] = rmse(error_all);
  row["xy_p50_m"] = p50_all;
  row["xy_p95_m"] = p95_all;
  row["max_xy_error_m"] = maximum_error;
  row["xy_valid_fraction"] =
      samples ? static_cast<double>(valid_all) / static_cast<double>(samples) : 0.0;
  row["xy_structural_fraction"] =
      samples ? static_cast<double>(structural_all) / static_cast<double>(samples) : 0.0;
  row["xy_peer_fraction"] =
      samples ? static_cast<double>(peer_valid_all) / static_cast<double>(samples) : 0.0;
  row["false_valid_fraction"] =
      samples ? static_cast<double>(false_valid) / static_cast<double>(samples) : 0.0;
  row["common_drift_rmse_m"] =
      common_count ? std::sqrt(common_squared / static_cast<double>(common_count)) : 0.0;
  row["pair_relative_rmse_m"] =
      relative_count ? std::sqrt(relative_squared / static_cast<double>(relative_count)) : 0.0;
  row["tof_calls"] = tof_calls;
  row["tof_accepted"] = tof_accepted;
  row["tof_accept_fraction"] = tof_calls ? static_cast<double>(tof_accepted) / tof_calls : 0.0;
  row["uwb_calls"] = uwb_calls;
  row["uwb_accepted"] = uwb_accepted;
  row["uwb_accept_fraction"] = uwb_calls ? static_cast<double>(uwb_accepted) / uwb_calls : 0.0;
  row["uwb_rejections"] = uwb_rejections;
  row["collaboration_evidence"] =
      (spec.comm != "off" && uwb_accepted > 0) ? "accepted_updates" : "no_accepted_updates";
  row["vehicles"] = vehicle_rows;

  RunResult result;
  result.row = std::move(row);
  result.trace = trace.str();
  return result;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    std::string terrain_path;
    std::string trajectory_path;
    std::string scenarios_path = "simulation/research/scenarios.json";
    std::string output_path;
    std::string trace_path;
    double start_s = 7.0;
    double seconds = 24.0;
    double evaluation_start_s = 3.0;
    double landing_start_s = 90.0;
    double landing_seconds = 10.0;
    int delay_ms = 0;
    int jobs = 1;
    bool allow_default_scenarios = false;
    bool run_landing = true;
    std::vector<int> seeds{2027, 4099, 6151};

    const auto argument = [&argc, &argv](int index) -> std::string {
      return index < argc ? std::string(argv[index]) : std::string();
    };
    for (int index = 1; index < argc; ++index) {
      const std::string key = argument(index);
      const auto value = [&]() -> std::string {
        if (index + 1 >= argc) throw std::runtime_error("missing value for " + key);
        return argument(++index);
      };
      if (key == "--terrain") {
        terrain_path = value();
      } else if (key == "--trajectory") {
        trajectory_path = value();
      } else if (key == "--scenarios") {
        scenarios_path = value();
      } else if (key == "--output") {
        output_path = value();
      } else if (key == "--trace") {
        trace_path = value();
      } else if (key == "--seconds") {
        seconds = std::stod(value());
      } else if (key == "--start") {
        start_s = std::stod(value());
      } else if (key == "--eval-start") {
        evaluation_start_s = std::stod(value());
      } else if (key == "--landing-start") {
        landing_start_s = std::stod(value());
      } else if (key == "--landing-seconds") {
        landing_seconds = std::stod(value());
      } else if (key == "--no-landing") {
        run_landing = false;
      } else if (key == "--comm-delay-ms") {
        delay_ms = std::stoi(value());
      } else if (key == "--jobs") {
        jobs = std::max(1, std::stoi(value()));
      } else if (key == "--allow-default-scenarios") {
        allow_default_scenarios = true;
      } else if (key == "--seeds") {
        seeds.clear();
        std::stringstream stream(value());
        std::string token;
        while (std::getline(stream, token, ',')) {
          if (!token.empty()) seeds.push_back(std::stoi(token));
        }
        if (seeds.empty()) throw std::runtime_error("--seeds needs at least one value");
      } else {
        throw std::runtime_error("unknown argument: " + key);
      }
    }
    if (terrain_path.empty() || trajectory_path.empty() || output_path.empty()) {
      throw std::runtime_error(
          "usage: localization_study --terrain PATH --trajectory PATH [--scenarios PATH] "
          "--output PATH [--trace PATH] [--seconds 24] [--start 7] [--seeds 2027,4099,6151] "
          "[--jobs N] [--allow-default-scenarios]");
    }
    if (trace_path.empty()) trace_path = output_path + ".trace.jsonl";

    const auto map = TerrainMap::load(terrain_path);
    const Trajectory trajectory = Trajectory::load(trajectory_path);
    const ScenarioLoad scenario_load = loadScenarios(scenarios_path, allow_default_scenarios);

    std::vector<Window> windows;
    const auto add_window = [&trajectory](std::vector<Window>& target, const std::string& name,
                                          double start, double length) {
      Window window;
      window.name = name;
      window.start_s = start;
      window.seconds = length;
      if (window.start_s + window.seconds > trajectory.tMax()) {
        window.seconds = std::max(0.0, trajectory.tMax() - window.start_s);
        window.clamped = true;
      }
      if (window.start_s < trajectory.tMin()) {
        window.start_s = trajectory.tMin();
        window.clamped = true;
      }
      if (window.seconds <= 0.0) {
        throw std::runtime_error("window " + name + " lies outside the trajectory record");
      }
      target.push_back(window);
    };
    add_window(windows, "transit", start_s, seconds);
    if (run_landing) add_window(windows, "landing", landing_start_s, landing_seconds);

    const std::array<std::string, 3> comm_modes{"on", "off", "intermittent"};
    const std::array<std::pair<std::string, bool>, 2> fusion_modes{
        std::make_pair(std::string("tof_imu_height"), false),
        std::make_pair(std::string("tof_flow_imu_height"), true)};

    json runs = json::array();
    json sensor_model = json::array();
    std::ofstream trace_file(trace_path);
    if (!trace_file) throw std::runtime_error("cannot open trace file: " + trace_path);

    std::mutex print_mutex;
    const int delay_ticks = static_cast<int>(std::lround(delay_ms / 1000.0 / kLoopStepS));

    for (const Window& window : windows) {
      const int steps = static_cast<int>(std::lround(window.seconds / kLoopStepS));
      // 初始位姿按窗口起点加固定偏移，窗口之间不共享初值。
      std::array<Eigen::Vector3d, kVehicles> initial_position;
      std::array<double, kVehicles> initial_yaw;
      std::array<Eigen::Vector3d, kVehicles> initial_truth;
      std::array<double, kVehicles> initial_truth_yaw;
      for (int vehicle = 0; vehicle < kVehicles; ++vehicle) {
        const Eigen::Vector3d truth = trajectory.position(vehicle, window.start_s);
        initial_truth[vehicle] = truth;
        initial_position[vehicle] = truth + Eigen::Vector3d(kInitOffsetX, kInitOffsetY, 0.0);
        initial_truth_yaw[vehicle] = trajectory.yaw(vehicle, window.start_s);
        initial_yaw[vehicle] = initial_truth_yaw[vehicle] + kInitYawOffset;
      }
      for (const Scenario& scenario : scenario_load.scenarios) {
        for (int seed : seeds) {
          InputSet input =
              generateInputs(trajectory, scenario, map, seed, window, steps, scenario_load.tof);

          std::vector<RunSpec> specs;
          for (const std::string& comm : comm_modes) {
            for (const auto& fusion : fusion_modes) {
              RunSpec spec;
              spec.window = window.name;
              spec.scenario = scenario.name;
              spec.profile = scenario.profile_name;
              spec.seed = seed;
              spec.comm = comm;
              spec.fusion = fusion.first;
              spec.use_flow = fusion.second;
              specs.push_back(std::move(spec));
            }
          }

          std::vector<RunResult> results(specs.size());
          std::vector<std::string> failures(specs.size());
          const auto work = [&](size_t begin, size_t end) {
            for (size_t index = begin; index < end; ++index) {
              try {
                results[index] =
                    runVariant(input, specs[index], map, scenario_load.tof, initial_position,
                               initial_yaw, initial_truth, initial_truth_yaw, evaluation_start_s,
                               delay_ticks);
              } catch (const std::exception& error) {
                failures[index] = error.what();
              }
            }
          };
          if (jobs <= 1) {
            work(0, specs.size());
          } else {
            std::vector<std::thread> pool;
            const size_t total = specs.size();
            const size_t chunk = (total + static_cast<size_t>(jobs) - 1) / static_cast<size_t>(jobs);
            for (size_t begin = 0; begin < total; begin += chunk) {
              pool.emplace_back(work, begin, std::min(total, begin + chunk));
            }
            for (auto& thread : pool) thread.join();
          }

          for (size_t index = 0; index < specs.size(); ++index) {
            if (!failures[index].empty()) {
              throw std::runtime_error("variant failed: " + failures[index]);
            }
            results[index].row["input_checksum"] = hex64(input.checksum);
            {
              std::lock_guard<std::mutex> guard(print_mutex);
              std::cout << window.name << " " << scenario.name << " " << seed << " "
                        << specs[index].comm << " " << specs[index].fusion
                        << " rmse=" << results[index].row["xy_rmse_m"].get<double>()
                        << " valid=" << results[index].row["xy_valid_fraction"].get<double>()
                        << " uwb_acc=" << results[index].row["uwb_accepted"].get<long long>()
                        << std::endl;
            }
            trace_file << results[index].trace;
            runs.push_back(results[index].row);
          }

          SensorStats& stats = input.stats;
          json entry;
          entry["window"] = window.name;
          entry["scenario"] = scenario.name;
          entry["profile"] = scenario.profile_name;
          entry["seed"] = seed;
          entry["rays"] = stats.rays;
          entry["robot_occluded_rays"] = stats.robot_occluded;
          entry["robot_occluded_fraction"] = stats.occludedFraction();
          entry["no_return"] = stats.no_return;
          entry["dropped"] = stats.dropped;
          entry["out_of_range"] = stats.out_of_range;
          entry["vehicle0_blackout"] = stats.blackout;
          entry["valid_returns"] = stats.valid_returns;
          entry["robot_count"] = static_cast<int>(scenario.robots.size());
          if (!stats.terrain_error.empty()) {
            std::vector<double> errors(stats.terrain_error.begin(), stats.terrain_error.end());
            double sum = 0.0;
            double sum_squared = 0.0;
            for (double value : errors) {
              sum += value;
              sum_squared += value * value;
            }
            const double mean = sum / static_cast<double>(errors.size());
            std::sort(errors.begin(), errors.end());
            entry["terrain_error_mean_m"] = mean;
            entry["terrain_error_std_m"] =
                std::sqrt(std::max(0.0, sum_squared / static_cast<double>(errors.size()) -
                                            mean * mean));
            entry["terrain_error_p50_m"] = percentile(errors, 0.50);
            entry["terrain_error_p95_m"] = percentile(errors, 0.95);
            entry["terrain_error_absmax_m"] =
                std::max(std::abs(errors.front()), std::abs(errors.back()));
          }
          entry["input_checksum"] = hex64(input.checksum);
          sensor_model.push_back(entry);
        }
      }
    }

    // 按 seed 配对的通信模式对比，不做把帧当独立样本的区间估计。
    json paired = json::array();
    for (const Window& window : windows) {
      for (const Scenario& scenario : scenario_load.scenarios) {
        for (const auto& fusion : fusion_modes) {
          for (int seed : seeds) {
            const auto find = [&](const std::string& comm) -> const json* {
              for (const auto& row : runs) {
                if (row["window"] == window.name && row["scenario"] == scenario.name &&
                    row["seed"] == seed && row["comm"] == comm && row["fusion"] == fusion.first) {
                  return &row;
                }
              }
              return nullptr;
            };
            const json* on = find("on");
            const json* off = find("off");
            const json* intermittent = find("intermittent");
            if (!on || !off || !intermittent) continue;
            json entry;
            entry["window"] = window.name;
            entry["scenario"] = scenario.name;
            entry["fusion"] = fusion.first;
            entry["seed"] = seed;
            entry["xy_rmse_on_m"] = (*on)["xy_rmse_m"];
            entry["xy_rmse_off_m"] = (*off)["xy_rmse_m"];
            entry["xy_rmse_intermittent_m"] = (*intermittent)["xy_rmse_m"];
            entry["delta_rmse_on_minus_off_m"] =
                (*on)["xy_rmse_m"].get<double>() - (*off)["xy_rmse_m"].get<double>();
            entry["delta_rmse_intermittent_minus_off_m"] =
                (*intermittent)["xy_rmse_m"].get<double>() - (*off)["xy_rmse_m"].get<double>();
            entry["delta_valid_on_minus_off"] =
                (*on)["xy_valid_fraction"].get<double>() -
                (*off)["xy_valid_fraction"].get<double>();
            entry["uwb_accepted_on"] = (*on)["uwb_accepted"];
            entry["uwb_accepted_intermittent"] = (*intermittent)["uwb_accepted"];
            entry["collaboration_evidence_on"] = (*on)["collaboration_evidence"];
            paired.push_back(entry);
          }
        }
      }
    }

    json output;
    output["schema"] = 1;
    output["generator"] = "simulation/research/localization_study.cpp";
    output["terrain"] = {{"path", terrain_path},
                         {"triangles", map->triangleCount()},
                         {"storage_bytes", map->storageBytes()}};
    output["trajectory"] = {
        {"path", trajectory_path},
        {"vehicles", kVehicles},
        {"span_s", trajectory.tMax() - trajectory.tMin()},
        {"t_min_s", trajectory.tMin()},
        {"t_max_s", trajectory.tMax()},
        {"interpolation_step_s", kLoopStepS},
        {"kinematics_smoothing_span_s", 2.0 * kKinematicsHalfS}};
    json window_rows = json::array();
    for (const Window& window : windows) {
      window_rows.push_back({{"name", window.name},
                             {"start_s", window.start_s},
                             {"seconds", window.seconds},
                             {"clamped_to_record", window.clamped},
                             {"evaluation_start_s", evaluation_start_s}});
    }
    output["windows"] = window_rows;
    output["rates_hz"] = {{"imu", 100.0},
                          {"height_assist", 50.0},
                          {"optical_flow", 50.0},
                          {"tof", kTofHz},
                          {"tof_sensors", 3},
                          {"uwb", 20.0}};
    json profile_rows = json::object();
    for (const auto& entry : scenario_load.profiles) {
      profile_rows[entry.first] = {{"noise_std_m", entry.second.noise_std_m},
                                   {"range_sigma_per_m", entry.second.range_sigma_per_m},
                                   {"bias_bound_m", entry.second.bias_bound_m},
                                   {"scale_bound", entry.second.scale_bound},
                                   {"frame_bias_std_m", entry.second.frame_bias_std_m},
                                   {"drop_probability", entry.second.drop_probability}};
    }
    output["profiles"] = profile_rows;
    json scenario_rows = json::array();
    for (const Scenario& scenario : scenario_load.scenarios) {
      json robots = json::array();
      for (const Robot& robot : scenario.robots) {
        robots.push_back({{"id", robot.id},
                          {"role", robot.role},
                          {"center_m", {robot.center.x(), robot.center.y(), robot.center.z()}},
                          {"size_m", {robot.size.x(), robot.size.y(), robot.size.z()}},
                          {"velocity_mps",
                           {robot.velocity.x(), robot.velocity.y(), robot.velocity.z()}},
                          {"moving", robot.moving}});
      }
      scenario_rows.push_back({{"name", scenario.name},
                               {"profile", scenario.profile_name},
                               {"robots", robots},
                               {"vehicle0_tof_blackout", scenario.vehicle0_tof_blackout}});
    }
    output["scenarios"] = scenario_rows;
    output["scenarios_source"] = {{"path", scenarios_path},
                                  {"used_builtin_defaults", scenario_load.used_defaults}};
    output["tof_geometry"] = {
        {"yaw_mounts_rad",
         {scenario_load.tof.yaw_rad[0], scenario_load.tof.yaw_rad[1], scenario_load.tof.yaw_rad[2]}},
        {"radius_m", scenario_load.tof.radius_m},
        {"horizontal_fov_rad", scenario_load.tof.horizontal_fov_rad},
        {"vertical_fov_rad", scenario_load.tof.vertical_fov_rad},
        {"pitch_down_rad", scenario_load.tof.pitch_down_rad},
        {"min_range_m", scenario_load.tof.min_range_m},
        {"max_range_m", scenario_load.tof.max_range_m},
        {"cells", kTofCellCount}};
    output["scenario_groups_available"] = scenario_load.available_groups;
    output["scenario_groups_excluded"] = scenario_load.excluded_groups;
    output["communication_modes"] = {
        {"on", "本地传感器更新后发布估计器快照，再融合全部队友链路"},
        {"off", "不调用 fuseUwb"},
        {"intermittent",
         {{"packet_drop_probability", kIntermittentDropProbability},
          {"vehicle0_outage_s", {kIntermittentOutageStartS, kIntermittentOutageEndS}},
          {"outage_reference", "窗口内相对秒"}}}};
    output["vehicle0_tof_blackout_s"] = {{"start_s", kVehicle0TofBlackoutStartS},
                                          {"end_s", kVehicle0TofBlackoutEndS},
                                          {"reference", "窗口内相对秒"}};
    output["fusion_modes"] = {
        {"tof_imu_height", "ToF + IMU + 高度辅助，不启用光流"},
        {"tof_flow_imu_height", "ToF + IMU + 高度辅助 + 光流"}};
    output["sensor_noise"] = {{"imu_accel_bias_mps2", kImuAccelBiasMps2},
                              {"imu_accel_white_mps2", kImuAccelWhiteMps2},
                              {"imu_yaw_bias_rad", kImuYawBiasRad},
                              {"imu_yaw_white_rad", kImuYawWhiteRad},
                              {"height_white_m", kHeightWhiteM},
                              {"flow_scale", kFlowScale},
                              {"flow_white_mps", kFlowWhiteMps},
                              {"uwb_range_sigma_m", kUwbRangeSigmaM},
                              {"uwb_bearing_sigma_rad", kUwbBearingSigmaRad}};
    output["tof_error_model"] = {
        {"header", "simulation/research/tof_error_model.hpp"},
        {"shared_with_gateway", true},
        {"range_policy", "kMarkInvalid"},
        {"edge_model_enabled", false}};
    output["initialization"] = {{"offset_m", {kInitOffsetX, kInitOffsetY}},
                                {"yaw_offset_rad", kInitYawOffset},
                                {"covariance_diagonal", kInitCovariance},
                                {"per_frame_truth_reset", false},
                                {"communication_delay_ms", delay_ms}};
    output["limitations"] = json::array(
        {"真值日志只提供位置与 yaw，roll/pitch 固定为 0，属于仅 yaw 姿态重建",
         "传感器与估计器先验共用同一静态地形网格，结构定位精度不代表真实建图误差",
         "高度辅助使用带噪真值高度，不是真实高度传感器；两种融合模式均为 ToF+IMU+高度辅助",
         "机器人以 AABB 射线交点只进入传感器模型，不进入估计器先验；高空飞行时地面机器人遮挡比例可能为 0",
         "共享 schema 中的 tray_pressure 组是托盘平台上的非比赛站位，未纳入六个场景；其遮挡压力不代表正式比赛",
         "UWB 距离与方位由真值相对几何加噪生成，仅用于合成量测；队友位姿来自各估计器快照",
         "初始位置与航向偏移固定，若门控不收敛即为该配置下的实际结果"});
    output["sensor_model"] = sensor_model;
    output["runs"] = runs;
    output["paired_by_seed"] = paired;

    std::ofstream output_file(output_path);
    if (!output_file) throw std::runtime_error("cannot open output: " + output_path);
    output_file << output.dump(2) << std::endl;
    if (!output_file.good()) throw std::runtime_error("failed to write output");
    trace_file.flush();
    std::cout << "runs=" << runs.size() << " output=" << output_path << " trace=" << trace_path
              << std::endl;
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << std::endl;
    return 2;
  }
}
