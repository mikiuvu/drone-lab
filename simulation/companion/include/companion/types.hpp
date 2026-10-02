#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

namespace companion {
class TerrainMap;

constexpr int kTofCellCount = 64;
constexpr double kPi = 3.14159265358979323846;

inline double wrapAngle(double value) {
  while (value > kPi) value -= 2.0 * kPi;
  while (value < -kPi) value += 2.0 * kPi;
  return value;
}

struct Pose2 {
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};
};

struct Pose3 {
  double x_m{0.0};
  double y_m{0.0};
  double z_m{0.0};
  double roll_rad{0.0};
  double pitch_rad{0.0};
  double yaw_rad{0.0};
};

struct MapLineSegment {
  Eigen::Vector2d start_m{Eigen::Vector2d::Zero()};
  Eigen::Vector2d end_m{Eigen::Vector2d::Zero()};
  double z_min_m{-std::numeric_limits<double>::infinity()};
  double z_max_m{std::numeric_limits<double>::infinity()};
};

/** @brief ToF 8x8 mount in body FLU; positive pitch points downward. */
struct TofMount {
  Eigen::Vector3d translation_body_m{Eigen::Vector3d::Zero()};
  double yaw_rad{0.0};
  double roll_rad{0.0};
  double pitch_down_rad{0.0};
  double horizontal_fov_rad{1.22};
  double vertical_fov_rad{1.22};
  double min_range_m{0.05};
  double max_range_m{4.0};
};

struct TofFrame {
  uint64_t sim_time_us{0};
  int sensor_index{-1};
  std::array<double, kTofCellCount> distance_m{};
  std::array<uint8_t, kTofCellCount> valid{};
};

struct FlowSample {
  uint64_t sim_time_us{0};
  double vx_body_mps{0.0};
  double vy_body_mps{0.0};
  double quality{0.0};
  double variance_m2s2{0.0};
};

struct UwbPeerSample {
  uint64_t sim_time_us{0};
  int peer_id{-1};
  double range_m{0.0};
  double bearing_rad{0.0};
  bool valid{false};
  double quality{1.0};
  bool nlos{false};
};

struct AnchorRangeSample {
  uint64_t sim_time_us{0};
  int anchor_id{-1};
  double range_m{0.0};
  double variance_m2{0.0};
  double quality{1.0};
  bool valid{false};
};

struct HeightAssistSample {
  uint64_t sim_time_us{0};
  double z_world_m{0.0};
  std::string source;
};

struct ImuSample {
  uint64_t sim_time_us{0};
  bool attitude_valid{false};
  double roll_flu_rad{0.0};
  double pitch_flu_rad{0.0};
  double yaw_enu_rad{0.0};
  bool acceleration_valid{false};
  Eigen::Vector3d acceleration_enu_mps2{Eigen::Vector3d::Zero()};
  bool pressure_valid{false};
  double absolute_pressure_hpa{0.0};
};

struct PeerEstimate {
  uint64_t sim_time_us{0};
  int vehicle_id{-1};
  Pose3 pose;
  Eigen::Vector3d velocity_enu_mps{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d pose_covariance{Eigen::Matrix3d::Identity()};
  bool height_assisted{false};
  bool xy_structural_valid{false};
  bool xy_peer_valid{false};
  int reference_origin{-1};
  uint64_t reference_seq{0};
  uint64_t reference_time_us{0};
  int reference_hops{0};
  std::vector<int> reference_path;
  std::string source;
};

struct EstimatorConfig {
  int vehicle_id{0};
  Pose3 initial_pose;
  Eigen::Matrix<double, 7, 7> initial_covariance =
      Eigen::Matrix<double, 7, 7>::Identity();
  std::vector<MapLineSegment> map_lines;
  std::shared_ptr<const TerrainMap> terrain;
  double terrain_range_gate_m{0.35};
  double terrain_plane_gate_m{0.15};
  double terrain_max_translation_update_m{0.15};
  double terrain_max_yaw_update_rad{0.10};
  std::vector<Pose2> safe_waypoints;
  double waypoint_radius_m{0.45};
  std::array<TofMount, 3> tof_mounts{};
  uint64_t tof_timeout_us{250000};
  uint64_t peer_timeout_us{400000};
  uint64_t height_timeout_us{250000};
  double tof_sigma_m{0.035};
  double uwb_range_sigma_m{0.15};
  double uwb_bearing_sigma_rad{0.08};
  double uwb_xy_covariance_floor_m2{0.04};
  double height_sigma_m{0.025};
  double baro_sigma_m{0.30};
  double initial_z_sigma_m{0.50};
  double floor_z_m{0.0};
  uint64_t prediction_timeout_us{5000000};
  uint64_t flow_timeout_us{500000};
  double flow_xy_covariance_limit{4.0};
  uint64_t anchor_timeout_us{500000};
  uint64_t spawn_prior_timeout_us{4000000};
  double spawn_prior_xy_variance_limit{0.40};
  bool use_baro_fallback{true};
  std::unordered_map<int, Eigen::Vector3d> uwb_anchors;
};

struct Estimate {
  uint64_t sim_time_us{0};
  int vehicle_id{-1};
  Pose3 pose;
  Eigen::Vector3d velocity_enu_mps{Eigen::Vector3d::Zero()};
  Eigen::Matrix<double, 7, 7> covariance =
      Eigen::Matrix<double, 7, 7>::Identity();
  bool initialized{false};
  bool xy_valid{false};
  bool xy_structural_valid{false};
  bool xy_peer_valid{false};
  bool spawn_prior_valid{false};
  bool z_valid{false};
  bool height_assisted{false};
  bool vz_derived_from_height{false};
  bool degenerate_geometry{false};
  bool stale_input{false};
  bool flow_valid{false};
  double position_confidence{0.0};
  std::string localization_mode{"SPAWN_PRIOR"};
  int reference_origin{-1};
  uint64_t reference_seq{0};
  uint64_t reference_time_us{0};
  int reference_hops{0};
  std::vector<int> reference_path;
  std::string source{"ESTIMATE"};
};

struct FusionReport {
  bool accepted{false};
  bool stale{false};
  bool gated{false};
  bool degenerate{false};
  int inliers{0};
  double innovation_norm{0.0};
  std::string reason;
};

struct PpmImage {
  int width{0};
  int height{0};
  std::vector<uint8_t> rgb;
  uint64_t sim_time_us{0};
  uint64_t frame_seq{0};
};

struct BrightStrip {
  double center_x_px{0.0};
  double center_y_px{0.0};
  int min_x{0};
  int max_x{0};
  int min_y{0};
  int max_y{0};
  int area{0};
};

struct ArmorObservation {
  bool valid{false};
  double centroid_x_px{0.0};
  double centroid_y_px{0.0};
  double angular_error_rad{0.0};
  double vertical_error_rad{0.0};
  double range_m{0.0};
  bool range_valid{false};
  double confidence{0.0};
  BrightStrip left;
  BrightStrip right;
};

struct CameraConfig {
  int width{0};
  int height{0};
  double fx_px{0.0};
  double fy_px{0.0};
  double cx_px{0.0};
  double cy_px{0.0};
  double strip_center_separation_m{0.135};
  uint64_t timeout_us{250000};
};

struct ControlIntention {
  uint64_t sim_time_us{0};
  std::string mode{"NAVIGATE"};
  Eigen::Vector3d velocity_flu_mps{Eigen::Vector3d::Zero()};
  double yaw_rate_ref_rad_s{0.0};
  double target_yaw_error_rad{0.0};
  double target_range_m{0.0};
  bool target_valid{false};
  bool avoidance_active{false};
  bool safe_to_arm{false};
  bool waypoint_valid{false};
  int waypoint_index{-1};
  std::string source{"ESTIMATE"};
};

}  // namespace companion
