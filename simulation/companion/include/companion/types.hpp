/**
 * @file types.hpp
 * @brief 伴随计算机的坐标、传感器观测、七维估计及视觉控制数据契约。
 * @note 世界坐标 ENU 为东、北、上；机体坐标 FLU 为前、左、上。时间戳均为仿真时间 us。
 *       字段后缀 m/rad/mps 分别表示 m/rad/m/s，sigma 表示标准差，quality 为 0..1 的质量值。
 */
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

/**
 * @brief 将有限角度折回 [-π,π]，使跨越角度边界的差值保持连续。
 * @param value 有限角度，单位 rad；调用方应先检查有限性。
 * @return 等价角度，单位 rad。
 */
inline double wrapAngle(double value) {
  while (value > kPi) value -= 2.0 * kPi;
  while (value < -kPi) value += 2.0 * kPi;
  return value;
}

/** @brief ENU 平面位置 m 与绕世界 Z 轴的航向 rad。 */
struct Pose2 {
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};
};

/** @brief ENU 位置 m 与 FLU 到 ENU 的姿态 rad，旋转次序为 Rz(yaw)Ry(pitch)Rx(roll)。 */
struct Pose3 {
  double x_m{0.0};
  double y_m{0.0};
  double z_m{0.0};
  double roll_rad{0.0};
  double pitch_rad{0.0};
  double yaw_rad{0.0};
};

/** @brief 平面地图中的有限墙段，端点和有效高度区间均为 m。 */
struct MapLineSegment {
  Eigen::Vector2d start_m{Eigen::Vector2d::Zero()};
  Eigen::Vector2d end_m{Eigen::Vector2d::Zero()};
  double z_min_m{-std::numeric_limits<double>::infinity()};
  double z_max_m{std::numeric_limits<double>::infinity()};
};

/** @brief ToF 8×8 阵列的 FLU 安装外参；正 pitch_down 绕 Y 轴将前向光轴转向下方。 */
struct TofMount {
  Eigen::Vector3d translation_body_m{Eigen::Vector3d::Zero()}; // 机体原点到传感器原点，FLU，m
  double yaw_rad{0.0};
  double roll_rad{0.0};
  double pitch_down_rad{0.0};
  double horizontal_fov_rad{1.22}; // 水平全视场角，rad
  double vertical_fov_rad{1.22}; // 竖直全视场角，rad
  double min_range_m{0.05}; // 可用于地图匹配的最小轴向深度，m
  double max_range_m{4.0}; // 可用于地图匹配的最大轴向深度，m
};

/**
 * @brief 一个传感器的轴向深度帧。
 * @note 数组按行排列，索引为 row*8+col；深度乘未归一化光学射线得到三维位移，
 *       不是沿单位射线的欧氏距离。valid=0 的像素不参与地图匹配。
 */
struct TofFrame {
  uint64_t sim_time_us{0};
  int sensor_index{-1}; // 传感器索引 0..2，与 tof_mounts 对应，-1 表示尚无帧
  std::array<double, kTofCellCount> distance_m{};
  std::array<uint8_t, kTofCellCount> valid{};
};

/** @brief 光流模块输出的 FLU 前向/左向速度 m/s，而非原始像素位移。 */
struct FlowSample {
  uint64_t sim_time_us{0};
  double vx_body_mps{0.0};
  double vy_body_mps{0.0};
  double quality{0.0};
  double variance_m2s2{0.0}; // 两个速度分量共用的测量方差，m²/s²，必须为正
};

/** @brief 指向队友的三维距离 m 和 FLU 水平方位角 rad，atan2(左,前)，左为正。 */
struct UwbPeerSample {
  uint64_t sim_time_us{0};
  int peer_id{-1};
  double range_m{0.0};
  double bearing_rad{0.0};
  bool valid{false};
  double quality{1.0};
  bool nlos{false}; // 非视距标记；融合权重由 quality 与噪声配置决定
};

/** @brief 到已知 ENU 基站的三维距离 m；variance_m2 为正距离方差 m²。 */
struct AnchorRangeSample {
  uint64_t sim_time_us{0};
  int anchor_id{-1};
  double range_m{0.0};
  double variance_m2{0.0};
  double quality{1.0};
  bool valid{false};
};

/** @brief 世界 ENU 高度 m；融合器要求 source 为 SIM_HEIGHT_ASSIST。 */
struct HeightAssistSample {
  uint64_t sim_time_us{0};
  double z_world_m{0.0};
  std::string source;
};

/** @brief 姿态 rad、ENU 平动加速度 m/s² 与绝对气压 hPa，分别用有效标志控制使用。 */
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

/**
 * @brief 队友状态及其绝对定位信息的传播凭据。
 * @note pose_covariance 只包含 [x,y,z] 的位置协方差 m²，不包含姿态。
 *       origin 与 seq 标识一份源信息，time 保留源观测时间，hops/path 记录传播链；
 *       转发时不能把旧源观测时间改成当前报文时间，否则会错误延长参考有效期。
 */
struct PeerEstimate {
  uint64_t sim_time_us{0};
  int vehicle_id{-1};
  Pose3 pose;
  Eigen::Vector3d velocity_enu_mps{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d pose_covariance{Eigen::Matrix3d::Identity()};
  bool height_assisted{false};
  bool xy_structural_valid{false};
  bool xy_peer_valid{false};
  int reference_origin{-1}; // 首次获得绝对参考的车辆编号，-1 表示缺省
  uint64_t reference_seq{0}; // 源车辆递增参考序号，0 表示缺省
  uint64_t reference_time_us{0}; // 源观测时间 us，转发时保留
  int reference_hops{0}; // 从源到当前发送者的转发跳数
  std::vector<int> reference_path; // 从源开始的车辆编号链，接收者检查自身是否已在链中
  std::string source;
};

/**
 * @brief 估计器的初值、地图、噪声和时效配置。
 * @note initial_covariance 按 [x,y,z,yaw,vx,vy,vz] 排列；terrain 非空时优先使用三角网格，
 *       否则使用 map_lines。sigma 是标准差，进入滤波前平方为方差。
 */
struct EstimatorConfig {
  int vehicle_id{0};
  Pose3 initial_pose;
  Eigen::Matrix<double, 7, 7> initial_covariance =
      Eigen::Matrix<double, 7, 7>::Identity();
  std::vector<MapLineSegment> map_lines;
  std::shared_ptr<const TerrainMap> terrain;
  double terrain_range_gate_m{0.35}; // 实测与地图轴向深度差门限，m
  double terrain_plane_gate_m{0.15}; // 点到面残差及最终 RMS 门限，m
  double terrain_max_translation_update_m{0.15}; // 一次匹配的 XY 总修正量上限，m
  double terrain_max_yaw_update_rad{0.10}; // 一次匹配的航向总修正量上限，rad
  std::vector<Pose2> safe_waypoints;
  double waypoint_radius_m{0.45};
  std::array<TofMount, 3> tof_mounts{};
  uint64_t tof_timeout_us{250000};
  uint64_t peer_timeout_us{400000};
  uint64_t height_timeout_us{250000};
  double tof_sigma_m{0.035};
  double uwb_range_sigma_m{0.15};
  double uwb_bearing_sigma_rad{0.08};
  double uwb_xy_covariance_floor_m2{0.04}; // 队友 UWB 更新后 Pxx/Pyy 的下限，m²
  double height_sigma_m{0.025};
  double baro_sigma_m{0.30};
  double initial_z_sigma_m{0.50};
  double floor_z_m{0.0};
  uint64_t prediction_timeout_us{5000000};
  uint64_t flow_timeout_us{500000};
  double flow_xy_covariance_limit{4.0}; // 光流支撑 XY 有效性时的 Pxx+Pyy 上限，m²
  uint64_t anchor_timeout_us{500000};
  uint64_t spawn_prior_timeout_us{4000000};
  double spawn_prior_xy_variance_limit{0.40}; // 出生先验和队友参考的 Pxx+Pyy 上限，m²
  bool use_baro_fallback{true};
  std::unordered_map<int, Eigen::Vector3d> uwb_anchors; // 基站编号到固定 ENU 位置，m
};

/**
 * @brief 自身位置、姿态、速度及定位来源快照。
 * @note 七维状态顺序为 [x,y,z,yaw,vx,vy,vz]，单位为 [m,m,m,rad,m/s,m/s,m/s]。
 *       covariance(i,j) 是两维误差的相关程度，单位为对应状态单位的乘积；对角线为方差，
 *       例如位置 m²、航向 rad²、速度 m²/s²。非对角项使速度测量也能修正相关位置误差。
 *       roll/pitch 由 IMU 提供，不属于这七维；xy_valid 汇总来源，不能代替检查来源标志。
 *       初始位姿属于状态初值，不自动构成有效 XY 量测；没有有效来源时应显示 INITIALIZING。
 */
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
  bool spawn_prior_valid{false}; // 初始位置先验仍在有效时间窗内，且水平不确定度足够小
  bool z_valid{false};
  bool height_assisted{false};
  bool vz_derived_from_height{false}; // 垂直速度来自连续辅助高度的时间差分
  bool degenerate_geometry{false}; // ToF 的 [x,y,yaw] 信息秩不足 3，可与 accepted 同时为真
  bool stale_input{false};
  bool flow_valid{false};
  double position_confidence{0.0}; // exp(-0.5*(Pxx+Pyy)) 的协方差评分，不是统计校准后的概率
  double xy_uncertainty_m{0.0}; // sqrt(Pxx+Pyy)，XY 协方差迹的平方根，单位 m
  std::string localization_mode{"INITIALIZING"};
  int reference_origin{-1};
  uint64_t reference_seq{0};
  uint64_t reference_time_us{0};
  int reference_hops{0};
  std::vector<int> reference_path;
  std::string source{"ESTIMATE"};
};

/** @brief 融合接受、时效、门限和几何诊断；reason 给出具体原因。 */
struct FusionReport {
  bool accepted{false};
  bool stale{false};
  bool gated{false};
  bool degenerate{false};
  int inliers{0};
  double innovation_norm{0.0}; // ToF 为残差 RMS m；其他来源按实现报告，UWB 混合距离与角度
  int fit_rank{0}; // ToF 最后一次匹配的 [x,y,yaw] 信息秩
  double fit_translation_m{0.0}; // 截断前的 XY 匹配修正量
  double fit_yaw_rad{0.0}; // 截断前的有符号航向匹配修正量
  bool bounded_recovery_requested{false};
  std::string recovery_gate;
  std::string reason;
};

/** @brief P6 图像；rgb 为按行 RGB 三通道字节，帧身份来自图像内注释。 */
struct PpmImage {
  int width{0};
  int height{0};
  std::vector<uint8_t> rgb;
  uint64_t sim_time_us{0};
  uint64_t frame_seq{0};
};

/** @brief 八邻域亮像素连通区域；中心和包围盒单位 px，area 为亮像素数量。 */
struct BrightStrip {
  double center_x_px{0.0};
  double center_y_px{0.0};
  int min_x{0};
  int max_x{0};
  int min_y{0};
  int max_y{0};
  int area{0};
};

/** @brief 灯条配对结果；像素中心 px、左/上为正的角误差 rad、针孔模型距离 m。 */
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
  bool image_clipped{false}; // 灯条触及图像边界，完整配对/测距不可用；跟踪器按失锁处理
  int bright_pixels{0}, candidate_components{0}, accepted_strips{0}, tested_pairs{0};
  int rejected_pair_aspect{0};
  double max_observed_pair_aspect{0};
  std::string rejection_reason{"invalid_image"};
};

/** @brief 灯条候选颜色；AnyBright 保持红、蓝和近白亮像素的兼容检测。 */
enum class ArmorColor { AnyBright, Blue };

/** @brief 像素颜色门限；蓝色模式要求 B 同时超过 R/G 的指定比例。 */
struct ArmorDetectionConfig {
  ArmorColor enemy_color{ArmorColor::AnyBright};
  int min_blue_channel{160}; // B 通道下限，0..255
  double blue_over_red_ratio{1.4};
  double blue_over_green_ratio{1.4};
  double max_pair_aspect{4.0}; // 图像中灯条间距/高度上限；倾斜板面由任务显式配置
};

/**
 * @brief 相机视轴闭环的时效、增益和 FLU 输出约束。
 * @note range_m 为沿前向光轴的深度；stop_range_m 控制减速终点，撞击任务可设为 0。
 *       预测窗口从最后有效帧曝光时间计时；失锁预测期间始终停止前向推进。
 */
struct VisualGuidanceConfig {
  double max_observation_age_s{0.12};
  double max_prediction_s{0.20};
  double max_lateral_speed_mps{2.0};
  double max_forward_speed_mps{0.8};
  double max_vertical_speed_mps{0.4};
  double max_yaw_rate_rad_s{1.5};
  double yaw_gain{2.5}; // 水平角误差到偏航角速度，1/s
  double lateral_gain{1.0}; // 横向相对位移到速度，1/s
  double forward_gain{0.9}; // 深度到接近速度，1/s
  double vertical_gain{0.6}; // 竖向相对位移到速度，1/s
  double stop_range_m{0.0};
  double velocity_filter_alpha{0.5}; // 相邻帧横向差分低通权重，0..1
  double max_tracking_angle_rad{1.2}; // 角误差上限，必须小于 π/2
};

/** @brief 相机跟踪输出；target_valid 表示新鲜观测，predicting 表示短时失锁预测。 */
struct VisualGuidanceResult {
  bool target_valid{false};
  bool predicting{false};
  bool tracking_valid{false}; // 新鲜跟踪或窗口内预测，可用于视觉事件门控
  bool range_valid{false}; // 预测时深度保持最近有效帧的值，需结合观测年龄核验
  double observation_age_s{0.0}; // 最近有效观测的年龄 s；无历史时为 0
  double yaw_error_rad{0.0};
  double range_m{0.0};
  Eigen::Vector3d velocity_flu_mps{Eigen::Vector3d::Zero()};
  double yaw_rate_ref_rad_s{0.0};
  double lateral_velocity_mps{0.0}; // 相机系相对横向运动估计，左为正，限幅 m/s
};

/** @brief 图像尺寸 px、焦距与主点 px、灯条中心间距 m、图像时效 us。 */
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

/** @brief FLU 速度 m/s 与航向角速度 rad/s 的控制意图，供上层转换为执行器指令。 */
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
