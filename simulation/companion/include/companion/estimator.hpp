/**
 * @file estimator.hpp
 * @brief 七维 EKF 及结构定位、光流、UWB、高度融合接口。
 */
#pragma once
#include "companion/types.hpp"
#include <Eigen/Geometry>
#include <optional>

namespace companion {

/**
 * @brief 维护单架飞机的 ENU 状态、协方差和定位参考来源。
 * @note 七维状态为 [x,y,z,yaw,vx,vy,vz]，单位依次为 m、m、m、rad、m/s、m/s、m/s。
 *       roll、pitch 来自 IMU，不占协方差维度；调用方应按仿真时间顺序交付样本。
 */
class Estimator {
 public:
/**
 * @brief 保存配置并复位；编号为负、ToF 标准差不大于 0 或协方差含非有限数时抛出 invalid_argument。
 * @param config 初始状态、地图、安装外参及测量噪声配置。
 */
  explicit Estimator(EstimatorConfig config);
/**
 * @brief 恢复初始状态，清空测量缓存、去重记录和参考历史。
 * @param now_us 仿真时间，单位 us；0 表示等待首个有效时间戳。
 */
  void reset(uint64_t now_us = 0);
/**
 * @brief 推进状态和协方差；过去或零时间戳不推进。
 * @param now_us 目标时间，单位 us；间隔超过 2 s 时只扩大协方差并清空加速度。
 */
  void advanceTo(uint64_t now_us);
/**
 * @brief 用上一份加速度推进，再接收 IMU 姿态增量和新加速度。
 * @param sample ENU 加速度与姿态样本，重复 IMU 时间戳被忽略。
 */
  void predict(const ImuSample& sample);
  /** @brief 返回接收本条有效 IMU 后的地图航向，保留已有地图修正；不改变状态。
   * @note 首个有效姿态建立增量参考。无效或重复样本沿用当前航向。
   */
  double yawAfterImu(const ImuSample& sample) const;
  /**
   * @brief 在确认着陆且已解除武装时清除惯性速度和保持加速度。
   * @param now_us 可选仿真时间 us；早于当前状态时忽略，0 表示不推进时间。
   * @note 调用方负责确认着陆状态；位置和地图参考保留。
   */
  void resetInertialVelocityWhileLanded(uint64_t now_us = 0);
/**
 * @brief 用已知接触面约束法向位置和法向速度。
 * @param now_us 接触时间，单位 us，不得早于状态时间。
 * @param normal ENU 单位法向；平面满足 normal.dot(position)=plane_offset_m。
 * @param plane_offset_m 平面有符号偏移，单位 m。
 * @return 接受或门限拒绝结果；接受后清空保持加速度。
 */
  FusionReport fuseMapContact(uint64_t now_us, const Eigen::Vector3d& normal, double plane_offset_m);
/**
 * @brief 将深度匹配到地图，仅沿可观测的 x/y/yaw 方向融合。
 * @param frame 三个传感器之一的 8×8 轴向深度帧，单位 m。
 * @return 内点数、残差均方根及退化/拒绝原因；退化和接受可以同时成立。
 * @note 历史曝光需被姿态缓冲覆盖，插值仅使用间隔不超过 100 ms 的相邻姿态。
 *       最新姿态之后采用零阶保持；未输入过 IMU 姿态时采用固定姿态模型。
 */
  FusionReport fuseTof(const TofFrame& frame);
/**
 * @brief 用机体 FLU 前向、左向速度约束世界系速度。
 * @param sample 带正方差的测量，质量至少 0.5 才融合。
 * @return 时效、质量检查及融合结果。
 * @note 机体投影使用采样时刻的历史姿态并保留当前地图航向修正。
 *       采样早于姿态历史或插值间隔超过 100 ms 时拒绝；最新姿态之后零阶保持。
 *       无 IMU 历史时使用状态姿态；速度更新当前状态，不回滚或重放过去的速度。
 */
  FusionReport fuseOpticalFlow(const FlowSample& sample);
/**
 * @brief 结合队友参考与相对距离/方位角更新自身状态。
 * @param sample 指向队友的三维距离和机体系水平角。
 * @param peer 队友估计、位置协方差与参考路径。
 * @return 时间、循环路径、重复参考、几何和创新门限检查结果。
 */
  FusionReport fuseUwb(const UwbPeerSample& sample, const PeerEstimate& peer);
/**
 * @brief 融合已配置基站的三维距离。
 * @param sample 基站编号、距离、方差与质量。
 * @return 更新结果；新鲜基站的水平几何充分时才建立绝对参考。
 */
  FusionReport fuseAnchorRange(const AnchorRangeSample& sample);
/**
 * @brief 直接替换辅助高度并重设 z 协方差，适当间隔内差分得到垂直速度。
 * @param sample 来源为 SIM_HEIGHT_ASSIST 的世界高度，单位 m。
 * @return 来源、时间检查及更新结果。
 */
  FusionReport fuseHeightAssist(const HeightAssistSample& sample);
/**
 * @brief 高度辅助过期时，用相对初始气压的高度作为后备观测。
 * @param sample 绝对气压，单位 hPa；首个有效值绑定当前估计高度。
 * @return 接受或跳过原因，是否启用由配置决定。
 */
  FusionReport fuseBaroPressure(const ImuSample& sample);
/**
 * @brief 返回状态副本，按当前时间和协方差重新判定定位来源有效性。
 * @return 位置、速度、不确定度及参考来源快照。
 */
  Estimate estimate() const;
/**
 * @brief 取得实例持有的配置只读引用，引用随实例销毁而失效。
 */
  const EstimatorConfig& config() const { return config_; }
/**
 * @brief 由像素中心和安装角生成 FLU 射线。
 * @param mount 视场和旋转外参，平移由观测函数另行处理。
 * @param row 行号 0..7，向下增加。
 * @param col 列号 0..7，向右增加。
 * @return 未归一化射线，乘轴向深度即为相对传感器的机体系位移。
 */
  static Eigen::Vector3d tofRayBody(const TofMount& mount, int row, int col);

 private:
  // 单点残差为 m，雅可比列对应 [x,y,yaw]；normal 在三角网格观测中保存 ENU 面法向。
  struct LineObservation { double residual{0}; Eigen::RowVector3d jacobian{Eigen::RowVector3d::Zero()}; Eigen::Vector3d normal{Eigen::Vector3d::Zero()}; };
  bool lineObservation(const TofFrame&, int, const Pose3&, const Eigen::Matrix3d&, LineObservation&) const;
  bool terrainObservation(const TofFrame&, int, const Pose3&, const Eigen::Matrix3d&, LineObservation&) const;
  // 将曝光时机体向量转到最新 IMU 机体系；无姿态输入时使用固定姿态模型。
  std::optional<Eigen::Matrix3d> attitudeDelta(uint64_t stamp) const;
  // 通用 EKF 更新：H 为 M×7，残差为 M×1，测量噪声协方差为 M×M。
  void update(const Eigen::MatrixXd&, const Eigen::VectorXd&, const Eigen::MatrixXd&);
  bool fresh(uint64_t stamp, uint64_t timeout) const;
  EstimatorConfig config_;
  // 安装外参在实例生命周期内固定；缓存三路 8×8 射线，匹配迭代中只更新机体姿态。
  std::array<std::array<Eigen::Vector3d,kTofCellCount>,3> tof_rays_;
  Estimate state_;
  // 每个方向保留最新帧；已接收和已融合时间分开记录，允许等待互补几何但禁止重复计权。
  std::array<TofFrame,3> tof_cache_{};
  std::array<uint64_t,3> last_tof_us_{};
  std::array<uint64_t,3> assimilated_tof_us_{};
  struct StampedAttitude { uint64_t stamp{0}; Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()}; };
  // 固定容量保存原始 IMU 姿态；地图匹配的航向修正不写入这段相对运动历史。
  std::array<StampedAttitude,128> attitude_history_;
  std::size_t attitude_begin_{0},attitude_count_{0};
  std::unordered_map<int, uint64_t> last_uwb_us_;
  std::unordered_map<int, uint64_t> last_anchor_us_;
  // 按参考源记录已使用的最大序号，避免同一绝对信息沿队友链路反复回流。
  std::unordered_map<int, uint64_t> seen_reference_seq_;
  struct AnchorMeasurement { Eigen::Vector3d position{Eigen::Vector3d::Zero()}; uint64_t sim_time_us{0}; double quality{0.0}; };
  std::unordered_map<int, AnchorMeasurement> accepted_anchors_;
  // [x,y,yaw] 的衰减累计信息，仅用于累计秩诊断；新绝对参考仍要求当前匹配的 XY 可观测。
  Eigen::Matrix3d structural_information_{Eigen::Matrix3d::Zero()};
  uint64_t structural_information_us_{0};
  // 各来源最后一次有效测量时间，单位 us；0 表示尚无样本。
  uint64_t last_imu_us_{0}, last_height_us_{0}, last_baro_us_{0}, last_absolute_us_{0};
  uint64_t startup_sim_us_{0};
  uint64_t last_local_tof_us_{0}, last_peer_xy_us_{0};
  uint64_t last_flow_us_{0}, next_reference_seq_{0};
  int local_tof_rank_{0};
  bool flow_valid_{false};
  // 依次为上一 IMU 航向 rad、基准气压 hPa、基准高度 m、上一辅助高度 m。
  double last_imu_yaw_{0}, baro_reference_hpa_{0}, baro_reference_z_{0}, previous_height_m_{0};
  uint64_t previous_height_us_{0};
  // ENU 加速度，单位 m/s²；预测期间零阶保持，IMU 超过 100 ms 未更新时不再使用。
  Eigen::Vector3d held_acceleration_{Eigen::Vector3d::Zero()};
};
}
