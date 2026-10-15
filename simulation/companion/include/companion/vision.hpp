/**
 * @file vision.hpp
 * @brief PPM 读取、灯条像素检测及目标/航点控制意图接口。
 */
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "companion/types.hpp"

namespace companion {

struct GuideObservation {
  bool valid{false};
  double x_px{0}, y_px{0};
  int area{0}, candidates{0};
};
/** @brief 两条检测链的独立前景掩码，按行存储；strips 为 0/1，pnp 为 0/255。 */
struct ArmorColorMasks {
  std::vector<uint8_t> strips,pnp;
};
/** @brief 单次 RGB 扫描生成灯条颜色配置与固定 PnP 蓝色门限对应的掩码。
 * @note 非法颜色门限或配对门限抛出 invalid_argument；无效图像返回空掩码。
 */
ArmorColorMasks classifyArmorPixels(const PpmImage& image,const ArmorDetectionConfig& detection);
/** @brief 世界点 m 及其对曝光状态 [x,y,z,yaw] 的 3×4 雅可比。 */
struct WorldPointProjection {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Matrix<double,3,4> pose_jacobian{Eigen::Matrix<double,3,4>::Zero()};
};
/** @brief 相机右/下/前坐标旋转平移到 ENU，包含前向安装偏移，长度单位 m。 */
std::optional<WorldPointProjection> cameraPointProjection(
    const Eigen::Vector3d& point_camera_m,const Pose3& pose,double camera_forward_m);
/** @brief 绿色引导的平面求交及曝光状态雅可比；安装偏移视为固定标定量。 */
std::optional<WorldPointProjection> guideArmorProjection(
    const GuideObservation& guide,const CameraConfig& camera,const Pose3& pose,
    double camera_forward_m,double guide_plane_x_m,const Eigen::Vector3d& guide_to_armor_world_m);
/** @brief 传播曝光状态协方差，叠加正的各向同性视觉基准方差 m²；非法矩阵返回无效。 */
std::optional<Eigen::Matrix3d> worldObservationCovariance(
    const Eigen::Matrix<double,3,4>& pose_jacobian,const Eigen::Matrix4d& pose_covariance,
    double visual_variance_m2);
/** @brief 在预测图像位置附近寻找紧凑绿色亮斑；多候选时不擅自选择最大亮斑。 */
GuideObservation detectGreenGuide(const PpmImage& image, double expected_x,
                                  double expected_y, double radius_px);
/** @brief 视线与已知固定滑轨 X 平面相交，再加绿灯到装甲中心的安装偏移。 */
std::optional<Eigen::Vector3d> guideArmorPosition(
    const GuideObservation& guide, const CameraConfig& camera,
    const Pose3& pose, double camera_forward_m, double guide_plane_x_m,
    const Eigen::Vector3d& guide_to_armor_world_m);

/**
 * @brief 读取二进制 P6 PPM，并解析首个注释行中的帧序号和时间。
 * @param path 图像路径；仅接受 8 位通道、宽高各不超过 4096 的图像。
 * @return 完整 RGB 图像；格式错误、读失败或数据不完整时返回 nullopt。
 */
std::optional<PpmImage> readPpm(const std::string& path);
/**
 * @brief 从红、蓝及近白亮像素中寻找竖直灯条对。
 * @param image 按行存储的 RGB 图像。
 * @param config 像素内参及两灯条实际中心间距。
 * @return 最佳配对的中心、角误差与测距；无配对时 valid=false。
 * @note 距离由已知灯条间距的针孔投影估算；角误差左、上为正。
 */
ArmorObservation detectArmorStrips(const PpmImage& image, const CameraConfig& config);
/**
 * @brief 用指定敌方颜色筛选像素后配对；Blue 拒绝红色和近白候选。
 * @param image 按行存储的 RGB 图像。
 * @param config 像素内参及灯条实际中心间距。
 * @param detection RGB 强度与蓝色通道比例门限，非法门限抛出 invalid_argument。
 * @return 最佳完整配对；蓝色模式无完整配对且有触边灯条时 image_clipped=true。
 * @note 触边灯条不参与蓝色配对；单条灯条不估算深度。
 */
ArmorObservation detectArmorStrips(const PpmImage& image, const CameraConfig& config,
                                   const ArmorDetectionConfig& detection);
/** @brief 使用预分类前景并接管掩码存储；非零表示前景，尺寸须等于图像像素数。
 * @note 掩码应对应 detection 的颜色门限；尺寸不符时返回无效观测。
 */
ArmorObservation detectArmorStrips(const PpmImage& image,const CameraConfig& config,
    const ArmorDetectionConfig& detection,std::vector<uint8_t> mask);

/**
 * @brief 从相机角误差、轴向深度与曝光时间估计相对横向运动，生成 FLU 速度参考。
 * @note 假设光轴沿机体前向、图像右/下对应机体右/下；相机自身运动会进入相对速度。
 *       连续帧需对应同一灯条目标；颜色与几何配对未提供多目标身份关联。
 */
class VisualTargetTracker {
 public:
  /** @brief 保存配置；非有限、负约束或非法滤波/角度参数抛出 invalid_argument。 */
  explicit VisualTargetTracker(VisualGuidanceConfig config = {});
  /**
   * @brief 消费观测并推进到当前时间；重复帧不重复估计速度。
   * @param now_us 当前单调时间 us，与帧时间使用同一时钟。
   * @param observation 灯条配对结果；无图像/失锁时传默认无效观测。
   * @param frame_time_us 曝光时间 us；0、未来、陈旧或乱序帧不能更新有效观测与速度缓存。
   * @return 新鲜帧可推进；失锁时短时预测横向/偏航并停止推进，超时所有参考归零。
   * @note 无深度时只保留偏航对准；灯条出视野按失锁处理，重新识别后恢复。
   */
  VisualGuidanceResult update(uint64_t now_us, const ArmorObservation& observation,
                             uint64_t frame_time_us);
  /** @brief 清空时钟、观测与速度缓存，允许新的时间序列。 */
  void reset();

 private:
  VisualGuidanceConfig config_;
  ArmorObservation last_observation_;
  uint64_t last_now_us_{0};
  uint64_t last_frame_us_{0}; // 包括无效帧，避免旧有效帧覆盖较新的失锁帧
  uint64_t last_valid_us_{0};
  double lateral_velocity_mps_{0.0};
};
/**
 * @brief 根据目标、航点和队友距离生成 FLU 速度与航向角速度意图。
 * @param now_us 当前仿真时间，单位 us，用于剔除过期队友。
 * @param estimate 自身定位及有效性。
 * @param target 灯条检测结果，调用方负责管理目标新鲜度。
 * @param camera 相机配置，timeout_us 为零时将输出模式置为 NAVIGATE。
 * @param peers 队友估计，400 ms 内的有效来源用于水平排斥。
 * @param safe_waypoints ENU 平面航点列表。
 * @param waypoint_index 可空的输入/输出索引，到达航点后向后推进。
 * @param waypoint_radius_m 到达半径，单位 m，内部下限为 0.05。
 * @param armor_enabled 是否启用目标引导分支。
 * @return 含限幅速度、避障标志和解锁条件的意图；此函数不直接驱动执行器。
 */
ControlIntention makeControlIntention(uint64_t now_us,
                                       const Estimate& estimate,
                                       const ArmorObservation& target,
                                       const CameraConfig& camera,
                                       const std::vector<PeerEstimate>& peers,
                                       const std::vector<Pose2>& safe_waypoints = {},
                                       int* waypoint_index = nullptr,
                                       double waypoint_radius_m = 0.45,
                                       bool armor_enabled = true);

}  // namespace companion
