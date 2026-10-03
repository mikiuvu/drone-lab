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
