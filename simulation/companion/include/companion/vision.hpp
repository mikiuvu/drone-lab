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
