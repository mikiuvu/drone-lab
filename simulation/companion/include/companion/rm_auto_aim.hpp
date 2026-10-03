#pragma once

#include <array>
#include <limits>

#include "companion/types.hpp"

namespace companion {

/** @brief 蓝色装甲板检测；位置为相机右、下、前坐标，单位 m。 */
struct RmArmorDetection {
  bool valid{false};
  Eigen::Vector3d position_camera_m{Eigen::Vector3d::Zero()};
  double reprojection_error_px{std::numeric_limits<double>::infinity()};
  // 灯条中心线端点，顺序为左下、左上、右上、右下，单位 px。
  std::array<Eigen::Vector2d, 4> corners{
      Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(),
      Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero()};
};

/**
 * @brief 从 RGB 蓝色灯条的四角求 IPPE PnP，相机内参使用 fx_px/fy_px/cx_px/cy_px。
 * @param camera strip_center_separation_m 为两条灯条的真实中心间距 m。
 * @param strip_height_m 灯条中心线的真实端点间距 m。
 * @return 装甲板中心相对相机的位置及四角重投影 RMS px；失败时 valid=false。
 * @note 输入应为无畸变针孔图像；畸变系数为零。头文件不依赖 OpenCV。
 */
RmArmorDetection detectRmArmor(const PpmImage& image, const CameraConfig& camera,
                               double strip_height_m = 0.055,
                               double bottom_separation_m = 0.0);

}  // namespace companion
