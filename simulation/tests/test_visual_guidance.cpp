/**
 * @file test_visual_guidance.cpp
 * @brief 合成 RGB 灯条和相机观测的颜色、运动、时效及失锁回归测试。
 */
#include "companion/vision.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace companion;

namespace {
// 使用运行时检查，使 Release 构建也执行全部测试条件。
void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

PpmImage image() {
  PpmImage result;
  result.width = 160;
  result.height = 100;
  result.rgb.assign(160 * 100 * 3, 0);
  return result;
}

void strip(PpmImage& frame, int x, int top, int bottom, uint8_t r, uint8_t g, uint8_t b) {
  for (int y = top; y <= bottom; ++y) {
    for (int dx = 0; dx < 3; ++dx) {
      const size_t index = static_cast<size_t>((y * frame.width + x + dx) * 3);
      frame.rgb[index] = r;
      frame.rgb[index + 1] = g;
      frame.rgb[index + 2] = b;
    }
  }
}

CameraConfig camera() {
  CameraConfig result;
  result.fx_px = result.fy_px = 100.0;
  result.cx_px = 80.0;
  result.cy_px = 50.0;
  return result;
}

ArmorObservation observation(double left_m, double depth_m = 2.0) {
  ArmorObservation result;
  result.valid = result.range_valid = true;
  result.confidence = 1.0;
  result.range_m = depth_m;
  result.angular_error_rad = std::atan2(left_m, depth_m);
  return result;
}

void colorAndPixelGeometry() {
  ArmorDetectionConfig blue;
  blue.enemy_color = ArmorColor::Blue;
  auto frame = image();
  strip(frame, 40, 40, 59, 10, 20, 230);
  strip(frame, 80, 40, 59, 10, 20, 230);
  // 较大的红色灯条对不能夺走蓝色目标的配对。
  strip(frame, 95, 20, 59, 240, 10, 10);
  strip(frame, 145, 20, 59, 240, 10, 10);
  const auto found = detectArmorStrips(frame, camera(), blue);
  require(found.valid && found.range_valid && !found.image_clipped, "blue pair detection");
  require(std::abs(found.centroid_x_px - 61.0) < 1e-12, "red interference rejected");
  require(found.angular_error_rad > 0.0 && found.vertical_error_rad > 0.0, "pixel FLU signs");
  require(std::abs(found.range_m - 0.3375) < 1e-12, "pinhole axial depth");
  require(detectArmorStrips(frame, camera()).centroid_x_px > 90, "legacy larger bright pair");
  auto red = image();
  strip(red, 40, 40, 59, 240, 10, 10);
  strip(red, 80, 40, 59, 240, 10, 10);
  require(!detectArmorStrips(red, camera(), blue).valid, "red-only rejection");
  require(detectArmorStrips(red, camera()).valid, "legacy red compatibility");
  auto white = image();
  strip(white, 40, 40, 59, 240, 240, 240);
  strip(white, 80, 40, 59, 240, 240, 240);
  require(!detectArmorStrips(white, camera(), blue).valid, "white rejection in blue mode");
  require(detectArmorStrips(white, camera()).valid, "legacy white compatibility");
  blue.min_blue_channel = 240;
  require(!detectArmorStrips(frame, camera(), blue).valid, "configurable blue threshold");
}

void lateralMotionAndLimits() {
  VisualGuidanceConfig config;
  config.velocity_filter_alpha = 1.0;
  config.lateral_gain = 0.0;
  for (double sign : {-1.0, 1.0}) {
    VisualTargetTracker tracker(config);
    require(tracker.update(1000000, observation(0.0), 1000000).tracking_valid, "initial lock");
    const auto moving = tracker.update(1100000, observation(sign * 0.1), 1100000);
    require(moving.target_valid && !moving.predicting, "fresh motion lock");
    require(std::abs(moving.lateral_velocity_mps - sign) < 1e-10, "relative lateral speed");
    require(std::abs(moving.velocity_flu_mps.y() - sign) < 1e-10, "FLU lateral feedforward");
    require(sign * moving.yaw_rate_ref_rad_s > 0.0, "yaw direction");
    const auto repeated = tracker.update(1110000, observation(sign * 0.1), 1100000);
    require(std::abs(repeated.lateral_velocity_mps - sign) < 1e-10, "duplicate frame no re-difference");
    require(sign * repeated.yaw_error_rad > sign * moving.yaw_error_rad, "exposure delay prediction");
    const auto limited = tracker.update(1120000, observation(sign * 3.0), 1120000);
    require(std::abs(limited.velocity_flu_mps.y()) <= config.max_lateral_speed_mps, "lateral output bound");
    require(std::abs(limited.yaw_rate_ref_rad_s) <= config.max_yaw_rate_rad_s, "yaw output bound");
    require(limited.velocity_flu_mps.x() <= config.max_forward_speed_mps, "forward output bound");
  }
  VisualTargetTracker depthless;
  auto angular_only = observation(0.2);
  angular_only.range_valid = false;
  auto result = depthless.update(1000000, angular_only, 1000000);
  require(result.target_valid && !result.range_valid, "angle-only lock");
  require(result.velocity_flu_mps.isZero() && result.yaw_rate_ref_rad_s > 0.0, "angle-only yaw");
  config.stop_range_m = 1.0;
  VisualTargetTracker stopping(config);
  result = stopping.update(1000000, observation(0.0, 0.5), 1000000);
  require(result.velocity_flu_mps.x() == 0.0, "configured stop depth");
}

void pixelMotion() {
  ArmorDetectionConfig blue;
  blue.enemy_color = ArmorColor::Blue;
  VisualGuidanceConfig config;
  config.velocity_filter_alpha = 1.0;
  config.lateral_gain = 0.0;
  // 两帧完整蓝色灯条的像素平移，经针孔角误差和深度直接驱动跟踪器。
  for (int shift : {-5, 5}) {
    auto first = image();
    strip(first, 59, 40, 59, 10, 20, 230);
    strip(first, 99, 40, 59, 10, 20, 230);
    auto second = image();
    strip(second, 59 + shift, 40, 59, 10, 20, 230);
    strip(second, 99 + shift, 40, 59, 10, 20, 230);
    VisualTargetTracker tracker(config);
    tracker.update(1000000, detectArmorStrips(first, camera(), blue), 1000000);
    const auto result = tracker.update(1100000, detectArmorStrips(second, camera(), blue), 1100000);
    const double expected_mps = -0.3375 * shift / 100.0 / 0.1;
    require(result.target_valid && std::abs(result.lateral_velocity_mps - expected_mps) < 1e-10,
            "pixel motion to FLU relative velocity");
  }
}

void freshnessAndRecovery() {
  VisualTargetTracker tracker;
  require(!tracker.update(1000000, observation(0.0), 800000).tracking_valid, "stale initial frame");
  require(!tracker.update(1000000, observation(0.0), 1000001).tracking_valid, "future initial frame");
  require(!tracker.update(1000000, observation(0.0), 0).tracking_valid, "zero frame timestamp");
  auto locked = tracker.update(1000000, observation(0.1), 1000000);
  require(locked.target_valid && locked.range_valid, "fresh acquisition");
  const auto old = tracker.update(1010000, observation(-0.5), 990000);
  require(old.predicting && !old.target_valid && old.yaw_error_rad > 0.0, "out-of-order frame ignored");
  require(old.velocity_flu_mps.x() == 0.0, "out-of-order stops advance");
  const auto future = tracker.update(1020000, observation(-0.5), 2000000);
  require(future.predicting && future.yaw_error_rad > 0.0, "future frame cannot alter cache");
  const auto lost = tracker.update(1100000, {}, 1100000);
  require(lost.tracking_valid && lost.predicting && lost.range_valid, "bounded loss prediction");
  require(lost.range_m == locked.range_m && std::abs(lost.observation_age_s - 0.1) < 1e-12,
          "prediction depth and exposure age");
  require(lost.velocity_flu_mps.x() == 0.0 && lost.velocity_flu_mps.z() == 0.0, "loss stops approach");
  require(!tracker.update(1090000, observation(-0.5), 1090000).tracking_valid, "clock rollback rejected");
  const auto expired = tracker.update(1200001, {}, 0);
  require(!expired.tracking_valid && !expired.range_valid && !expired.predicting, "prediction expires");
  require(expired.velocity_flu_mps.isZero() && expired.yaw_rate_ref_rad_s == 0.0, "expired zero outputs");
  auto recovered = tracker.update(1250000, observation(-0.1), 1250000);
  require(recovered.target_valid && recovered.lateral_velocity_mps == 0.0, "reacquisition resets velocity");
  auto nan = observation(0.0);
  nan.angular_error_rad = std::numeric_limits<double>::quiet_NaN();
  require(!tracker.update(1260000, nan, 1260000).target_valid, "NaN observation rejected");
  tracker.reset();
  require(tracker.update(1, observation(0.0), 1).target_valid, "reset allows new clock");
}

void nearFieldClipping() {
  ArmorDetectionConfig blue;
  blue.enemy_color = ArmorColor::Blue;
  auto clipped = image();
  strip(clipped, 0, 40, 59, 10, 20, 230);
  strip(clipped, 40, 40, 59, 10, 20, 230);
  const auto partial = detectArmorStrips(clipped, camera(), blue);
  require(!partial.valid && !partial.range_valid && partial.image_clipped, "edge clipped pair rejected");
  auto single = image();
  strip(single, 40, 40, 59, 10, 20, 230);
  require(!detectArmorStrips(single, camera(), blue).valid, "single visible strip no depth guess");
  VisualTargetTracker tracker;
  tracker.update(1000000, observation(0.02, 0.2), 1000000);
  const auto prediction = tracker.update(1050000, partial, 1050000);
  require(prediction.predicting && prediction.velocity_flu_mps.x() == 0.0, "near loss stops approach");
  require(!tracker.update(1250000, {}, 0).tracking_valid, "near loss waits for recognition");
  require(tracker.update(1300000, observation(0.0, 0.2), 1300000).target_valid, "near reacquisition");
}
}

int main() {
  try {
    colorAndPixelGeometry();
    lateralMotionAndLimits();
    pixelMotion();
    freshnessAndRecovery();
    nearFieldClipping();
    std::cout << "PASS visual guidance tests\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << "\n";
    return 1;
  }
}
