/**
 * @file test_visual_guidance.cpp
 * @brief 合成 RGB 灯条和相机观测的颜色、运动、时效及失锁回归测试。
 */
#include "companion/vision.hpp"
#include "companion/color_threshold.hpp"

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

void colorThresholdEquivalence(){
  for(double ratio:{1.0000000000000002,1.2,1.4,1.5,1.6,2.0,2.75,256.0,1e300}){
    const auto thresholds=detail::colorThresholds(ratio);
    for(int other=0;other<256;++other)for(int channel=0;channel<256;++channel)
      require((channel>ratio*other)==(channel>=thresholds[other]),"byte ratio classification matches floating reference");
  }
  for(int other=0;other<256;++other)for(int channel=0;channel<256;++channel)
    require((channel>1.5*other)==(2*channel>3*other),"green integer ratio is exact");
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
    {
      auto frame = image();
      strip(frame, 40, 40, 49, 0, 8, 255);
      strip(frame, 86, 40, 49, 0, 8, 255);
      ArmorDetectionConfig blue;
      blue.enemy_color = ArmorColor::Blue;
      auto rejected = detectArmorStrips(frame, camera(), blue);
      require(!rejected.valid && rejected.rejection_reason == "pair_aspect",
              "inclined panel rejected by legacy aspect limit");
      require(rejected.accepted_strips == 2 && rejected.bright_pixels == 60,
              "color and component stages preserve both lights");
      blue.max_pair_aspect = 6.0;
      auto accepted = detectArmorStrips(frame, camera(), blue);
      require(accepted.valid && accepted.range_valid, "inclined panel pair accepted");
      require(std::abs(accepted.centroid_x_px - 64) < 1e-9, "inclined pair center");
      require(std::abs(accepted.range_m - .135 * 100 / 46) < 1e-9, "inclined pair range");
    }
    colorAndPixelGeometry();
    {
      auto frame=image();
      for(int y=47;y<=53;++y)for(int x=77;x<=83;++x)
        if((x-80)*(x-80)+(y-50)*(y-50)<=9)frame.rgb[(y*160+x)*3+1]=255;
      auto guide=detectGreenGuide(frame,80,50,20);
      require(guide.valid && guide.candidates==1,"green guide located");
      Pose3 pose;pose.x_m=1;pose.y_m=2;pose.z_m=3;
      const Eigen::Vector3d offset(.08,0,.08);
      auto point=guideArmorPosition(guide,camera(),pose,.052,4,offset);
      require(point && (*point-Eigen::Vector3d(4.08,2,3.08)).norm()<1e-9,
              "guide offset targets armor rather than green light");
      for(int y=47;y<=53;++y)for(int x=90;x<=96;++x)
        if((x-93)*(x-93)+(y-50)*(y-50)<=9)frame.rgb[(y*160+x)*3+1]=255;
      require(!detectGreenGuide(frame,80,50,20).valid,"ambiguous green guides rejected");
      require(!detectGreenGuide(frame,10,10,10).valid,"green outside expected region rejected");
    }
    {
      auto frame = image();
      for (int y=40; y<52; ++y) for (int base : {40, 65}) {
        const int x=base+(y-40)/2;
        frame.rgb[(y*frame.width+x)*3+2]=255;
      }
      ArmorDetectionConfig blue; blue.enemy_color=ArmorColor::Blue;
      const auto found=detectArmorStrips(frame,camera(),blue);
      require(found.valid && found.accepted_strips==2,
              "one-pixel diagonal lights stay connected");
    }
    colorThresholdEquivalence();
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
