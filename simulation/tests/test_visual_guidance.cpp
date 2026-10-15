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

void sharedColorMasks(){
  PpmImage frame;frame.width=frame.height=256;frame.rgb.resize(256*256*3);
  for(int mode=0;mode<5;++mode){
    ArmorDetectionConfig detection;
    if(mode){detection.enemy_color=ArmorColor::Blue;}
    if(mode==2){detection.min_blue_channel=0;detection.blue_over_red_ratio=1.8;detection.blue_over_green_ratio=2.3;}
    if(mode==3){detection.min_blue_channel=120;detection.blue_over_red_ratio=1.4;detection.blue_over_green_ratio=1.2;}
    if(mode==4){detection.min_blue_channel=0;detection.blue_over_red_ratio=1.1;detection.blue_over_green_ratio=1.05;}
    for(int b=0;b<256;++b){
      if(mode>=2&&b!=0&&b!=1&&b!=119&&b!=120&&b!=159&&b!=160&&b!=200&&b!=254&&b!=255)continue;
      for(int g=0;g<256;++g)for(int r=0;r<256;++r){
        const size_t i=g*256+r;frame.rgb[i*3]=r;frame.rgb[i*3+1]=g;frame.rgb[i*3+2]=b;
      }
      const auto masks=classifyArmorPixels(frame,detection);
      require(masks.strips.size()==65536&&masks.pnp.size()==65536,"shared mask dimensions");
      for(int g=0;g<256;++g)for(int r=0;r<256;++r){
        const size_t i=g*256+r;
        const bool blue=b>=detection.min_blue_channel&&b>detection.blue_over_red_ratio*r&&b>detection.blue_over_green_ratio*g;
        const int maximum=std::max({r,g,b}),minimum=std::min({r,g,b});
        const bool bright=(r>=160&&r>1.6*g&&r>1.4*b)||(b>=160&&b>1.4*g&&b>1.4*r)||(maximum>=220&&maximum-minimum<=70);
        const bool pnp=b>=120&&b>1.4*r&&b>1.2*g;
        require(masks.strips[i]==(mode?blue:bright),"shared pixel mask matches scalar RGB criterion");
        require(masks.pnp[i]==(pnp?255:0),"shared PnP mask preserves independent RGB criterion");
      }
    }
  }
  ArmorDetectionConfig detection;detection.enemy_color=ArmorColor::Blue;
  require(classifyArmorPixels({},detection).strips.empty(),"invalid image has no mask");
  auto target=image();strip(target,40,40,59,10,20,230);strip(target,80,40,59,10,20,230);
  const auto original=target.rgb;
  auto masks=classifyArmorPixels(target,detection);
  const auto reference=detectArmorStrips(target,camera(),detection);
  const auto combined=detectArmorStrips(target,camera(),detection,std::move(masks.strips));
  require(reference.valid&&combined.valid&&reference.range_m==combined.range_m&&
      reference.centroid_x_px==combined.centroid_x_px&&reference.bright_pixels==combined.bright_pixels,
      "preclassified mask preserves strip geometry");
  require(target.rgb==original,"mask consumers preserve RGB image");
  require(!detectArmorStrips(target,camera(),detection,std::vector<uint8_t>(1)).valid,"wrong mask size rejected");
  for(int invalid=0;invalid<3;++invalid){
    auto bad=detection;
    if(invalid==0)bad.blue_over_red_ratio=1;
    if(invalid==1)bad.min_blue_channel=-1;
    if(invalid==2)bad.max_pair_aspect=std::numeric_limits<double>::quiet_NaN();
    bool rejected=false;try{classifyArmorPixels(target,bad);}catch(const std::invalid_argument&){rejected=true;}
    require(rejected,"shared classification validates configuration");
  }
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

template<class Projection>
void checkProjectionUncertainty(const Pose3& pose,Projection project){
  const auto reference=project(pose);
  require(bool(reference),"projection fixture valid");
  const double step=1e-6;
  for(int axis=0;axis<4;++axis){
    auto high=pose,low=pose;
    double* high_values[]={&high.x_m,&high.y_m,&high.z_m,&high.yaw_rad};
    double* low_values[]={&low.x_m,&low.y_m,&low.z_m,&low.yaw_rad};
    *high_values[axis]+=step;*low_values[axis]-=step;
    const auto a=project(high),b=project(low);
    require(a&&b,"finite difference projections valid");
    require(((a->position-b->position)/(2*step)-reference->pose_jacobian.col(axis)).norm()<5e-8,
        "exposure pose Jacobian matches finite differences");
  }
  Eigen::Matrix4d basis;
  basis<<.2,0,0,0, .04,.15,0,0, .02,-.03,.08,0, -.02,.015,.01,.06;
  const auto propagated=worldObservationCovariance(reference->pose_jacobian,basis*basis.transpose(),.0009);
  require(bool(propagated),"correlated exposure covariance accepted");
  Eigen::Matrix3d numerical=Eigen::Matrix3d::Identity()*.0009;
  for(int axis=0;axis<4;++axis){
    auto high=pose,low=pose;
    double* high_values[]={&high.x_m,&high.y_m,&high.z_m,&high.yaw_rad};
    double* low_values[]={&low.x_m,&low.y_m,&low.z_m,&low.yaw_rad};
    for(int row=0;row<4;++row){*high_values[row]+=step*basis(row,axis);*low_values[row]-=step*basis(row,axis);}
    const auto a=project(high),b=project(low);
    require(a&&b,"correlated perturbations valid");
    const Eigen::Vector3d delta=(a->position-b->position)/(2*step);
    numerical+=delta*delta.transpose();
  }
  require((numerical-*propagated).norm()<1e-8,"full covariance matches independent geometric perturbations");
}

void projectionCovariance(){
  GuideObservation guide;guide.valid=true;guide.x_px=90;guide.y_px=45;
  const Eigen::Vector3d offset(.08,.02,.08);
  for(double yaw:{-.8,.35,1.2}){
    Pose3 pose{1,2,3,.1,-.2,yaw};
    const auto pnp=[&](const Pose3& p){return cameraPointProjection({.2,-.1,2.3},p,.052);};
    const auto green=[&](const Pose3& p){return guideArmorProjection(guide,camera(),p,.052,3,offset);};
    checkProjectionUncertainty(pose,pnp);checkProjectionUncertainty(pose,green);
    const auto projection=green(pose);const auto legacy=guideArmorPosition(guide,camera(),pose,.052,3,offset);
    require(projection&&legacy&&(projection->position-*legacy).norm()<1e-12,"green point compatibility");
    require(projection->pose_jacobian.row(0).norm()<1e-12,"fixed plane removes own-position X sensitivity");
    const auto noise=worldObservationCovariance(projection->pose_jacobian,Eigen::Matrix4d::Identity()*.04,.0064);
    require(noise&&std::abs((*noise)(0,0)-.0064)<1e-12,"plane X preserves only base visual noise");
  }
  const auto projection=cameraPointProjection({0,0,2},Pose3{},.052);
  require(bool(projection),"camera projection valid");
  const auto exact=worldObservationCovariance(projection->pose_jacobian,Eigen::Matrix4d::Zero(),.0009);
  require(exact&&(*exact-Eigen::Matrix3d::Identity()*.0009).norm()<1e-12,"exact fixture pose preserves visual variance");
  Eigen::Matrix4d covariance=Eigen::Matrix4d::Zero();covariance(1,1)=.04;
  require(bool(worldObservationCovariance(projection->pose_jacobian,covariance,.0009)),"semidefinite pose covariance accepted");
  covariance(0,0)=-.01;
  require(!worldObservationCovariance(projection->pose_jacobian,covariance,.0009),"negative covariance rejected");
  covariance=Eigen::Matrix4d::Identity();covariance(0,1)=.01;
  require(!worldObservationCovariance(projection->pose_jacobian,covariance,.0009),"asymmetric covariance rejected");
  covariance=Eigen::Matrix4d::Identity();covariance(0,0)=std::numeric_limits<double>::quiet_NaN();
  require(!worldObservationCovariance(projection->pose_jacobian,covariance,.0009),"nonfinite covariance rejected");
  require(!worldObservationCovariance(projection->pose_jacobian,Eigen::Matrix4d::Identity(),0),"nonpositive visual noise rejected");
  const Eigen::Matrix<double,3,4> overflowing=Eigen::Matrix<double,3,4>::Constant(std::numeric_limits<double>::max());
  require(!worldObservationCovariance(overflowing,Eigen::Matrix4d::Identity(),.0009),"overflowing propagation rejected");
  auto pose=Pose3{};pose.yaw_rad=std::numeric_limits<double>::quiet_NaN();
  require(!cameraPointProjection({0,0,2},pose,.052),"nonfinite exposure pose rejected");
  pose.yaw_rad=kPi/2;guide.x_px=camera().cx_px;
  require(!guideArmorProjection(guide,camera(),pose,.052,3,offset),"parallel guide ray rejected");
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
    sharedColorMasks();
    projectionCovariance();
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
