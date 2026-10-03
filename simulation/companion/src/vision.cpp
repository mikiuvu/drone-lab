/**
 * @file vision.cpp
 * @brief PPM 帧读取、灯条连通域配对和视觉/航点控制意图生成。
 */
#include "companion/vision.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <queue>
#include <sstream>
#include <stdexcept>

namespace companion {
namespace {

/**
 * @brief 读取 PPM 头部的一个非空白字段，跳过字段前的 # 注释行。
 * @param stream 当前输入流，读取后位置推进。
 * @param token 非空输出指针，原内容被覆盖。
 * @return 是否读取到非空字段。
 */
bool readToken(std::istream& stream, std::string* token) {
  *token = {};
  char ch = 0;
  while (stream.get(ch)) {
    if (ch == '#') {
      std::string ignored;
      std::getline(stream, ignored);
      continue;
    }
    if (!std::isspace(static_cast<unsigned char>(ch))) {
      token->push_back(ch);
      break;
    }
  }
  while (stream.get(ch) && !std::isspace(static_cast<unsigned char>(ch))) {
    token->push_back(ch);
  }
  return !token->empty();
}

// 红/蓝通道突出或近白高亮像素可作为灯条候选；这里只做颜色筛选，形状和配对在后续判断。
bool brightPixel(const PpmImage& image, int x, int y, const ArmorDetectionConfig& config) {
  const size_t index = static_cast<size_t>((y * image.width + x) * 3);
  const int r = image.rgb[index];
  const int g = image.rgb[index + 1];
  const int b = image.rgb[index + 2];
  if (config.enemy_color == ArmorColor::Blue) {
    return b >= config.min_blue_channel && b > r * config.blue_over_red_ratio &&
           b > g * config.blue_over_green_ratio;
  }
  const int maximum = std::max({r, g, b});
  const int minimum = std::min({r, g, b});
  const bool red = r >= 160 && r > g * 1.6 && r > b * 1.4;
  const bool blue = b >= 160 && b > g * 1.4 && b > r * 1.4;
  return red || blue || (maximum >= 220 && maximum - minimum <= 70);
}

double clamp(double value, double lower, double upper) {
  return std::max(lower, std::min(upper, value));
}

Eigen::Matrix2d rotation2(double yaw) {
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  Eigen::Matrix2d result;
  result << c, -s, s, c;
  return result;
}

}  // namespace

std::optional<PpmImage> readPpm(const std::string& path) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) return std::nullopt;
  PpmImage image;
  // 网关先后替换图像和元数据；从 PPM 注释读取帧身份，避免相邻帧被静默错配。
  std::string first_line,comment;
  std::getline(stream,first_line);
  if(first_line!="P6")return std::nullopt;
  if(stream.peek()=='#') {
    std::getline(stream,comment);
    auto seq=comment.find("frame_seq="),time=comment.find("sim_time_us=");
    try {
      if(seq!=std::string::npos)image.frame_seq=std::stoull(comment.substr(seq+10));
      if(time!=std::string::npos)image.sim_time_us=std::stoull(comment.substr(time+12));
    } catch(...) {return std::nullopt;}
  }
  std::string magic;
  std::string width;
  std::string height;
  std::string max_value;
  if (!readToken(stream, &width) ||
      !readToken(stream, &height) || !readToken(stream, &max_value)) {
    return std::nullopt;
  }
  try {
    image.width = std::stoi(width);
    image.height = std::stoi(height);
    const int max_channel = std::stoi(max_value);
    if (image.width <= 0 || image.height <= 0 || image.width>4096 || image.height>4096 || max_channel != 255) return std::nullopt;
  } catch (...) {
    return std::nullopt;
  }
  image.rgb.resize(static_cast<size_t>(image.width) * image.height * 3);
  stream.read(reinterpret_cast<char*>(image.rgb.data()),
             static_cast<std::streamsize>(image.rgb.size()));
  if (stream.gcount() != static_cast<std::streamsize>(image.rgb.size())) return std::nullopt;
  return image;
}

ArmorObservation detectArmorStrips(const PpmImage& image, const CameraConfig& config) {
  return detectArmorStrips(image, config, ArmorDetectionConfig{});
}

ArmorObservation detectArmorStrips(const PpmImage& image, const CameraConfig& config,
                                   const ArmorDetectionConfig& detection) {
  ArmorObservation result;
  if (detection.enemy_color == ArmorColor::Blue &&
      (detection.min_blue_channel < 0 || detection.min_blue_channel > 255 ||
       !std::isfinite(detection.blue_over_red_ratio) || detection.blue_over_red_ratio <= 1.0 ||
       !std::isfinite(detection.blue_over_green_ratio) || detection.blue_over_green_ratio <= 1.0)) {
    throw std::invalid_argument("invalid blue armor color thresholds");
  }
  if (image.width <= 0 || image.height <= 0 ||
      image.rgb.size() != static_cast<size_t>(image.width) * image.height * 3) {
    return result;
  }
  // 1. 用四邻域广度搜索合并亮像素，得到面积、中心及包围盒；尺寸门限剔除散点和横条。
  std::vector<uint8_t> visited(static_cast<size_t>(image.width) * image.height, 0);
  std::vector<BrightStrip> components;
  const int min_area = std::max(4, image.width * image.height / 40000);
  for (int y = 0; y < image.height; ++y) {
    for (int x = 0; x < image.width; ++x) {
      const size_t seed = static_cast<size_t>(y * image.width + x);
      if (visited[seed] != 0 || !brightPixel(image, x, y, detection)) continue;
      std::queue<std::pair<int, int>> pending;
      pending.emplace(x, y);
      visited[seed] = 1;
      BrightStrip component;
      component.min_x = component.max_x = x;
      component.min_y = component.max_y = y;
      long long sum_x = 0;
      long long sum_y = 0;
      while (!pending.empty()) {
        const auto [cx, cy] = pending.front();
        pending.pop();
        ++component.area;
        sum_x += cx;
        sum_y += cy;
        component.min_x = std::min(component.min_x, cx);
        component.max_x = std::max(component.max_x, cx);
        component.min_y = std::min(component.min_y, cy);
        component.max_y = std::max(component.max_y, cy);
        for (const auto [nx, ny] : {std::pair<int, int>{cx - 1, cy},
                                    std::pair<int, int>{cx + 1, cy},
                                    std::pair<int, int>{cx, cy - 1},
                                    std::pair<int, int>{cx, cy + 1}}) {
          if (nx < 0 || nx >= image.width || ny < 0 || ny >= image.height) continue;
          const size_t index = static_cast<size_t>(ny * image.width + nx);
          if (visited[index] == 0 && brightPixel(image, nx, ny, detection)) {
            visited[index] = 1;
            pending.emplace(nx, ny);
          }
        }
      }
      const int component_width = component.max_x - component.min_x + 1;
      const int component_height = component.max_y - component.min_y + 1;
      if (component.area >= min_area && component_height >= component_width * 1.2 &&
          component_height >= 3) {
        component.center_x_px = static_cast<double>(sum_x) / component.area;
        component.center_y_px = static_cast<double>(sum_y) / component.area;
        // 触边灯条可能被裁切，蓝色模式不将它用于完整灯条间距测距。
        if (detection.enemy_color == ArmorColor::Blue &&
            (component.min_x == 0 || component.max_x == image.width - 1 ||
             component.min_y == 0 || component.max_y == image.height - 1)) {
          result.image_clipped = true;
          continue;
        }
        components.push_back(component);
      }
    }
  }
  if (components.size() < 2) return result;

  // 2. 比较灯条高度比例、纵向对齐及间距/高度比，选择面积较大且形状一致的配对。
  double best_score = -1.0;
  BrightStrip best_left;
  BrightStrip best_right;
  for (size_t i = 0; i < components.size(); ++i) {
    for (size_t j = i + 1; j < components.size(); ++j) {
      BrightStrip left = components[i];
      BrightStrip right = components[j];
      if (left.center_x_px > right.center_x_px) std::swap(left, right);
      const double separation = right.center_x_px - left.center_x_px;
      const double height_left = left.max_y - left.min_y + 1.0;
      const double height_right = right.max_y - right.min_y + 1.0;
      const double height_ratio = std::min(height_left, height_right) /
                                  std::max(height_left, height_right);
      const double y_alignment = std::abs(left.center_y_px - right.center_y_px) /
                                 std::max(height_left, height_right);
      const double width_limit = image.width * 0.75;
      const double pair_aspect=separation / (0.5*(height_left+height_right));
      if (separation < 3.0 || separation > width_limit || height_ratio < 0.65 ||
          y_alignment > 0.4 || pair_aspect<1.0 || pair_aspect>4.0) continue;
      const double score = height_ratio * (1.0 - clamp(y_alignment, 0.0, 1.0)) *
                           std::sqrt(static_cast<double>(left.area * right.area));
      if (score > best_score) {
        best_score = score;
        best_left = left;
        best_right = right;
      }
    }
  }
  if (best_score <= 0.0) return result;
  // 其他触边候选不影响当前完整配对；完整蓝色灯条对可重新建立观测。
  result.image_clipped = false;

  // 3. 针孔模型给出水平/竖直角误差与距离 Z≈fx*真实间距/像素间距。
  //    图像右/下为正，而机体左/上为正，角误差取负号；侧视目标会改变表观间距，影响测距。
  result.left = best_left;
  result.right = best_right;
  result.centroid_x_px = 0.5 * (best_left.center_x_px + best_right.center_x_px);
  result.centroid_y_px = 0.5 * (best_left.center_y_px + best_right.center_y_px);
  if (config.fx_px > 0.0) {
    result.angular_error_rad = -std::atan2(result.centroid_x_px - config.cx_px, config.fx_px);
  }
  if(config.fy_px>0.0)result.vertical_error_rad=-std::atan2(result.centroid_y_px-config.cy_px,config.fy_px);
  const double gap_px = best_right.center_x_px - best_left.center_x_px;
  if (config.strip_center_separation_m > 0.0 && config.fx_px > 0.0 && gap_px > 1.0) {
    result.range_m = config.strip_center_separation_m * config.fx_px / gap_px;
    result.range_valid = std::isfinite(result.range_m) && result.range_m > 0.05 &&
                         result.range_m < 100.0;
  }
  const double hleft=best_left.max_y-best_left.min_y+1.0;
  const double hright=best_right.max_y-best_right.min_y+1.0;
  result.confidence = std::min(hleft,hright)/std::max(hleft,hright) *
      (1.0-std::abs(best_left.center_y_px-best_right.center_y_px)/std::max(hleft,hright));
  result.valid = result.confidence >= 0.6;
  return result;
}

VisualTargetTracker::VisualTargetTracker(VisualGuidanceConfig config) : config_(config) {
  // 配置在实例构造时校验，避免运行过程中 NaN 或负限幅传播到执行器参考。
  for (double value : {config_.max_observation_age_s, config_.max_prediction_s,
                       config_.max_lateral_speed_mps, config_.max_forward_speed_mps,
                       config_.max_vertical_speed_mps, config_.max_yaw_rate_rad_s,
                       config_.yaw_gain, config_.lateral_gain, config_.forward_gain,
                       config_.vertical_gain, config_.stop_range_m}) {
    if (!std::isfinite(value) || value < 0.0) {
      throw std::invalid_argument("visual guidance constraints must be finite and nonnegative");
    }
  }
  if (!std::isfinite(config_.velocity_filter_alpha) || config_.velocity_filter_alpha < 0.0 ||
      config_.velocity_filter_alpha > 1.0 || !std::isfinite(config_.max_tracking_angle_rad) ||
      config_.max_tracking_angle_rad <= 0.0 || config_.max_tracking_angle_rad >= kPi / 2.0) {
    throw std::invalid_argument("invalid visual guidance filter or tracking angle");
  }
}

void VisualTargetTracker::reset() {
  last_observation_ = {};
  last_now_us_ = last_frame_us_ = last_valid_us_ = 0;
  lateral_velocity_mps_ = 0.0;
}

VisualGuidanceResult VisualTargetTracker::update(uint64_t now_us,
                                               const ArmorObservation& observation,
                                               uint64_t frame_time_us) {
  VisualGuidanceResult result;
  // 当前时钟回退时不修改缓存，也不沿旧时钟继续发出运动参考；显式 reset 可开始新序列。
  if (now_us == 0 || now_us < last_now_us_) return result;
  last_now_us_ = now_us;
  const bool timestamp_valid = frame_time_us != 0 && frame_time_us <= now_us;
  const double frame_age_s = timestamp_valid ? (now_us - frame_time_us) * 1.0e-6 : 0.0;
  const bool observation_usable = observation.valid && !observation.image_clipped &&
      std::isfinite(observation.angular_error_rad) &&
      std::isfinite(observation.vertical_error_rad) &&
      std::isfinite(observation.confidence) && observation.confidence >= 0.6 &&
      observation.confidence <= 1.0 &&
      std::abs(observation.angular_error_rad) <= config_.max_tracking_angle_rad &&
      std::abs(observation.vertical_error_rad) <= config_.max_tracking_angle_rad &&
      (!observation.range_valid ||
       (std::isfinite(observation.range_m) && observation.range_m > 0.0));
  bool fresh = false;
  if (timestamp_valid && frame_time_us > last_frame_us_) {
    const bool consecutive = last_frame_us_ == last_valid_us_ && last_valid_us_ != 0;
    // 无效或陈旧帧同样推进水位，禁止后来到达的旧帧恢复过期目标。
    last_frame_us_ = frame_time_us;
    if (observation_usable && frame_age_s <= config_.max_observation_age_s) {
      const double dt_s = last_valid_us_ != 0 ? (frame_time_us - last_valid_us_) * 1.0e-6 : 0.0;
      if (consecutive && dt_s > 1.0e-6 && dt_s <= config_.max_prediction_s &&
          observation.range_valid && last_observation_.range_valid) {
        // Z*tan(水平角) 是光轴坐标中的左向位移。它的时间差分是相对运动，
        // 含飞机自身平动/转动影响；只有相邻完整深度帧才形成前馈项。
        const double previous_left_m = last_observation_.range_m *
                                       std::tan(last_observation_.angular_error_rad);
        const double current_left_m = observation.range_m * std::tan(observation.angular_error_rad);
        const double measured_mps = clamp((current_left_m - previous_left_m) / dt_s,
            -config_.max_lateral_speed_mps, config_.max_lateral_speed_mps);
        lateral_velocity_mps_ = config_.velocity_filter_alpha * measured_mps +
            (1.0 - config_.velocity_filter_alpha) * lateral_velocity_mps_;
      } else {
        lateral_velocity_mps_ = 0.0;
      }
      last_observation_ = observation;
      last_valid_us_ = frame_time_us;
      fresh = true;
    }
  } else if (timestamp_valid && frame_time_us == last_frame_us_ &&
             frame_time_us == last_valid_us_ && observation_usable &&
             frame_age_s <= config_.max_observation_age_s) {
    // 控制循环可重复使用同一新鲜图像，速度估计和观测内容仍以首次接受的帧为准。
    fresh = true;
  }
  if (last_valid_us_ == 0) return result;
  result.observation_age_s = (now_us - last_valid_us_) * 1.0e-6;
  if (!fresh && result.observation_age_s > config_.max_prediction_s) return result;

  result.target_valid = fresh;
  result.predicting = !fresh;
  result.tracking_valid = true;
  result.range_valid = last_observation_.range_valid;
  result.range_m = result.range_valid ? last_observation_.range_m : 0.0;
  result.lateral_velocity_mps = lateral_velocity_mps_;
  result.yaw_error_rad = last_observation_.angular_error_rad;
  if (result.range_valid) {
    // 用有界横向相对速度补偿曝光到当前时刻的延迟；深度不外推，避免失锁时继续接近。
    const double left_m = result.range_m * std::tan(last_observation_.angular_error_rad) +
                          lateral_velocity_mps_ * result.observation_age_s;
    result.yaw_error_rad = clamp(std::atan2(left_m, result.range_m),
        -config_.max_tracking_angle_rad, config_.max_tracking_angle_rad);
    result.velocity_flu_mps.y() = clamp(config_.lateral_gain * left_m + lateral_velocity_mps_,
        -config_.max_lateral_speed_mps, config_.max_lateral_speed_mps);
    if (fresh) {
      result.velocity_flu_mps.x() = clamp(
          config_.forward_gain * (result.range_m - config_.stop_range_m),
          0.0, config_.max_forward_speed_mps) * std::cos(result.yaw_error_rad);
      result.velocity_flu_mps.z() = clamp(config_.vertical_gain * result.range_m *
          std::tan(last_observation_.vertical_error_rad),
          -config_.max_vertical_speed_mps, config_.max_vertical_speed_mps);
    }
  }
  result.yaw_rate_ref_rad_s = clamp(config_.yaw_gain * result.yaw_error_rad,
      -config_.max_yaw_rate_rad_s, config_.max_yaw_rate_rad_s);
  return result;
}

ControlIntention makeControlIntention(uint64_t now_us,
                                      const Estimate& estimate,
                                      const ArmorObservation& target,
                                      const CameraConfig& camera,
                                      const std::vector<PeerEstimate>& peers,
                                      const std::vector<Pose2>& safe_waypoints,
                                      int* waypoint_index,
                                      double waypoint_radius_m,
                                      bool armor_enabled) {
  ControlIntention result;
  result.sim_time_us = now_us;
  result.source = "ESTIMATE";
  result.safe_to_arm = estimate.initialized && estimate.xy_valid && estimate.z_valid &&
                       !estimate.stale_input;
  // 1. 目标引导优先：航向误差比例控制，距离约束前进速度；偏离视轴时减小接近速度。
  if (armor_enabled && target.valid && estimate.xy_valid) {
    result.mode = "GUIDANCE";
    result.target_valid = true;
    result.target_yaw_error_rad = target.angular_error_rad;
    result.target_range_m = target.range_m;
    result.yaw_rate_ref_rad_s = clamp(2.5 * target.angular_error_rad, -1.5, 1.5);
    if (target.range_valid) {
      result.velocity_flu_mps.x() = clamp(0.9 * (target.range_m - 1.2), -0.8, 0.8);
      result.velocity_flu_mps.x() = std::max(0.0, result.velocity_flu_mps.x());
      result.velocity_flu_mps.x() *= std::max(0.0,std::cos(target.angular_error_rad));
    }
  // 2. 无有效引导目标时沿航点移动，把 ENU 位置差旋转到机体水平面，到达后推进索引。
  } else if (estimate.xy_valid && !safe_waypoints.empty() && waypoint_index != nullptr) {
    int index = std::clamp(*waypoint_index, 0, static_cast<int>(safe_waypoints.size()) - 1);
    const Pose2& waypoint = safe_waypoints[index];
    const Eigen::Vector2d delta(waypoint.x_m - estimate.pose.x_m,
                                waypoint.y_m - estimate.pose.y_m);
    const double distance = delta.norm();
    if (distance <= std::max(0.05, waypoint_radius_m) &&
        index + 1 < static_cast<int>(safe_waypoints.size())) {
      ++index;
    }
    *waypoint_index = index;
    const Pose2& active = safe_waypoints[index];
    const Eigen::Vector2d active_delta(active.x_m - estimate.pose.x_m,
                                       active.y_m - estimate.pose.y_m);
    const double active_distance = active_delta.norm();
    if (active_distance > std::max(0.05, waypoint_radius_m)) {
      const Eigen::Vector2d body_delta = rotation2(-estimate.pose.yaw_rad) * active_delta;
      const double speed = clamp(0.8 * active_distance, 0.0, 0.65);
      result.mode = "NAVIGATE";
      result.waypoint_valid = true;
      result.waypoint_index = index;
      result.velocity_flu_mps.head<2>() = body_delta.normalized() * speed;
    } else {
      result.mode = "HOLD_WAYPOINT";
      result.waypoint_valid = true;
      result.waypoint_index = index;
    }
  }

  // 3. 对 1.25 m 内的新鲜队友叠加水平排斥，最后逐轴限幅；该规则未查询地形障碍。
  const Eigen::Matrix2d world_to_body =
      rotation2(-estimate.pose.yaw_rad);
  for (const PeerEstimate& peer : peers) {
    if (peer.source != "ESTIMATE" || peer.vehicle_id == estimate.vehicle_id ||
        peer.sim_time_us == 0 || now_us < peer.sim_time_us ||
        now_us - peer.sim_time_us > 400000) {
      continue;
    }
    const Eigen::Vector2d delta(peer.pose.x_m - estimate.pose.x_m,
                                peer.pose.y_m - estimate.pose.y_m);
    const double distance = delta.norm();
    constexpr double kSafetyRadius = 1.25;
    if (distance <= 1.0e-6 || distance >= kSafetyRadius) continue;
    const double factor = (kSafetyRadius - distance) / kSafetyRadius;
    const Eigen::Vector2d repulsion = world_to_body * (-delta.normalized() * factor * 0.8);
    result.velocity_flu_mps.x() += repulsion.x();
    result.velocity_flu_mps.y() += repulsion.y();
    result.avoidance_active = true;
  }
  result.velocity_flu_mps.x() = clamp(result.velocity_flu_mps.x(), -0.8, 0.8);
  result.velocity_flu_mps.y() = clamp(result.velocity_flu_mps.y(), -0.8, 0.8);
  if(target.valid && target.range_valid)result.velocity_flu_mps.z()=clamp(target.range_m*std::tan(target.vertical_error_rad)*.6,-.4,.4);
  if (camera.timeout_us == 0) result.mode = "NAVIGATE";
  return result;
}

}  // namespace companion
