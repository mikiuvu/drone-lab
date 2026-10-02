#include "companion/vision.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <queue>
#include <sstream>

namespace companion {
namespace {

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

bool brightPixel(const PpmImage& image, int x, int y) {
  const size_t index = static_cast<size_t>((y * image.width + x) * 3);
  const int r = image.rgb[index];
  const int g = image.rgb[index + 1];
  const int b = image.rgb[index + 2];
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
  // The gateway atomically replaces image then metadata. Bind their identity
  // using the PPM comment so an adjacent frame is never silently mislabeled.
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
  ArmorObservation result;
  if (image.width <= 0 || image.height <= 0 ||
      image.rgb.size() != static_cast<size_t>(image.width) * image.height * 3) {
    return result;
  }
  std::vector<uint8_t> visited(static_cast<size_t>(image.width) * image.height, 0);
  std::vector<BrightStrip> components;
  const int min_area = std::max(4, image.width * image.height / 40000);
  for (int y = 0; y < image.height; ++y) {
    for (int x = 0; x < image.width; ++x) {
      const size_t seed = static_cast<size_t>(y * image.width + x);
      if (visited[seed] != 0 || !brightPixel(image, x, y)) continue;
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
          if (visited[index] == 0 && brightPixel(image, nx, ny)) {
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
        components.push_back(component);
      }
    }
  }
  if (components.size() < 2) return result;

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
