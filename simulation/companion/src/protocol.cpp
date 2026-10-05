/**
 * @file protocol.cpp
 * @brief schema=1 JSON 数据报的字段校验、解码与序列化。
 */
#include "companion/protocol.hpp"

#include <cmath>

#include <nlohmann/json.hpp>

namespace companion {
namespace {

using json = nlohmann::json;

bool schema1(const json& value) {
  return value.is_object() && value.value("schema", 0) == 1;
}

bool finiteNumber(const json& value, const char* key) {
  return value.contains(key) && value[key].is_number() &&
         std::isfinite(value[key].get<double>());
}

bool finiteArray64(const json& value, const char* key) {
  if (!value.contains(key) || !value[key].is_array() || value[key].size() != kTofCellCount) {
    return false;
  }
  for (const json& element : value[key]) {
    if (!element.is_number() || !std::isfinite(element.get<double>())) return false;
  }
  return true;
}

/**
 * @brief 读取可选整数路径；缺省时保留空路径，不在此处检查环路和重复节点。
 * @param value JSON 对象。
 * @param key 路径字段名。
 * @param output 追加解析出的车辆编号；失败时可能含部分内容，由调用方丢弃整份样本。
 * @return 字段缺省或全部元素为整数时为 true。
 */
bool finiteOptionalArray(const json& value, const char* key, std::vector<int>& output) {
  if (!value.contains(key)) return true;
  if (!value[key].is_array()) return false;
  for (const auto& item : value[key]) {
    if (!item.is_number_integer()) return false;
    output.push_back(item.get<int>());
  }
  return true;
}

}  // namespace

std::optional<TofFrame> parseTofDatagram(const std::string& payload, int expected_vehicle) {
  try {
    const json value = json::parse(payload);
    if (!schema1(value) || value.value("kind", "") != "tof" ||
        value.value("vehicle", -1) != expected_vehicle ||
        !finiteNumber(value, "sim_time_us") || value.value("sensor", -1) < 0 ||
        !finiteArray64(value, "distance_m")) {
      return std::nullopt;
    }
    // 数组必须为 64 个有限深度；未提供 valid 时默认全有效，量程和时效由融合器检查。
    TofFrame result;
    result.sim_time_us = value["sim_time_us"].get<uint64_t>();
    result.sensor_index = value["sensor"].get<int>();
    for (int i = 0; i < kTofCellCount; ++i) {
      result.distance_m[i] = value["distance_m"][i].get<double>();
      result.valid[i] = 1;
    }
    if (value.contains("valid")) {
      if (!value["valid"].is_array() || value["valid"].size() != kTofCellCount) {
        return std::nullopt;
      }
      for (int i = 0; i < kTofCellCount; ++i) {
        result.valid[i] = value["valid"][i].get<bool>() ? 1 : 0;
      }
    }
    return result;
  } catch (...) {
    return std::nullopt;
  }
}

std::optional<FlowSample> parseOpticalFlowDatagram(const std::string& payload,
                                                   int expected_vehicle) {
  try {
    const json value = json::parse(payload);
    if (!schema1(value) || value.value("kind", "") != "optical_flow" ||
        value.value("vehicle", -1) != expected_vehicle ||
        !finiteNumber(value, "sim_time_us") || !finiteNumber(value, "vx_body_mps") ||
        !finiteNumber(value, "vy_body_mps") || !finiteNumber(value, "quality") ||
        !finiteNumber(value, "variance_m2s2") ||
        !value["sim_time_us"].is_number_unsigned() || value["sim_time_us"]==0) return std::nullopt;
    FlowSample result{value["sim_time_us"].get<uint64_t>(), value["vx_body_mps"].get<double>(),
                      value["vy_body_mps"].get<double>(), value["quality"].get<double>(),
                      value["variance_m2s2"].get<double>()};
    if (result.quality < 0.0 || result.quality > 1.0 || result.variance_m2s2 <= 0.0) return std::nullopt;
    return result;
  } catch (...) { return std::nullopt; }
}

std::optional<UwbPeerSample> parseUwbDatagram(const std::string& payload,
                                              int expected_vehicle) {
  try {
    const json value = json::parse(payload);
    if (!schema1(value) || value.value("kind", "") != "uwb" ||
        value.value("vehicle", -1) != expected_vehicle ||
        !finiteNumber(value, "sim_time_us") || !finiteNumber(value, "range_m") ||
        !finiteNumber(value, "bearing_rad")) {
      return std::nullopt;
    }
    UwbPeerSample result;
    result.sim_time_us = value["sim_time_us"].get<uint64_t>();
    result.peer_id = value.value("peer", -1);
    result.range_m = value["range_m"].get<double>();
    result.bearing_rad = value["bearing_rad"].get<double>();
    result.valid = value.value("valid", true);
    result.quality = value.value("quality", 1.0);
    result.nlos = value.value("nlos", false);
    if (result.peer_id < 0 || result.range_m <= 0.0 || result.quality < 0.0 || result.quality > 1.0) return std::nullopt;
    return result;
  } catch (...) {
    return std::nullopt;
  }
}

std::optional<AnchorRangeSample> parseAnchorRangeDatagram(const std::string& payload,
                                                          int expected_vehicle) {
  try {
    const json value = json::parse(payload);
    if (!schema1(value) || value.value("kind", "") != "uwb_anchor" ||
        value.value("vehicle", -1) != expected_vehicle || !finiteNumber(value, "sim_time_us") ||
        !finiteNumber(value, "range_m") || !finiteNumber(value, "variance_m2") ||
        !finiteNumber(value, "quality") || !value["sim_time_us"].is_number_unsigned() ||
        value["sim_time_us"]==0) return std::nullopt;
    AnchorRangeSample result{value["sim_time_us"].get<uint64_t>(), value.value("anchor_id", -1),
                             value["range_m"].get<double>(), value["variance_m2"].get<double>(),
                             value["quality"].get<double>(), value.value("valid", true)};
    if (result.anchor_id < 0 || result.range_m <= 0.0 || result.variance_m2 <= 0.0 ||
        result.quality < 0.0 || result.quality > 1.0) return std::nullopt;
    return result;
  } catch (...) { return std::nullopt; }
}

std::optional<HeightAssistSample> parseHeightAssistDatagram(
    const std::string& payload, int expected_vehicle) {
  try {
    const json value = json::parse(payload);
    if (!schema1(value) || value.value("kind", "") != "height_assist" ||
        value.value("vehicle", -1) != expected_vehicle ||
        value.value("source", "") != "SIM_HEIGHT_ASSIST" ||
        !finiteNumber(value, "sim_time_us") || !finiteNumber(value, "z_world_m")) {
      return std::nullopt;
    }
    return HeightAssistSample{value["sim_time_us"].get<uint64_t>(),
                              value["z_world_m"].get<double>(),
                              "SIM_HEIGHT_ASSIST"};
  } catch (...) {
    return std::nullopt;
  }
}

std::optional<PeerEstimate> parsePeerEstimateDatagram(const std::string& payload,
                                                      int expected_vehicle) {
  try {
    const json value = json::parse(payload);
    if (!schema1(value) || value.value("kind", "") != "estimate" ||
        value.value("source", "") != "ESTIMATE" ||
        value.value("vehicle", -1) == expected_vehicle ||
        !finiteNumber(value, "sim_time_us") || !finiteNumber(value, "x_m") ||
        !finiteNumber(value, "y_m") || !finiteNumber(value, "z_m") ||
        !finiteNumber(value, "yaw_rad")) {
      return std::nullopt;
    }
    // 保留源观测身份和传播路径；报文到达并不刷新 reference_time_us。
    PeerEstimate result;
    result.sim_time_us = value["sim_time_us"].get<uint64_t>();
    result.vehicle_id = value["vehicle"].get<int>();
    result.pose.x_m = value["x_m"].get<double>();
    result.pose.y_m = value["y_m"].get<double>();
    result.pose.z_m = value["z_m"].get<double>();
    result.pose.yaw_rad = value["yaw_rad"].get<double>();
    result.velocity_enu_mps.x() = value.value("vx_mps", 0.0);
    result.velocity_enu_mps.y() = value.value("vy_mps", 0.0);
    result.velocity_enu_mps.z() = value.value("vz_mps", 0.0);
    result.height_assisted = value.value("height_assisted", false);
    result.xy_structural_valid = value.value("xy_structural_valid", false);
    result.xy_peer_valid = value.value("xy_peer_valid", false);
    result.reference_origin = value.value("reference_origin", -1);
    result.reference_seq = value.value("reference_seq", uint64_t(0));
    result.reference_time_us = value.value("reference_time_us", uint64_t(0));
    result.reference_hops = value.value("reference_hops", 0);
    if (!finiteOptionalArray(value, "reference_path", result.reference_path)) return std::nullopt;
    result.source = "ESTIMATE";
    // 位置协方差按行还原 3×3；缺省或形状不符时保留结构默认的单位矩阵。
    // 数值有限性由 fuseUwb 检查，解析成功还不等同于接受融合。
    if (value.contains("pose_covariance") && value["pose_covariance"].is_array() &&
        value["pose_covariance"].size() == 9) {
      for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
          result.pose_covariance(row, col) = value["pose_covariance"][row * 3 + col].get<double>();
        }
      }
    }
    return result;
  } catch (...) {
    return std::nullopt;
  }
}

std::string serializeEstimateDatagram(const Estimate& estimate) {
  json value = {
      {"schema", 1},
      {"kind", "estimate"},
      {"vehicle", estimate.vehicle_id},
      {"sim_time_us", estimate.sim_time_us},
      {"source", "ESTIMATE"},
      {"x_m", estimate.pose.x_m},
      {"y_m", estimate.pose.y_m},
      {"z_m", estimate.pose.z_m},
      {"yaw_rad", estimate.pose.yaw_rad},
      {"vx_mps", estimate.velocity_enu_mps.x()},
      {"vy_mps", estimate.velocity_enu_mps.y()},
      {"vz_mps", estimate.velocity_enu_mps.z()},
      {"height_assisted", estimate.height_assisted},
      {"vz_derived_from_height", estimate.vz_derived_from_height},
      {"z_valid", estimate.z_valid},
      {"xy_valid", estimate.xy_valid},
      {"xy_structural_valid", estimate.xy_structural_valid},
      {"xy_peer_valid", estimate.xy_peer_valid},
      {"spawn_prior_valid", estimate.spawn_prior_valid},
      {"degenerate_geometry", estimate.degenerate_geometry},
      {"flow_valid", estimate.flow_valid},
      {"position_confidence", estimate.position_confidence},
      {"xy_uncertainty_m", estimate.xy_uncertainty_m},
      {"localization_mode", estimate.localization_mode},
      {"reference_origin", estimate.reference_origin},
      {"reference_seq", estimate.reference_seq},
      {"reference_time_us", estimate.reference_time_us},
      {"reference_hops", estimate.reference_hops},
      {"reference_path", estimate.reference_path},
      {"pose_covariance", json::array()},
  };
  // 仅发送 [x,y,z] 的位置协方差；yaw 与速度协方差留在本机七维滤波器内。
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      value["pose_covariance"].push_back(estimate.covariance(row, col));
    }
  }
  return value.dump();
}

std::string serializeControlDatagram(const ControlIntention& intention) {
  json value = {
      {"schema", 1},
      {"kind", "control_intention"},
      {"sim_time_us", intention.sim_time_us},
      {"source", "ESTIMATE"},
      {"mode", intention.mode},
      {"vx_flu_mps", intention.velocity_flu_mps.x()},
      {"vy_flu_mps", intention.velocity_flu_mps.y()},
      {"vz_flu_mps", intention.velocity_flu_mps.z()},
      {"yaw_rate_ref_rad_s", intention.yaw_rate_ref_rad_s},
      {"target_yaw_error_rad", intention.target_yaw_error_rad},
      {"target_range_m", intention.target_range_m},
      {"target_valid", intention.target_valid},
      {"avoidance_active", intention.avoidance_active},
      {"safe_to_arm", intention.safe_to_arm},
      {"waypoint_valid", intention.waypoint_valid},
      {"waypoint_index", intention.waypoint_index},
  };
  return value.dump();
}

}  // namespace companion
