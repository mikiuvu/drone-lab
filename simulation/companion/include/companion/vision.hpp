#pragma once

#include <optional>
#include <string>
#include <vector>

#include "companion/types.hpp"

namespace companion {

std::optional<PpmImage> readPpm(const std::string& path);
ArmorObservation detectArmorStrips(const PpmImage& image, const CameraConfig& config);
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
