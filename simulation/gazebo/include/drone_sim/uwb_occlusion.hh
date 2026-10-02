#pragma once

#include <cmath>
#include "companion/terrain.hpp"

namespace drone_sim
{
// Static world ENU segment query, with a 0.1 mm exclusion at both endpoints.
// Direction is normalized: TerrainMap hit depth is then distance in metres.
inline bool TerrainOccludesPeer(const companion::TerrainMap *terrain,
    const Eigen::Vector3d &observer, const Eigen::Vector3d &peer)
{
  constexpr double endpointMarginM = 1e-4;
  if (!terrain || !observer.allFinite() || !peer.allFinite())
    return false;
  const Eigen::Vector3d delta = peer - observer;
  const double distance = delta.norm();
  if (!std::isfinite(distance) || distance <= 2.0 * endpointMarginM)
    return false;
  const Eigen::Vector3d direction = delta / distance;
  companion::TerrainMap::Hit hit;
  return terrain->raycast(observer + endpointMarginM * direction, direction,
      distance - 2.0 * endpointMarginM, hit);
}
}  // namespace drone_sim
