#include <cassert>
#include <cmath>
#include <iostream>

#include "drone_sim/uwb_occlusion.hh"

using companion::TerrainMap;

int main()
{
  using T = TerrainMap::Triangle;
  // A vertical wall at x=0 blocks the segment from x=-1 to x=1.
  TerrainMap wall({
      T{Eigen::Vector3f(0, -2, 0), Eigen::Vector3f(0, 2, 0), Eigen::Vector3f(0, 2, 3)},
      T{Eigen::Vector3f(0, -2, 0), Eigen::Vector3f(0, 2, 3), Eigen::Vector3f(0, -2, 3)},
  });
  const Eigen::Vector3d origin(-1, 0, 1);
  const Eigen::Vector3d peer(1, 0, 1);
  assert(drone_sim::TerrainOccludesPeer(&wall, origin, peer));

  // The exact peer endpoint is excluded from the query, so endpoint geometry
  // cannot manufacture an occlusion when there is no earlier intersection.
  TerrainMap endpoint({
      T{Eigen::Vector3f(1, -1, 0), Eigen::Vector3f(1, 1, 0), Eigen::Vector3f(1, 1, 2)},
      T{Eigen::Vector3f(1, -1, 0), Eigen::Vector3f(1, 1, 2), Eigen::Vector3f(1, -1, 2)},
  });
  assert(!drone_sim::TerrainOccludesPeer(&endpoint, origin, peer));
  assert(!drone_sim::TerrainOccludesPeer(nullptr, origin, peer));
  assert(!drone_sim::TerrainOccludesPeer(&wall, origin, origin));
  assert(drone_sim::TerrainOccludesPeer(&wall, peer, origin));
  assert(!drone_sim::TerrainOccludesPeer(&wall,
      Eigen::Vector3d(-1, 0, 4), Eigen::Vector3d(1, 0, 4)));
  assert(!drone_sim::TerrainOccludesPeer(&wall, origin,
      Eigen::Vector3d(-0.5, 0, 1))); // Wall beyond the peer.
  assert(!drone_sim::TerrainOccludesPeer(&wall,
      Eigen::Vector3d(0, 0, 1), peer)); // Observer endpoint excluded.
  TerrainMap combined({
      T{Eigen::Vector3f(1, -2, 0), Eigen::Vector3f(1, 2, 0), Eigen::Vector3f(1, 0, 3)},
      T{Eigen::Vector3f(0, -2, 0), Eigen::Vector3f(0, 2, 0), Eigen::Vector3f(0, 0, 3)},
  });
  assert(drone_sim::TerrainOccludesPeer(&combined, origin, peer));

  std::cout << "uwb occlusion geometry: OK\n";
  return 0;
}
