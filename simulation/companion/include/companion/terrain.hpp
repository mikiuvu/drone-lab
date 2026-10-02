#pragma once

#include <array>
#include <filesystem>
#include <memory>
#include <cstddef>
#include <vector>
#include <Eigen/Core>

namespace companion {

// Static world ENU geometry only. Depth along an unnormalised optical ray is axial.
class TerrainMap {
 public:
  using Triangle = std::array<Eigen::Vector3f, 3>;
  struct Hit { double depth_m{0}; Eigen::Vector3d point, normal; };
  explicit TerrainMap(std::vector<Triangle> triangles);
  static std::shared_ptr<const TerrainMap> load(const std::filesystem::path& path);
  bool raycast(const Eigen::Vector3d& origin, const Eigen::Vector3d& ray,
               double max_depth, Hit& hit) const;
  size_t triangleCount() const { return triangles_.size(); }
  size_t storageBytes() const;

 private:
  struct Node {
    Eigen::Vector3f low, high;
    uint32_t begin{0}, count{0}, left{0}, right{0};
  };
  uint32_t build(uint32_t begin, uint32_t end);
  std::vector<Triangle> triangles_;
  std::vector<uint32_t> order_;
  std::vector<Node> nodes_;
};
}
