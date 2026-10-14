/**
 * @file terrain.hpp
 * @brief 静态三角网格载入、包围盒索引和轴向深度射线查询。
 */
#pragma once

#include <array>
#include <filesystem>
#include <memory>
#include <cstddef>
#include <vector>
#include <Eigen/Core>

namespace companion {

/**
 * @brief 持有静态 ENU 三角网格，并用包围体层次结构加速射线查询。
 * @note 射线未归一化，光学轴分量为 1 时 depth_m 为轴向深度，不是欧氏距离。
 */
class TerrainMap {
 public:
  // 三顶点位置，单位 m；内部以 float 存储，求交时提升为 double。
  using Triangle = std::array<Eigen::Vector3f, 3>;
  // depth_m 为射线参数 t；point=origin+t*ray，normal 为顶点绕序决定的单位面法向。
  struct Hit { double depth_m{0}; Eigen::Vector3d point, normal; };
/**
 * @brief 校验顶点、剔除零面积面并构建空间索引。
 * @param triangles 1..1000000 个 ENU 三角形；非法数据或全部退化时抛异常。
 */
  explicit TerrainMap(std::vector<Triangle> triangles);
/**
 * @brief 加载原始小端 float32 三角形数组。
 * @param path 无文件头的 N×3×3 顶点文件，每面 36 字节，坐标单位 m。
 * @return 可共享的只读地图；读取、格式或几何校验失败时抛异常。
 */
  static std::shared_ptr<const TerrainMap> load(const std::filesystem::path& path);
/**
 * @brief 查找给定射线上最近的可见三角形。
 * @param origin ENU 起点，单位 m。
 * @param ray 未归一化方向；ray 的尺度决定深度参数含义。
 * @param max_depth 最大正射线参数，光学轴分量为 1 时表示轴向深度 m。
 * @param hit 成功时输出交点；失败时不要使用其内容。
 * @return 命中且遍历完整时为 true；参数非法、未命中或超过遍历资源上限时为 false。
 */
  bool raycast(const Eigen::Vector3d& origin, const Eigen::Vector3d& ray,
               double max_depth, Hit& hit) const;
/**
 * @brief 返回剔除退化面后的实际三角形数量。
 */
  size_t triangleCount() const { return triangles_.size(); }
/**
 * @brief 估算四个内部 vector 的容量占用字节数，不含对象本身及分配器开销。
 */
  size_t storageBytes() const;

 private:
  // AABB 节点：low/high 为 ENU 边界 m；count>0 为叶子，begin/count 指向连续三角形。
  struct Node {
    Eigen::Vector3f low, high;
    uint32_t begin{0}, count{0}, left{0}, right{0};
  };
  uint32_t build(uint32_t begin, uint32_t end);
  std::vector<Triangle> triangles_;
  std::vector<double> parallel_thresholds_;  ///< 每面 1e-10*|e1|*|e2|，免去每次求交的重复开方。
  std::vector<uint32_t> order_;
  std::vector<Node> nodes_;
};
}
