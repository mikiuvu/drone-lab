/**
 * @file terrain.cpp
 * @brief 三角网格校验、AABB 索引构建和最近射线交点计算。
 */
#include "companion/terrain.hpp"
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <numeric>
#include <limits>
#include <stdexcept>

namespace companion {
namespace {
constexpr size_t kMaxTriangles = 1000000;
constexpr uint32_t kLeafTriangleLimit = 8;
constexpr int kNodeVisitLimit = 4096;
bool boxHit(const Eigen::Vector3f& low, const Eigen::Vector3f& high,
            const Eigen::Vector3d& origin, const Eigen::Vector3d& ray,const Eigen::Vector3d& inverse_ray,
            double far, double& near) {
  // Slab 方法：分别计算射线在三个轴向区间的进入/离开参数，取交集 [near,far]。
  // 与某轴平行时不能除以近零方向分量，只检查起点是否位于该轴区间内。
  near = 0;
  for (int a=0; a<3; ++a) {
    if (std::abs(ray(a)) < 1e-12) {
      if (origin(a)<low(a)-1e-6 || origin(a)>high(a)+1e-6) return false;
    } else {
      double lo=(low(a)-1e-6-origin(a))*inverse_ray(a), hi=(high(a)+1e-6-origin(a))*inverse_ray(a);
      if(lo>hi)std::swap(lo,hi);
      near=std::max(near,lo); far=std::min(far,hi);
      if(near>far)return false;
    }
  }
  return true;
}
}

TerrainMap::TerrainMap(std::vector<Triangle> triangles) {
  if(triangles.empty() || triangles.size()>kMaxTriangles)
    throw std::invalid_argument("terrain triangle count must be 1..1000000");
  // 拒绝非有限顶点；退化三角形直接丢弃，不能凭空为其构造法向或平面。
  for(const auto& t:triangles) {
    for(const auto& p:t)if(!p.allFinite() || p.cwiseAbs().maxCoeff()>100000)
      throw std::invalid_argument("terrain vertex invalid");
    const Eigen::Vector3d a=(t[1]-t[0]).cast<double>(), b=(t[2]-t[0]).cast<double>();
    if(a.cross(b).squaredNorm()>1e-16)triangles_.push_back(t);
  }
  if(triangles_.empty())throw std::invalid_argument("terrain has no nondegenerate faces");
  order_.resize(triangles_.size());std::iota(order_.begin(),order_.end(),0);
  // 中位数二分树的完整层数给出节点容量上界，避免建树过程中反复扩容。
  size_t leaf_capacity=1;
  while(leaf_capacity*kLeafTriangleLimit<triangles_.size())leaf_capacity*=2;
  nodes_.reserve(2*leaf_capacity-1);build(0,order_.size());
  // 建树后按索引重排三角形，叶子可直接连续访问，临时顺序表随即释放。
  std::vector<Triangle> reordered;reordered.reserve(triangles_.size());
  for(const auto index:order_)reordered.push_back(triangles_[index]);
  triangles_.swap(reordered);std::vector<uint32_t>().swap(order_);
  parallel_thresholds_.reserve(triangles_.size());
  for(const auto& triangle:triangles_){
    const Eigen::Vector3d vertex=triangle[0].cast<double>();
    const Eigen::Vector3d e1=triangle[1].cast<double>()-vertex,e2=triangle[2].cast<double>()-vertex;
    parallel_thresholds_.push_back(1e-10*e1.norm()*e2.norm());
  }
}

std::shared_ptr<const TerrainMap> TerrainMap::load(const std::filesystem::path& path) {
  static_assert(sizeof(float)==4 && std::numeric_limits<float>::is_iec559);
  std::ifstream file(path,std::ios::binary|std::ios::ate);
  if(!file)throw std::runtime_error("cannot read terrain: "+path.string());
  const auto bytes=file.tellg();
  if(bytes<=0 || bytes%36!=0 || bytes/36>static_cast<std::streamoff>(kMaxTriangles))
    throw std::runtime_error("terrain must be raw little-endian float32 Nx3x3, 1..1000000 faces");
  file.seekg(0);std::vector<Triangle> triangles(static_cast<size_t>(bytes/36));
  // 文件无头，每面 9 个 float32；显式拼小端字节，再复制位模式，避免宿主字节序影响。
  for(auto& t:triangles)for(auto& p:t)for(int a=0;a<3;++a) {
    unsigned char b[4];if(!file.read(reinterpret_cast<char*>(b),4))throw std::runtime_error("terrain truncated");
    uint32_t word=uint32_t(b[0])|(uint32_t(b[1])<<8)|(uint32_t(b[2])<<16)|(uint32_t(b[3])<<24);
    float value;std::memcpy(&value,&word,4);p(a)=value;
  }
  return std::make_shared<const TerrainMap>(std::move(triangles));
}

uint32_t TerrainMap::build(uint32_t begin,uint32_t end) {
  Node node;node.low=Eigen::Vector3f::Constant(std::numeric_limits<float>::infinity());
  node.high=-node.low;node.begin=begin;node.count=end-begin;
  for(uint32_t i=begin;i<end;++i)for(const auto& p:triangles_[order_[i]]) {
    node.low=node.low.cwiseMin(p);node.high=node.high.cwiseMax(p);
  }
  const uint32_t id=nodes_.size();nodes_.push_back(node);
  // 包围盒最长轴按三角形重心中位数二分；小叶子减少同一包围盒内的无关三角形测试。
  if(end-begin>kLeafTriangleLimit) {
    Eigen::Index axis; (node.high-node.low).maxCoeff(&axis);
    const auto mid=begin+(end-begin)/2;
    std::nth_element(order_.begin()+begin,order_.begin()+mid,order_.begin()+end,
      [&](uint32_t a,uint32_t b){const auto& x=triangles_[a];const auto& y=triangles_[b];
        return x[0](axis)+x[1](axis)+x[2](axis)<y[0](axis)+y[1](axis)+y[2](axis);});
    const auto left=build(begin,mid),right=build(mid,end);
    nodes_[id].left=left;nodes_[id].right=right;nodes_[id].count=0;
  }
  return id;
}

bool TerrainMap::raycast(const Eigen::Vector3d& origin,const Eigen::Vector3d& ray,
                         double max_depth,Hit& hit) const {
  if(!origin.allFinite()||!ray.allFinite()||!std::isfinite(max_depth)||max_depth<=0)return false;
  const double ray_norm=ray.norm();
  if(ray_norm<1e-9)return false;
  Eigen::Vector3d inverse_ray=Eigen::Vector3d::Zero();
  for(int axis=0;axis<3;++axis)if(std::abs(ray(axis))>=1e-12)inverse_ray(axis)=1/ray(axis);
  double root_near;
  if(!boxHit(nodes_[0].low,nodes_[0].high,origin,ray,inverse_ray,max_depth,root_near))return false;
  struct Visit { uint32_t node; double near; };
  // 每个有效槽位在入栈时完整赋值，未使用槽位无需清零。
  std::array<Visit,64> stack;size_t pending=1;stack[0]={0,root_near};
  int visits=0;bool found=false;double best=max_depth;
  while(pending) {
    // 超过资源上限时保守失败，未遍历完的结果不能证明是最近可见表面。
    if(++visits>kNodeVisitLimit)return false;
    const auto visit=stack[--pending];
    // 入栈时已验证包围盒；最近命中缩短后，只需比较缓存的进入深度。
    if(visit.near>best)continue;
    const auto& n=nodes_[visit.node];
    if(!n.count) {
      double a,b;const auto& l=nodes_[n.left];const auto& r=nodes_[n.right];
      const bool hl=boxHit(l.low,l.high,origin,ray,inverse_ray,best,a),hr=boxHit(r.low,r.high,origin,ray,inverse_ray,best,b);
      if(pending+2>stack.size())return false;
      // 栈后进先出，较近子树最后压入、最先处理；best 随命中缩短，可提前剪枝远处节点。
      if(hl&&hr){
        if(a<b){stack[pending++]={n.right,b};stack[pending++]={n.left,a};}
        else{stack[pending++]={n.left,a};stack[pending++]={n.right,b};}
      }else if(hl)stack[pending++]={n.left,a};else if(hr)stack[pending++]={n.right,b};
      continue;
    }
    for(uint32_t i=n.begin;i<n.begin+n.count;++i) {
      const auto& t=triangles_[i];const Eigen::Vector3d v=t[0].cast<double>();
      const Eigen::Vector3d e1=t[1].cast<double>()-v,e2=t[2].cast<double>()-v;
      // Möller–Trumbore 求解 origin+d*ray=v+u*e1+w*e2；u,w 是重心坐标，d 是射线参数。
      // det 接近零表示射线与面近乎平行；u≥0、w≥0、u+w≤1 才落在三角形内。
      const Eigen::Vector3d p=ray.cross(e2);const double det=e1.dot(p);
      if(std::abs(det)<parallel_thresholds_[i]*ray_norm)continue;
      const Eigen::Vector3d s=origin-v;
      const double inverse_det=1/det;
      const double u=s.dot(p)*inverse_det;
      if(u< -1e-7)continue;
      const Eigen::Vector3d q=s.cross(e1);
      const double w=ray.dot(q)*inverse_det;
      if(w< -1e-7||u+w>1+1e-7)continue;
      const double d=e2.dot(q)*inverse_det;
      if(d<1e-6||d>best)continue;
      best=d;found=true;hit.depth_m=d;hit.point=origin+d*ray;hit.normal=e1.cross(e2).normalized();
    }
  }
  return found;
}
size_t TerrainMap::storageBytes() const {
  return triangles_.capacity()*sizeof(Triangle)+order_.capacity()*sizeof(uint32_t)+nodes_.capacity()*sizeof(Node)+parallel_thresholds_.capacity()*sizeof(double);
}
}
