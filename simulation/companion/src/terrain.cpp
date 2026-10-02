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
bool boxHit(const Eigen::Vector3f& low, const Eigen::Vector3f& high,
            const Eigen::Vector3d& origin, const Eigen::Vector3d& ray,
            double far, double& near) {
  near = 0;
  for (int a=0; a<3; ++a) {
    if (std::abs(ray(a)) < 1e-12) {
      if (origin(a)<low(a)-1e-6 || origin(a)>high(a)+1e-6) return false;
    } else {
      double lo=(low(a)-1e-6-origin(a))/ray(a), hi=(high(a)+1e-6-origin(a))/ray(a);
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
  // Reject nonfinite input; discard zero-area faces without inventing a plane.
  for(const auto& t:triangles) {
    for(const auto& p:t)if(!p.allFinite() || p.cwiseAbs().maxCoeff()>100000)
      throw std::invalid_argument("terrain vertex invalid");
    const Eigen::Vector3d a=(t[1]-t[0]).cast<double>(), b=(t[2]-t[0]).cast<double>();
    if(a.cross(b).squaredNorm()>1e-16)triangles_.push_back(t);
  }
  if(triangles_.empty())throw std::invalid_argument("terrain has no nondegenerate faces");
  order_.resize(triangles_.size());std::iota(order_.begin(),order_.end(),0);
  nodes_.reserve(triangles_.size()/16+32);build(0,order_.size());
  std::vector<Triangle> reordered;reordered.reserve(triangles_.size());
  for(const auto index:order_)reordered.push_back(triangles_[index]);
  triangles_.swap(reordered);std::vector<uint32_t>().swap(order_);
}

std::shared_ptr<const TerrainMap> TerrainMap::load(const std::filesystem::path& path) {
  static_assert(sizeof(float)==4 && std::numeric_limits<float>::is_iec559);
  std::ifstream file(path,std::ios::binary|std::ios::ate);
  if(!file)throw std::runtime_error("cannot read terrain: "+path.string());
  const auto bytes=file.tellg();
  if(bytes<=0 || bytes%36!=0 || bytes/36>static_cast<std::streamoff>(kMaxTriangles))
    throw std::runtime_error("terrain must be raw little-endian float32 Nx3x3, 1..1000000 faces");
  file.seekg(0);std::vector<Triangle> triangles(static_cast<size_t>(bytes/36));
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
  if(end-begin>32) {
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
  if(!origin.allFinite()||!ray.allFinite()||!std::isfinite(max_depth)||max_depth<=0||ray.norm()<1e-9)return false;
  std::array<uint32_t,64> stack{};size_t pending=1;stack[0]=0;
  int visits=0;bool found=false;double best=max_depth;
  while(pending) {
    // Fail closed: an incomplete traversal cannot certify the first visible surface.
    if(++visits>1024)return false;
    const auto& n=nodes_[stack[--pending]];double near;
    if(!boxHit(n.low,n.high,origin,ray,best,near))continue;
    if(!n.count) {
      double a,b;const auto& l=nodes_[n.left];const auto& r=nodes_[n.right];
      const bool hl=boxHit(l.low,l.high,origin,ray,best,a),hr=boxHit(r.low,r.high,origin,ray,best,b);
      if(pending+2>stack.size())return false;
      if(hl&&hr){stack[pending++]=a<b?n.right:n.left;stack[pending++]=a<b?n.left:n.right;}
      else if(hl)stack[pending++]=n.left;else if(hr)stack[pending++]=n.right;
      continue;
    }
    for(uint32_t i=n.begin;i<n.begin+n.count;++i) {
      const auto& t=triangles_[i];const Eigen::Vector3d v=t[0].cast<double>();
      const Eigen::Vector3d e1=t[1].cast<double>()-v,e2=t[2].cast<double>()-v;
      const Eigen::Vector3d p=ray.cross(e2);const double det=e1.dot(p);
      if(std::abs(det)<1e-10*e1.norm()*e2.norm()*ray.norm())continue;
      const Eigen::Vector3d s=origin-v,q=s.cross(e1);
      const double u=s.dot(p)/det,w=ray.dot(q)/det,d=e2.dot(q)/det;
      if(u< -1e-7||w< -1e-7||u+w>1+1e-7||d<1e-6||d>best)continue;
      best=d;found=true;hit.depth_m=d;hit.point=origin+d*ray;hit.normal=e1.cross(e2).normalized();
    }
  }
  return found;
}
size_t TerrainMap::storageBytes() const {
  return triangles_.capacity()*sizeof(Triangle)+order_.capacity()*sizeof(uint32_t)+nodes_.capacity()*sizeof(Node);
}
}
