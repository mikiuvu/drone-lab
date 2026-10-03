#include "companion/armor_tracker.hpp"
#include <cassert>

int main(){
  companion::ArmorTracker tracker;
  for(uint64_t i=1;i<=80;++i){
    const double t=i*.05;
    assert(tracker.observe(i*50000,Eigen::Vector3d(11,t*.2,1)));
  }
  const auto result=tracker.at(4050000);
  assert(result.valid&&result.measured);
  assert(std::abs(result.velocity.y()-.2)<.03);
  assert(std::abs(result.position.y()-.81)<.02);
  assert(!tracker.observe(4000000,Eigen::Vector3d(100,100,100)));
  assert(!tracker.at(4900000).valid);
  assert(!tracker.observe(4100000,Eigen::Vector3d(20,20,20)));
  // 曝光时刻的机体位姿补偿：静止目标不能随相机平移而获得假速度。
  companion::ArmorTracker fixed;
  for(uint64_t i=1;i<=80;++i){
    const Eigen::Vector3d camera_pose(i*.01,i*.02,1);
    const Eigen::Vector3d measured_relative=Eigen::Vector3d(11,0,1)-camera_pose;
    fixed.observe(i*50000,camera_pose+measured_relative);
  }
  assert(fixed.at(4000000).velocity.norm()<1e-8);
}
