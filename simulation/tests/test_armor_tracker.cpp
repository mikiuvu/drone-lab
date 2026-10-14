#include "companion/armor_tracker.hpp"
#include <cassert>
#include <stdexcept>

int main(){
  for(double noise:{0.0,-1.0,std::numeric_limits<double>::infinity()}){
    bool rejected=false;try{companion::ArmorTracker invalid(noise,.04);}catch(const std::invalid_argument&){rejected=true;}
    assert(rejected);
  }
  // 低速滑轨的曝光噪声不能形成大幅虚假速度，重捕获保持相同速度先验。
  companion::ArmorTracker smooth(.04,.04),reference;
  for(uint64_t i=1;i<=30;++i){
    const auto stamp=i*66000;
    const double y=.2*stamp*1e-6+(i==30?.18:0);
    smooth.observe(stamp,{11,y,1.4},.0064);reference.observe(stamp,{11,y,1.4},.0064);
  }
  const auto filtered=smooth.at(1980000),noisy=reference.at(1980000);
  const double truth=.2*(1.98+.5);
  const double filtered_error=std::abs(filtered.position.y()+.5*filtered.velocity.y()-truth);
  const double noisy_error=std::abs(noisy.position.y()+.5*noisy.velocity.y()-truth);
  assert(filtered_error<.5*noisy_error);
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
