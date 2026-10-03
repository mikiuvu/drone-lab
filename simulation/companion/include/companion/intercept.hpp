/** @brief 滚动相遇点规划。输入全部为同一时刻的世界系估计，不读取场地目标真值。 */
#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
namespace companion {
struct InterceptPlan {
  bool valid{false}, reachable{false};
  double arrival_s{0};
  Eigen::Vector3d aim{Eigen::Vector3d::Zero()}, velocity{Eigen::Vector3d::Zero()};
};
inline InterceptPlan planIntercept(const Eigen::Vector3d& self,
    const Eigen::Vector3d& self_velocity,const Eigen::Vector3d& target,
    const Eigen::Vector3d& target_velocity,double speed=1.2,double accel=1.5,
    double horizon=.8) {
  InterceptPlan result;
  if(!self.allFinite()||!self_velocity.allFinite()||!target.allFinite()||!target_velocity.allFinite()||
     !std::isfinite(speed)||!std::isfinite(accel)||!std::isfinite(horizon)||speed<=0||accel<=0||horizon<.05)return result;
  // 搜索可达时间：把本机现有沿目标方向速度计入加速段，速度达到上限后匀速。
  double selected=horizon;
  for(double t=.05;t<=horizon+1e-9;t+=.025){
    const Eigen::Vector3d delta=target+target_velocity*t-self;
    double distance=delta.norm();if(distance<1e-6){selected=t;result.reachable=true;break;}
    const double initial=std::clamp(self_velocity.dot(delta/distance),-speed,speed);
    const double ramp=std::min(t,(speed-initial)/accel);
    const double travel=initial*ramp+.5*accel*ramp*ramp+speed*(t-ramp);
    // 横向初速度需要被纠正，不把纯径向可达性当成严格动力学保证。
    const double transverse=(self_velocity-initial*delta/distance).norm();
    if(travel>=distance&&transverse<=accel*t){selected=t;result.reachable=true;break;}
  }
  result.valid=true;result.arrival_s=selected;
  result.aim=target+target_velocity*selected;
  Eigen::Vector3d delta=result.aim-self;
  if(delta.norm()>1e-6){
    // 未在可信预测窗内可达时先朝有界预测点运动，每次新观测重新规划。
    const double command=std::min(speed,delta.norm()/std::max(.05,selected));
    result.velocity=delta.normalized()*command;
  }
  return result;
}
}
