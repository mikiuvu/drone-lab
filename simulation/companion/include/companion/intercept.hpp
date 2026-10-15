/** @brief 滚动相遇点规划。输入全部为同一时刻的世界系估计，不读取场地目标真值。 */
#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
namespace companion {
/** @brief 目标邻域内的加速度计比力突增用于触发姿态恢复；输入单位 m/s²、m。 */
inline bool inertialContactCandidate(const Eigen::Vector3d& specific_force,
    const Eigen::Vector3d& target_displacement,double force_threshold,double proximity){
  if(!specific_force.allFinite()||!target_displacement.allFinite()||!std::isfinite(force_threshold)||
     !std::isfinite(proximity)||force_threshold<=0||proximity<=0)return false;
  const double force=specific_force.norm(),distance=target_displacement.norm();
  return std::isfinite(force)&&std::isfinite(distance)&&force>force_threshold&&distance<=proximity;
}
/** @brief 独立冲击样本保留首次时刻与峰值，等待压力确认；重复采样不延长等待。 */
struct PendingInertialContact {
  uint64_t since_us{0};
  double peak_specific_force_mps2{0};
  bool observe(uint64_t stamp,double force){
    if(!stamp||!std::isfinite(force)||force<=0||(since_us&&stamp<since_us))return false;
    const bool first=!since_us;
    if(first)since_us=stamp;
    peak_specific_force_mps2=std::max(peak_specific_force_mps2,force);
    return first;
  }
  bool mature(uint64_t now,uint64_t delay_us) const {
    return since_us&&now>=since_us&&now-since_us>=delay_us;
  }
  void clear(){since_us=0;peak_specific_force_mps2=0;}
};
/** @brief 按同一剩余时间缩放机体系位移，满足水平、侧向和垂向速度上限；单位 m、m/s。 */
inline std::optional<Eigen::Vector3d> terminalVelocity(const Eigen::Vector3d& displacement,
    double horizontal_limit,double lateral_limit,double vertical_limit){
  if(!displacement.allFinite()||displacement.x()<=0||!std::isfinite(horizontal_limit)||
     !std::isfinite(lateral_limit)||!std::isfinite(vertical_limit)||horizontal_limit<=0||
     lateral_limit<=0||vertical_limit<=0)return std::nullopt;
  const double horizontal=std::hypot(displacement.x(),displacement.y());
  if(!std::isfinite(horizontal)||horizontal<=1e-6)return std::nullopt;
  double scale=horizontal_limit/horizontal;
  if(std::abs(displacement.y())>0)scale=std::min(scale,lateral_limit/std::abs(displacement.y()));
  if(std::abs(displacement.z())>0)scale=std::min(scale,vertical_limit/std::abs(displacement.z()));
  const Eigen::Vector3d velocity=displacement*scale;
  if(!std::isfinite(scale)||scale<=0||!velocity.allFinite())return std::nullopt;
  return velocity;
}
/** @brief 提交时前向速度不超过三轴限速比例允许值，垂向不得远离目标；静止容差 0.05 m/s。 */
inline bool terminalEntryVelocityCompatible(const Eigen::Vector3d& displacement,
    const Eigen::Vector3d& velocity,double horizontal_limit,double lateral_limit,
    double vertical_limit,double min_forward){
  if(!velocity.allFinite()||!std::isfinite(min_forward)||min_forward<=0)return false;
  const auto reference=terminalVelocity(displacement,horizontal_limit,lateral_limit,vertical_limit);
  if(!reference||reference->x()<std::max(min_forward,velocity.x()))return false;
  return !((displacement.z()<0&&velocity.z()>.05)||(displacement.z()>0&&velocity.z()<-.05));
}
/** @brief 剩余接触时窗须覆盖一次俯冲上限及 20 ms 导航周期；deadline=0 表示不限时。 */
inline bool diveAttemptFitsWindow(uint64_t now,uint64_t deadline,double duration_s){
  if(!std::isfinite(duration_s)||duration_s<=0)return false;
  return !deadline||(now<deadline&&static_cast<double>(deadline-now)>std::ceil(duration_s*1e6)+20000.0);
}
/** @brief 本机沿任务航向超过目标平面的裕量后返回 true；位置及裕量单位 m。 */
inline bool passedTargetPlane(const Eigen::Vector3d& self,const Eigen::Vector3d& target,
    double heading,double margin){
  if(!self.allFinite()||!target.allFinite()||!std::isfinite(heading)||
     !std::isfinite(margin)||margin<0)return false;
  const Eigen::Vector3d forward(std::cos(heading),std::sin(heading),0);
  const double distance=(self-target).dot(forward);
  return std::isfinite(distance)&&distance>margin;
}
/** @brief 提交俯冲前核验独立曝光、入口高度、航向和前向运动；世界坐标单位 m、m/s。
 * @note 允许前向速度 −0.05 m/s 的静止估计误差；目标须位于航向前方至少 50 mm。
 */
inline bool diveCommitEligible(const Eigen::Vector3d& self,const Eigen::Vector3d& velocity,
    const Eigen::Vector3d& aim,double yaw,double heading,double entry_z,
    uint64_t exposure,uint64_t now,double height_tolerance=.12,double yaw_tolerance=.35){
  if(!self.allFinite()||!velocity.allFinite()||!aim.allFinite()||!std::isfinite(yaw)||
     !std::isfinite(heading)||!std::isfinite(entry_z)||!std::isfinite(height_tolerance)||
     !std::isfinite(yaw_tolerance)||height_tolerance<=0||yaw_tolerance<=0||
     !exposure||exposure>now||now-exposure>150000)return false;
  const Eigen::Vector3d forward(std::cos(heading),std::sin(heading),0);
  return (aim-self).dot(forward)>.05&&velocity.dot(forward)>=-.05&&
      std::abs(self.z()-entry_z)<=height_tolerance&&
      std::abs(std::remainder(yaw-heading,6.283185307179586))<=yaw_tolerance;
}
/** @brief 沿滑轨作限速线性预测，可按已知半行程反射；输入和输出为世界 Y，单位 m。
 * @param reflection_half_width 半行程；0 表示不预测边界反向。horizon 单位 s。
 */
inline std::optional<double> predictRailLateral(double position,double velocity,double horizon,
    double center,double speed_limit,double reflection_half_width=0){
  if(!std::isfinite(position)||!std::isfinite(velocity)||!std::isfinite(horizon)||
     !std::isfinite(center)||!std::isfinite(speed_limit)||!std::isfinite(reflection_half_width)||
     horizon<0||speed_limit<=0||reflection_half_width<0)return std::nullopt;
  double initial=position;
  if(reflection_half_width>0){
    if(!std::isfinite(center-reflection_half_width)||!std::isfinite(center+reflection_half_width))return std::nullopt;
    initial=std::clamp(initial,center-reflection_half_width,center+reflection_half_width);
  }
  const double projected=initial+std::clamp(velocity,-speed_limit,speed_limit)*horizon;
  if(!std::isfinite(projected))return std::nullopt;
  if(reflection_half_width==0)return projected;
  const double span=2*reflection_half_width,period=2*span,lower=center-reflection_half_width;
  if(!std::isfinite(period)||!std::isfinite(projected-lower))return std::nullopt;
  double phase=std::fmod(projected-lower,period);
  if(phase<0)phase+=period;
  return lower+(phase<=span?phase:period-phase);
}
/** @brief 已知当前沿目标方向速度时，按恒加速度、随后限速估计到达时间，单位 s。
 * @param distance 沿该方向的剩余距离，m；initial_speed 允许为负，m/s。
 */
inline std::optional<double> estimateArrivalTime(double distance,double initial_speed,double speed_limit,double acceleration){
  if(!std::isfinite(distance)||!std::isfinite(initial_speed)||!std::isfinite(speed_limit)||
      !std::isfinite(acceleration)||distance<0||speed_limit<=0||acceleration<=0)return std::nullopt;
  if(distance==0)return 0.0;
  const double initial=std::clamp(initial_speed,-speed_limit,speed_limit);
  const double ramp=(speed_limit-initial)/acceleration;
  const double ramp_distance=(initial+speed_limit)*.5*ramp;
  if(!std::isfinite(ramp)||!std::isfinite(ramp_distance))return std::nullopt;
  if(distance>=ramp_distance){
    const double time=ramp+(distance-ramp_distance)/speed_limit;
    return std::isfinite(time)?std::optional<double>(time):std::nullopt;
  }
  const double root=std::sqrt(initial*initial+2*acceleration*distance);
  if(!std::isfinite(root))return std::nullopt;
  const double time=initial>=0?2*distance/(initial+root):(root-initial)/acceleration;
  return std::isfinite(time)?std::optional<double>(time):std::nullopt;
}

/** @brief 对新的独立曝光修正瞄准 Y；陈旧、未来及重复曝光不改变目标。
 * @param max_rate 横向瞄准点最大修正速度，m/s；0 表示保持原瞄准点。
 */
inline bool correctLateralAim(double& aim,uint64_t& last_exposure,double candidate,
    uint64_t exposure,uint64_t now,double max_rate,double max_age_s=.15){
  if(!std::isfinite(aim)||!std::isfinite(candidate)||!std::isfinite(max_rate)||max_rate<=0||
     !std::isfinite(max_age_s)||max_age_s<=0||!exposure||exposure>now||
     exposure<=last_exposure||(now-exposure)*1e-6>max_age_s)return false;
  const double dt=std::min(max_age_s,last_exposure?(exposure-last_exposure)*1e-6:(now-exposure)*1e-6);
  const double correction=std::clamp(candidate-aim,-max_rate*dt,max_rate*dt);
  aim+=correction;last_exposure=exposure;return true;
}
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
