/**
 * @file estimator.cpp
 * @brief 七维状态预测、协方差传播及多源定位观测融合实现。
 */
#include "companion/estimator.hpp"
#include "companion/terrain.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_set>
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

namespace companion { namespace {
// R=Rz(y)Ry(p)Rx(r) 将 FLU 向量转到 ENU；Rᵀ 把世界速度或相对位移转回机体。
Eigen::Matrix3d rotation(double r,double p,double y) {
  return (Eigen::AngleAxisd(y,Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(p,Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(r,Eigen::Vector3d::UnitX())).toRotationMatrix();
}
// Huber 权重：2.5σ 内保持二次代价，外侧按 1/|残差| 降权，使离群深度不主导匹配。
double huber(double e,double s) { const double a=std::abs(e)/s; return a<=2.5?1.0:2.5/a; }
}

/**
 * @brief 校验估计器配置并从初始位姿、协方差开始工作。
 * @param c 将被保存的估计器配置。
 * @note 构造时立即调用 reset，因此缓存、参考序号和保持加速度均为空。
 */
Estimator::Estimator(EstimatorConfig c):config_(std::move(c)) {
  if(config_.vehicle_id<0||config_.tof_sigma_m<=0||!config_.initial_covariance.allFinite()) throw std::invalid_argument("invalid estimator config");
  for(int sensor=0;sensor<3;++sensor)for(int cell=0;cell<kTofCellCount;++cell)
    tof_rays_[sensor][cell]=tofRayBody(config_.tof_mounts[sensor],cell/8,cell%8);
  reset();
}
void Estimator::reset(uint64_t now) {
  state_=Estimate{}; state_.vehicle_id=config_.vehicle_id; state_.pose=config_.initial_pose; state_.covariance=config_.initial_covariance;
  // 初始位姿只是滤波器状态初值；在收到有效 XY 来源前不能把它当作定位量测。
  state_.covariance(2,2)=std::max(state_.covariance(2,2),config_.initial_z_sigma_m*config_.initial_z_sigma_m); state_.initialized=true; state_.xy_valid=false; state_.localization_mode="INITIALIZING"; state_.sim_time_us=now;
  assimilated_tof_us_.fill(0);
  last_landmark_us_=0;
  attitude_begin_=attitude_count_=0;
  last_imu_us_=last_height_us_=last_baro_us_=last_absolute_us_=0; startup_sim_us_=now; last_local_tof_us_=last_peer_xy_us_=last_flow_us_=0; local_tof_rank_=0; flow_valid_=false; next_reference_seq_=0; previous_height_us_=0; baro_reference_hpa_=0; last_imu_yaw_=0; last_tof_us_.fill(0); last_uwb_us_.clear(); last_anchor_us_.clear(); seen_reference_seq_.clear(); accepted_anchors_.clear(); tof_cache_={}; structural_information_.setZero(); structural_information_us_=0; held_acceleration_.setZero(); state_.reference_path.clear();
}
bool Estimator::fresh(uint64_t t,uint64_t timeout) const { return t&&t<=state_.sim_time_us&&state_.sim_time_us-t<=timeout; }
Estimate Estimator::estimate() const {
  Estimate e=state_;
  const double xy_trace=std::max(0.0,e.covariance(0,0)+e.covariance(1,1));
  e.xy_structural_valid=fresh(last_absolute_us_,config_.terrain?std::min<uint64_t>(config_.prediction_timeout_us,500000):config_.prediction_timeout_us)&&xy_trace<2.0;
  const bool flow=fresh(last_flow_us_,config_.flow_timeout_us)&&flow_valid_&&xy_trace<=config_.flow_xy_covariance_limit;
  const bool within_spawn_window=startup_sim_us_!=0&&state_.sim_time_us>=startup_sim_us_&&state_.sim_time_us-startup_sim_us_<=config_.spawn_prior_timeout_us;
  const bool has_xy_measurement=last_local_tof_us_||last_absolute_us_||last_peer_xy_us_||last_flow_us_;
  e.xy_peer_valid=!e.xy_structural_valid&&fresh(last_peer_xy_us_,config_.peer_timeout_us*2)&&xy_trace<=config_.spawn_prior_xy_variance_limit;
  // 出生点先验门限仍保留；没有任何有效 XY 来源时，初始位姿不能触发 SPAWN_PRIOR。
  e.spawn_prior_valid=!e.xy_structural_valid&&!e.xy_peer_valid&&!flow&&has_xy_measurement&&within_spawn_window&&xy_trace<=config_.spawn_prior_xy_variance_limit;
  e.flow_valid=flow;
  e.xy_valid=e.xy_structural_valid||e.xy_peer_valid||e.spawn_prior_valid||flow;
  e.z_valid=fresh(last_height_us_,config_.height_timeout_us)||fresh(last_baro_us_,500000);
  e.height_assisted=fresh(last_height_us_,config_.height_timeout_us);
  e.vz_derived_from_height=e.height_assisted&&state_.vz_derived_from_height;
  // 该值只描述协方差规模，保留原始公式用于诊断，不能解释为概率或校准置信度。
  e.position_confidence=std::exp(-0.5*xy_trace);
  e.xy_uncertainty_m=std::sqrt(xy_trace);
  e.localization_mode=e.xy_structural_valid?"STRUCTURAL":(e.xy_peer_valid?"PEER_REFERENCE":(flow?"FLOW_DRIFT":(e.spawn_prior_valid?"SPAWN_PRIOR":(has_xy_measurement?"INVALID":"INITIALIZING"))));
  return e;
}
/**
 * @brief 将七维状态从当前时间推进到目标时间。
 * @note 正常间隔按不超过 20 ms 的小步长积分，并用 FPFᵀ+Q 传播协方差；超过 2 s
 *       时放弃旧加速度，扩大不确定度后跳到目标时间，避免长时间失联期间产生虚假精度。
 */
void Estimator::advanceTo(uint64_t now) {
  // 预测步骤：p+=v*dt+0.5*a*dt²，v+=a*dt；F 的位置-速度块为 dt。
  // Q 每轴 [位置,速度] 块为 0.25*[[dt³/3,dt²/2],[dt²/2,dt]]，航向项为 0.003*dt。
  // 按不超过 20 ms 的小步长传播 P=FPFᵀ+Q；超过 2 s 时放弃旧加速度并扩大不确定度。
  if(!now||now<=state_.sim_time_us) return;
  if(state_.sim_time_us==0){state_.sim_time_us=now;if(!startup_sim_us_)startup_sim_us_=now;return;}
  double remain=(now-state_.sim_time_us)*1e-6;
  if(remain>2.0){ state_.covariance.diagonal().array()+=remain; state_.sim_time_us=now; held_acceleration_.setZero(); return; }
  while(remain>1e-8){ double dt=std::min(remain,0.02); Eigen::Vector3d a=fresh(last_imu_us_,100000)?held_acceleration_:Eigen::Vector3d::Zero(); Eigen::Vector3d p(state_.pose.x_m,state_.pose.y_m,state_.pose.z_m); p+=state_.velocity_enu_mps*dt+.5*a*dt*dt; state_.pose.x_m=p.x();state_.pose.y_m=p.y();state_.pose.z_m=p.z();state_.velocity_enu_mps+=a*dt; Eigen::Matrix<double,7,7> f=Eigen::Matrix<double,7,7>::Identity();f(0,4)=f(1,5)=f(2,6)=dt;Eigen::Matrix<double,7,7> q=Eigen::Matrix<double,7,7>::Zero();for(int i=0;i<3;i++){int v=4+i;q(i,i)=.25*dt*dt*dt/3;q(v,v)=.25*dt;q(i,v)=q(v,i)=.25*dt*dt/2;}q(3,3)=.003*dt;state_.covariance=f*state_.covariance*f.transpose()+q;remain-=dt;state_.sim_time_us+=static_cast<uint64_t>(std::llround(dt*1e6));} state_.sim_time_us=now;
}
double Estimator::yawAfterImu(const ImuSample& s) const {
  if(!s.sim_time_us||s.sim_time_us<=last_imu_us_||!s.acceleration_enu_mps2.allFinite()||
      !attitude_count_||!s.attitude_valid||!std::isfinite(s.yaw_enu_rad)||
      !std::isfinite(s.roll_flu_rad)||!std::isfinite(s.pitch_flu_rad))return state_.pose.yaw_rad;
  return wrapAngle(state_.pose.yaw_rad+wrapAngle(s.yaw_enu_rad-last_imu_yaw_));
}
/**
 * @brief 接收 IMU 姿态和 ENU 加速度，并交给下一次时间推进使用。
 * @note 航向只累加相邻 IMU 样本的包角差；加速度在两次样本之间零阶保持，超过 100 ms
 *       未更新则由 advanceTo 使用零加速度，防止陈旧加速度持续推演速度。
 */
void Estimator::predict(const ImuSample& s) {
  // 航向只累加相邻 IMU 样本的包角差；加速度零阶保持，超过 100 ms 未更新则不再使用。
  if(!s.sim_time_us||s.sim_time_us<=last_imu_us_||!s.acceleration_enu_mps2.allFinite()) return;
  const double yaw_after_imu=yawAfterImu(s);
  advanceTo(s.sim_time_us);
  if(s.attitude_valid && std::isfinite(s.yaw_enu_rad) && std::isfinite(s.roll_flu_rad) && std::isfinite(s.pitch_flu_rad)){
    state_.pose.yaw_rad=yaw_after_imu;
    last_imu_yaw_=s.yaw_enu_rad;state_.pose.roll_rad=s.roll_flu_rad;state_.pose.pitch_rad=s.pitch_flu_rad;
    const auto orientation=Eigen::Quaterniond(Eigen::AngleAxisd(s.yaw_enu_rad,Eigen::Vector3d::UnitZ())*
        Eigen::AngleAxisd(s.pitch_flu_rad,Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(s.roll_flu_rad,Eigen::Vector3d::UnitX()));
    const std::size_t slot=(attitude_begin_+attitude_count_)%attitude_history_.size();
    attitude_history_[slot]={s.sim_time_us,orientation};
    if(attitude_count_<attitude_history_.size())++attitude_count_;
    else attitude_begin_=(attitude_begin_+1)%attitude_history_.size();
  }
  held_acceleration_=s.acceleration_valid?s.acceleration_enu_mps2:Eigen::Vector3d::Zero();last_imu_us_=s.sim_time_us;
}
void Estimator::resetInertialVelocityWhileLanded(uint64_t now_us) {
  if(now_us && now_us<state_.sim_time_us) return;
  state_.velocity_enu_mps.setZero();
  held_acceleration_.setZero();
  if(now_us) advanceTo(now_us);
}
FusionReport Estimator::fuseMapContact(uint64_t now,const Eigen::Vector3d& normal,double offset) {
  FusionReport report;
  if(now<state_.sim_time_us||!normal.allFinite()||!std::isfinite(offset)||std::abs(normal.norm()-1)>.001){report.gated=true;return report;}
  advanceTo(now);
  Eigen::Matrix<double,2,7> h=Eigen::Matrix<double,2,7>::Zero();
  h.block<1,3>(0,0)=normal.transpose();h.block<1,3>(1,4)=normal.transpose();
  const Eigen::Vector3d p(state_.pose.x_m,state_.pose.y_m,state_.pose.z_m);
  Eigen::Vector2d residual(offset-normal.dot(p),-normal.dot(state_.velocity_enu_mps));
  Eigen::Matrix2d noise=Eigen::Matrix2d::Zero();noise(0,0)=.0004;noise(1,1)=.0025;
  update(h,residual,noise);held_acceleration_.setZero();
  report.accepted=true;report.reason="known_surface_contact";return report;
}
Eigen::Vector3d Estimator::tofRayBody(const TofMount& m,int row,int col){double u=(col-3.5)/4.0*std::tan(m.horizontal_fov_rad/2),v=(row-3.5)/4.0*std::tan(m.vertical_fov_rad/2);return rotation(m.roll_rad,m.pitch_down_rad,m.yaw_rad)*Eigen::Vector3d(1,-u,-v);}
bool Estimator::lineObservation(const TofFrame& f,int cell,const Pose3& pose,const Eigen::Matrix3d& r,LineObservation& o) const {
  // 轴向深度经安装外参和机体姿态转成世界端点，匹配有限墙段并检查高度范围。
  // 点到线残差 n·(endpoint-start) 的导数为 [nx,ny,n·(-off_y,off_x)]。
  if(f.sensor_index<0||f.sensor_index>=3||!f.valid[cell])return false;
  const auto&m=config_.tof_mounts[f.sensor_index];double d=f.distance_m[cell];
  if(!std::isfinite(d)||d<m.min_range_m||d>m.max_range_m)return false;
  Eigen::Vector3d off=r*(m.translation_body_m+d*tof_rays_[f.sensor_index][cell]);
  const Eigen::Vector3d ep=Eigen::Vector3d(pose.x_m,pose.y_m,pose.z_m)+off;
  if(ep.z()<config_.floor_z_m+.12)return false;
  double best=.6;bool found=false;
  for(const auto&w:config_.map_lines){
    const Eigen::Vector2d line=w.end_m-w.start_m;const double len=line.norm();
    if(len<1e-5||ep.z()<w.z_min_m||ep.z()>w.z_max_m)continue;
    const Eigen::Vector2d t=line/len,n(-t.y(),t.x());
    const double along=(ep.head<2>()-w.start_m).dot(t),res=n.dot(ep.head<2>()-w.start_m);
    if(along>=0&&along<=len&&std::abs(res)<best){best=std::abs(res);found=true;o.residual=res;o.jacobian<<n.x(),n.y(),n.dot(Eigen::Vector2d(-off.y(),off.x()));}
  }
  return found;
}
bool Estimator::terrainObservation(const TofFrame& f,int cell,const Pose3& pose,const Eigen::Matrix3d& r,LineObservation& o) const {
  if(!config_.terrain || f.sensor_index<0 || f.sensor_index>=3 || !f.valid[cell]) return false;
  const auto&m=config_.tof_mounts[f.sensor_index]; const double d=f.distance_m[cell];
  if(!std::isfinite(d)||d<m.min_range_m||d>m.max_range_m) return false;
  // 1. 保留未归一化射线，使 raycast 的参数仍与传感器轴向深度 d 一致。
  const auto& body_ray=tof_rays_[f.sensor_index][cell];
  const Eigen::Vector3d off=r*m.translation_body_m;
  const Eigen::Vector3d ray=r*body_ray, origin=Eigen::Vector3d(pose.x_m,pose.y_m,pose.z_m)+off;
  // 2. 地图射线找最近面，先筛轴向深度差，再用面法向得到点到面残差。
  TerrainMap::Hit hit; if(!config_.terrain->raycast(origin,ray,d+config_.terrain_range_gate_m,hit)) return false;
  if(std::abs(d-hit.depth_m)>config_.terrain_range_gate_m) return false;
  const Eigen::Vector3d endpoint=origin+d*ray; o.residual=hit.normal.dot(endpoint-hit.point);
  // 3. 航向绕世界 Z 轴旋转，端点导数为 (-lever_y,lever_x,0)，与法向点乘得到 yaw 列。
  const Eigen::Vector3d lever=off+d*ray, yaw_derivative(-lever.y(),lever.x(),0.0);
  o.jacobian<<hit.normal.x(),hit.normal.y(),hit.normal.dot(yaw_derivative); o.normal=hit.normal;
  return std::isfinite(o.residual) && std::abs(o.residual)<=config_.terrain_plane_gate_m && o.jacobian.allFinite();
}
void Estimator::update(const Eigen::MatrixXd&h,const Eigen::VectorXd&r,const Eigen::MatrixXd&rc){
  if (h.cols()!=7 || h.rows()==0 || r.size()!=h.rows() || rc.rows()!=h.rows() || rc.cols()!=h.rows() ||
      !h.allFinite() || !r.allFinite() || !rc.allFinite()) return;
  // 创新为测量减预测：S=HPHᵀ+R，K=PHᵀS⁻¹，δx=K*创新。LDLT 用于求解 S。
  // H 未直接观测的状态也可能通过 P 的相关项被修正，例如速度观测可修正相关位置。
  const Eigen::Matrix<double,7,7> p=state_.covariance;
  const Eigen::MatrixXd s=h*p*h.transpose()+rc;
  if (!s.allFinite()) return;
  const Eigen::MatrixXd k=p*h.transpose()*s.ldlt().solve(Eigen::MatrixXd::Identity(s.rows(),s.cols()));
  const Eigen::VectorXd d=k*r;
  state_.pose.x_m+=d(0);state_.pose.y_m+=d(1);state_.pose.z_m+=d(2);state_.pose.yaw_rad=wrapAngle(state_.pose.yaw_rad+d(3));state_.velocity_enu_mps+=d.segment<3>(4);
  // Joseph 形式 P=(I-KH)P(I-KH)ᵀ+KRKᵀ 减少数值误差，随后消除舍入造成的不对称。
  const Eigen::Matrix<double,7,7> a=Eigen::Matrix<double,7,7>::Identity()-k*h;
  state_.covariance=a*p*a.transpose()+k*rc*k.transpose();state_.covariance=.5*(state_.covariance+state_.covariance.transpose()).eval();
}
std::optional<Eigen::Matrix3d> Estimator::attitudeDelta(uint64_t stamp) const {
  if(!attitude_count_)return Eigen::Matrix3d::Identity();
  const auto at=[&](std::size_t i)->const StampedAttitude&{
    return attitude_history_[(attitude_begin_+i)%attitude_history_.size()];
  };
  const auto& latest=at(attitude_count_-1);
  if(stamp>=latest.stamp)return Eigen::Matrix3d::Identity();
  if(stamp<at(0).stamp)return std::nullopt;
  std::size_t low=0,high=attitude_count_-1;
  while(low<high){const auto middle=(low+high)/2;if(at(middle).stamp<stamp)low=middle+1;else high=middle;}
  const auto& right=at(low);
  Eigen::Quaterniond exposure=right.orientation;
  if(stamp!=right.stamp){
    const auto& left=at(low-1);
    if(right.stamp-left.stamp>100000)return std::nullopt;
    const double fraction=static_cast<double>(stamp-left.stamp)/(right.stamp-left.stamp);
    exposure=left.orientation.slerp(fraction,right.orientation);
  }
  if((exposure.coeffs().array()==latest.orientation.coeffs().array()).all())return Eigen::Matrix3d::Identity();
  return (latest.orientation.conjugate()*exposure).toRotationMatrix();
}

FusionReport Estimator::fuseTof(const TofFrame& frame) {
  FusionReport report;
  if(frame.sensor_index<0 || frame.sensor_index>=3 || !frame.sim_time_us ||
     frame.sim_time_us<=last_tof_us_[frame.sensor_index]) {
    report.stale=true;report.reason="tof_invalid_or_duplicate";return report;
  }
  advanceTo(frame.sim_time_us);
  if(!fresh(frame.sim_time_us,config_.tof_timeout_us)) {
    report.stale=true;report.reason="tof_stale";return report;
  }
  last_tof_us_[frame.sensor_index]=frame.sim_time_us;tof_cache_[frame.sensor_index]=frame;
  std::array<std::optional<Eigen::Matrix3d>,3> attitude_deltas;
  std::array<bool,3> rotated{};bool missing_attitude=false,exposure_ready=false;
  for(const auto& cached:tof_cache_){
    if(cached.sensor_index<0||!fresh(cached.sim_time_us,config_.tof_timeout_us)||
        cached.sim_time_us<=assimilated_tof_us_[cached.sensor_index])continue;
    const auto index=cached.sensor_index;
    attitude_deltas[index]=attitudeDelta(cached.sim_time_us);
    if(!attitude_deltas[index]){missing_attitude=true;continue;}
    exposure_ready=true;rotated[index]=!attitude_deltas[index]->isIdentity(0);
  }
  // 1. 缓存三个方向的新鲜帧，以预测位姿为起点做最多五轮鲁棒点到线/点到面匹配。
  const Pose3 prior=state_.pose;
  Pose3 working=prior;
  Eigen::Matrix3d info=Eigen::Matrix3d::Zero(), modes=Eigen::Matrix3d::Identity();
  Eigen::Vector3d values=Eigen::Vector3d::Zero();
  int count=0,rank=0;
  double sum_squared=0;
  for(int iteration=0;iteration<5;++iteration) {
    info.setZero(); Eigen::Vector3d gradient=Eigen::Vector3d::Zero();
    count=0;sum_squared=0; int independent_normals=0; Eigen::Vector3d first_normal=Eigen::Vector3d::Zero();
    const auto current_rotation=rotation(working.roll_rad,working.pitch_rad,working.yaw_rad);
    for(const auto& cached:tof_cache_) {
      if(cached.sensor_index<0 || !attitude_deltas[cached.sensor_index])continue;
      // 各传感器帧可能在不同时间采样；按当前速度回退到曝光时刻，避免把运动误当成地图误差。
      const double age=(state_.sim_time_us-cached.sim_time_us)*1e-6;
      Pose3 exposure_pose=working;
      exposure_pose.x_m-=state_.velocity_enu_mps.x()*age;
      exposure_pose.y_m-=state_.velocity_enu_mps.y()*age;
      exposure_pose.z_m-=state_.velocity_enu_mps.z()*age;
      Eigen::Matrix3d exposure_rotation=current_rotation;
      if(rotated[cached.sensor_index])exposure_rotation*=*attitude_deltas[cached.sensor_index];
      for(int cell=0;cell<64;++cell) {
        LineObservation observation;
        const bool observed=config_.terrain ? terrainObservation(cached,cell,exposure_pose,exposure_rotation,observation) : lineObservation(cached,cell,exposure_pose,exposure_rotation,observation);
        if(!observed)continue;
        if(config_.terrain) {
          if(first_normal.squaredNorm()<0.5) first_normal=observation.normal;
          else if(independent_normals<2 && first_normal.cross(observation.normal).norm()>0.20) independent_normals=2;
        }
        // 信息矩阵 I=ΣwJᵀJ、梯度 g=ΣwJᵀr；这里只优化 [x,y,yaw]，高度由独立观测提供。
        const double weight=huber(observation.residual,config_.tof_sigma_m)/std::pow(config_.tof_sigma_m,2);
        info+=weight*observation.jacobian.transpose()*observation.jacobian;
        gradient+=weight*observation.jacobian.transpose()*observation.residual;
        sum_squared+=observation.residual*observation.residual;++count;
      }
    }
    if(count<(config_.terrain?8:2)){
      report.stale=missing_attitude&&!exposure_ready;report.degenerate=!report.stale;
      report.reason=report.stale?"tof_attitude_history_missing":"too_few_structural_returns";return report;
    }
    // 相邻像素共享系统误差，按每 8 个像素最多贡献一个独立样本，避免信息量虚高。
    const double correlated=std::max(1.0,count/8.0);
    info/=correlated;gradient/=correlated;
    // 2. I 的特征向量给出约束方向，特征值表示信息强度。单墙通常缺沿墙方向信息；
    //    只对超过阈值的方向求 δ=-I⁺g，弱方向保留先验，避免退化矩阵强行求逆。
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(info);
    if(solver.info()!=Eigen::Success){report.reason="structure_eigen_failure";return report;}
    values=solver.eigenvalues();modes=solver.eigenvectors();
    const double threshold=std::max(0.1,values(2)*(config_.terrain?1e-3:1e-4));
    Eigen::Vector3d step=Eigen::Vector3d::Zero();rank=0;
    for(int axis=0;axis<3;++axis)if(values(axis)>threshold) {
      ++rank;step-=modes.col(axis)*(modes.col(axis).dot(gradient)/values(axis));
    }
    if(!rank){report.degenerate=true;report.reason="structure_unobservable";return report;}
    if(config_.terrain && independent_normals<2){report.degenerate=true;report.gated=true;report.reason="terrain_normals_not_independent";return report;}
    step*=std::min(1.0,0.2/std::max(step.norm(),1e-9));
    working.x_m+=step.x();working.y_m+=step.y();working.yaw_rad=wrapAngle(working.yaw_rad+step.z());
    if(step.norm()<1e-5)break;
  }
  // 3. 残差 RMS 和位姿信任域共同检查匹配可信度，避免错误表面关联引起大幅跳变。
  const Eigen::Vector3d shift(working.x_m-prior.x_m,working.y_m-prior.y_m,wrapAngle(working.yaw_rad-prior.yaw_rad));
  if (!std::isfinite(sum_squared) || count < 1 || std::sqrt(sum_squared/count) > (config_.terrain ? config_.terrain_plane_gate_m : 0.20)) {
    report.gated=true; report.degenerate=true; report.reason="tof_residual_gate"; return report;
  }
  if(config_.terrain && (shift.head<2>().norm()>config_.terrain_max_translation_update_m || std::abs(shift.z())>config_.terrain_max_yaw_update_rad)) {
    report.gated=true; report.degenerate=true; report.reason="terrain_update_trust_region"; return report;
  }
  if(structural_information_us_ && frame.sim_time_us>structural_information_us_) {
    const double dt=(frame.sim_time_us-structural_information_us_)*1e-6;
    structural_information_*=std::exp(-std::min(dt,10.0)/2.0);
  }
  structural_information_+=info;
  structural_information_us_=frame.sim_time_us;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> cumulative_solver(structural_information_);
  int cumulative_rank=0;
  if(cumulative_solver.info()==Eigen::Success) {
    const auto cumulative_values=cumulative_solver.eigenvalues();
    const double cumulative_threshold=std::max(0.1,cumulative_values(2)*1e-4);
    for(int axis=0;axis<3;++axis) if(cumulative_values(axis)>cumulative_threshold) ++cumulative_rank;
  }
  // 4. 将位移投影到可观测特征方向，作为 EKF 低维伪观测，噪声约为 1/特征值。
  //    rank<3 时仍能更新部分方向，因此 accepted 与 degenerate 可以同时为真。
  Eigen::MatrixXd h=Eigen::MatrixXd::Zero(rank,7),noise=Eigen::MatrixXd::Zero(rank,rank);
  Eigen::VectorXd residual(rank);int row=0;
  const double threshold=std::max(0.1,values(2)*(config_.terrain?1e-3:1e-4));
  for(int axis=0;axis<3;++axis)if(values(axis)>threshold) {
    h(row,0)=modes(0,axis);h(row,1)=modes(1,axis);h(row,3)=modes(2,axis);
    residual(row)=modes.col(axis).dot(shift);noise(row,row)=std::max(1.0/values(axis),1e-4);++row;
  }
  update(h,residual,noise);state_.degenerate_geometry=rank<3;
  // 每帧深度只计权一次；若本次因几何退化未能单独定位，仍保留缓存等待互补方向。
  for(const auto& cached:tof_cache_)
    if(cached.sensor_index>=0 && attitude_deltas[cached.sensor_index])
      assimilated_tof_us_[cached.sensor_index]=cached.sim_time_us;
  // 衰减累计信息只用于诊断，不能把当前不可观测的 XY 方向伪装成全队的新绝对参考。
  // 5. Schur 补 Ixy-Ixy,yaw*Iyaw,xy/Iyaw,yaw 消去航向耦合，判断未知航向下 XY 是否可观测。
  //    最小特征值过门限才创建本机绝对参考；累计秩不参与此决定。
  Eigen::Matrix2d xy_information=info.block<2,2>(0,0);
  if (info(2,2)>1e-9) xy_information-=info.block<2,1>(0,2)*info.block<1,2>(2,0)/info(2,2);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> xy_solver(xy_information);
  const bool xy_observable=xy_solver.info()==Eigen::Success && xy_solver.eigenvalues()(0)>threshold;
  if(xy_observable) {
    last_absolute_us_=frame.sim_time_us;
    state_.reference_origin=config_.vehicle_id; state_.reference_seq=++next_reference_seq_;
    state_.reference_time_us=frame.sim_time_us; state_.reference_hops=0; state_.reference_path={config_.vehicle_id};
  }
  local_tof_rank_=rank; last_local_tof_us_=frame.sim_time_us;
  report.accepted=true;report.degenerate=rank<3;report.inliers=count;
  report.innovation_norm=std::sqrt(sum_squared/count);report.reason=(config_.terrain?"terrain_point_to_plane_rank_":"point_to_line_rank_")+std::to_string(rank)+"_cumulative_"+std::to_string(cumulative_rank);
  return report;
}

FusionReport Estimator::fuseOpticalFlow(const FlowSample& sample) {
  FusionReport report;
  if (!sample.sim_time_us || sample.sim_time_us <= last_flow_us_ ||
      !std::isfinite(sample.vx_body_mps) || !std::isfinite(sample.vy_body_mps) ||
      !std::isfinite(sample.quality) || !std::isfinite(sample.variance_m2s2) ||
      sample.quality < 0.0 || sample.quality > 1.0 || sample.variance_m2s2 <= 0.0) {
    report.stale=true; report.reason="flow_invalid_or_duplicate"; return report;
  }
  advanceTo(sample.sim_time_us);
  if (sample.quality < 0.5 || !fresh(sample.sim_time_us, config_.flow_timeout_us)) {
    report.gated=true; report.reason="flow_quality_gate"; return report;
  }
  const auto delta=attitudeDelta(sample.sim_time_us);
  if (!delta) {
    report.stale=true; report.reason="flow_attitude_history_missing"; return report;
  }
  const Eigen::Matrix3d r=rotation(state_.pose.roll_rad,state_.pose.pitch_rad,state_.pose.yaw_rad)*(*delta);
  const Eigen::Vector2d measured(sample.vx_body_mps,sample.vy_body_mps);
  Eigen::Matrix<double,2,7> h=Eigen::Matrix<double,2,7>::Zero();
  // v_body_xy=(Rᵀ*v_enu).head<2>()；包含 roll/pitch 的旋转让 vz 也能影响机体 XY。
  // yaw 是滤波状态；其导数为 Rᵀ*(vy,-vx,0)，保留航向与世界速度的不确定度耦合。
  h.block<2,3>(0,4)=r.transpose().topRows<2>();
  h.block<2,1>(0,3)=(r.transpose()*Eigen::Vector3d(state_.velocity_enu_mps.y(),-state_.velocity_enu_mps.x(),0)).head<2>();
  const Eigen::Vector2d residual=measured-h.block<2,3>(0,4)*state_.velocity_enu_mps;
  // 较低质量增大方差、减小权重；光流提供速度约束，位置仍随积分漂移。
  Eigen::Matrix2d noise=Eigen::Matrix2d::Identity()*std::max(sample.variance_m2s2,1e-6)/sample.quality;
  update(h,residual,noise);
  last_flow_us_=sample.sim_time_us; flow_valid_=true; state_.flow_valid=true;
  report.accepted=true; report.innovation_norm=residual.norm(); report.reason="optical_flow_velocity";
  return report;
}

FusionReport Estimator::fuseStaticLandmark(uint64_t stamp, const Eigen::Vector3d& landmark,
    const Eigen::Vector3d& relative, double variance) {
  FusionReport report;
  if(!stamp||stamp<=last_landmark_us_||!landmark.allFinite()||!relative.allFinite()||
      !std::isfinite(variance)||variance<=0||relative.norm()<.1||relative.norm()>8) {
    report.stale=true;report.reason="landmark_invalid_or_duplicate";return report;
  }
  advanceTo(stamp);
  if(!fresh(stamp,150000)){report.stale=true;report.reason="landmark_stale";return report;}
  const auto delta=attitudeDelta(stamp);
  if(!delta){report.stale=true;report.reason="landmark_attitude_history_missing";return report;}
  last_landmark_us_=stamp;
  const double age=(state_.sim_time_us-stamp)*1e-6;
  const Eigen::Matrix3d r=rotation(state_.pose.roll_rad,state_.pose.pitch_rad,state_.pose.yaw_rad)*(*delta);
  const Eigen::Vector3d displacement=landmark-Eigen::Vector3d(state_.pose.x_m,state_.pose.y_m,state_.pose.z_m)
      +state_.velocity_enu_mps*age;
  const Eigen::Vector3d residual=relative-r.transpose()*displacement;
  Eigen::Matrix<double,3,7> h=Eigen::Matrix<double,3,7>::Zero();
  h.block<3,3>(0,0)=-r.transpose();
  h.block<3,3>(0,4)=age*r.transpose();
  h.col(3)=r.transpose()*Eigen::Vector3d(displacement.y(),-displacement.x(),0);
  // 速度回退期间的额外位置噪声取 σ=1.5*age² m，计入未建模加速度的影响。
  const Eigen::Matrix3d noise=Eigen::Matrix3d::Identity()*(variance+2.25*std::pow(age,4));
  const Eigen::Matrix3d innovation=h*state_.covariance*h.transpose()+noise;
  const double mahalanobis=residual.dot(innovation.ldlt().solve(residual));
  if(!std::isfinite(mahalanobis)||mahalanobis>11.35||residual.norm()>1) {
    report.gated=true;report.reason="landmark_innovation_gated";return report;
  }
  update(h,residual,noise);
  last_absolute_us_=stamp;
  state_.reference_origin=config_.vehicle_id;state_.reference_seq=++next_reference_seq_;
  state_.reference_time_us=stamp;state_.reference_hops=0;state_.reference_path={config_.vehicle_id};
  report.accepted=true;report.innovation_norm=residual.norm();report.reason="static_visual_landmark";
  return report;
}

FusionReport Estimator::fuseAnchorRange(const AnchorRangeSample& sample) {
  FusionReport report;
  const auto anchor=config_.uwb_anchors.find(sample.anchor_id);
  if (anchor==config_.uwb_anchors.end() || !sample.valid || !sample.sim_time_us ||
      !std::isfinite(sample.range_m)||!std::isfinite(sample.variance_m2)||!std::isfinite(sample.quality)||
      sample.sim_time_us<=last_anchor_us_[sample.anchor_id] || sample.range_m<=0.0 ||
      sample.variance_m2<=0.0 || sample.quality<=0.0 || sample.quality>1.0 ||
      !anchor->second.allFinite()) { report.stale=true; report.reason="anchor_invalid_or_unknown"; return report; }
  advanceTo(sample.sim_time_us);
  if (!fresh(sample.sim_time_us,config_.anchor_timeout_us)) {
    report.stale=true; report.reason="anchor_stale"; return report;
  }
  const Eigen::Vector3d p(state_.pose.x_m,state_.pose.y_m,state_.pose.z_m);
  // 固定基站距离 ||anchor-p|| 的位置导数为 -(anchor-p)ᵀ/距离，只约束径向。
  const Eigen::Vector3d delta=anchor->second-p; const double predicted=delta.norm();
  if (!std::isfinite(predicted) || predicted<0.1) { report.gated=true; report.reason="anchor_geometry"; return report; }
  Eigen::Matrix<double,1,7> h=Eigen::Matrix<double,1,7>::Zero(); h.block<1,3>(0,0)=-delta.transpose()/predicted;
  Eigen::VectorXd residual(1); residual(0)=sample.range_m-predicted;
  Eigen::MatrixXd noise(1,1); noise(0,0)=std::max(sample.variance_m2,1e-6)/sample.quality;
  const double innovation=(h*state_.covariance*h.transpose()+noise)(0,0);
  if (!std::isfinite(innovation) || residual(0)*residual(0)/innovation>13.82 || std::abs(residual(0))>1.5) { report.gated=true; report.reason="anchor_innovation_gated"; return report; }
  update(h,residual,noise); last_anchor_us_[sample.anchor_id]=sample.sim_time_us;
  accepted_anchors_[sample.anchor_id]={anchor->second,sample.sim_time_us,sample.quality};
  // 收集新鲜基站的水平单位方向外积，检查两个 XY 方向均有信息；至少两个基站、
  // 最小特征值>0.25 且全部质量≥0.5 时，才标记本机绝对 XY 参考。
  Eigen::Matrix2d geometry=Eigen::Matrix2d::Zero();
  bool all_high_quality=true; size_t fresh_count=0;
  for (const auto& item:accepted_anchors_) if(fresh(item.second.sim_time_us,config_.anchor_timeout_us)) { ++fresh_count; all_high_quality=all_high_quality&&item.second.quality>=0.5; const Eigen::Vector2d d=item.second.position.head<2>()-p.head<2>(); const double n=d.norm(); if(n>0.1) geometry+=d*d.transpose()/(n*n); }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(geometry);
  const bool full_xy=solver.info()==Eigen::Success && solver.eigenvalues()(0)>0.25 && fresh_count>=2 && all_high_quality && sample.quality>=0.5;
  if (full_xy) { last_absolute_us_=sample.sim_time_us; state_.reference_origin=config_.vehicle_id; state_.reference_seq=++next_reference_seq_; state_.reference_time_us=sample.sim_time_us; state_.reference_hops=0; state_.reference_path={config_.vehicle_id}; }
  report.accepted=true; report.innovation_norm=std::abs(residual(0)); report.reason=full_xy?"uwb_anchor_absolute":"uwb_anchor_range"; return report;
}

FusionReport Estimator::fuseUwb(const UwbPeerSample& sample,const PeerEstimate& peer) {
  FusionReport report;
  // 1. 检查源观测时效、路径环路/重复节点、已使用的源序号，阻止同一参考反复计权。
  const bool has_reference=peer.reference_origin>=0 && peer.reference_seq>0 && peer.reference_time_us>0 && peer.reference_hops>=0;
  bool loop=false; bool duplicate_path=false;
  std::unordered_set<int> path_ids;
  for(int id:peer.reference_path) { if(id==config_.vehicle_id) loop=true; if(!path_ids.insert(id).second) duplicate_path=true; }
  if (has_reference && (peer.reference_time_us>peer.sim_time_us || peer.sim_time_us-peer.reference_time_us>config_.peer_timeout_us)) { report.stale=true; report.reason="reference_stale"; return report; }
  if (loop || duplicate_path || (has_reference && seen_reference_seq_[peer.reference_origin]>=peer.reference_seq)) { report.gated=true; report.reason=loop?"reference_path_loop":(duplicate_path?"reference_path_duplicate":"reference_already_seen"); return report; }
  // 当前地图解已完整约束时优先保留独立定位，队友参考主要用于补缺，避免共同误差扩散。
  if(config_.terrain && estimate().xy_structural_valid && local_tof_rank_>=3) {
    report.gated=true;report.reason="independent_map_preferred";return report;
  }
  if(!peer.xy_structural_valid && !has_reference){report.gated=true;report.reason="peer_without_reference";return report;}
  if(!sample.valid || peer.source!="ESTIMATE" || peer.vehicle_id==config_.vehicle_id ||
     !std::isfinite(sample.quality)||!std::isfinite(peer.pose.x_m)||!std::isfinite(peer.pose.y_m)||!std::isfinite(peer.pose.z_m)||
     sample.peer_id!=peer.vehicle_id || sample.peer_id<0 || sample.quality<=0.0 || sample.quality>1.0 ||
     sample.sim_time_us<=last_uwb_us_[sample.peer_id] || !peer.pose_covariance.allFinite() ||
     !peer.velocity_enu_mps.allFinite() || !std::isfinite(sample.range_m) || sample.range_m<=0.0 ||
     !std::isfinite(sample.bearing_rad)) {
    report.stale=true;report.reason="uwb_or_peer_invalid";return report;
  }
  advanceTo(sample.sim_time_us);
  if(!fresh(sample.sim_time_us,config_.peer_timeout_us) || !fresh(peer.sim_time_us,config_.peer_timeout_us)) {
    report.stale=true;report.reason="uwb_or_peer_stale";return report;
  }
  last_uwb_us_[sample.peer_id]=sample.sim_time_us;
  // 2. 以队友速度外推到当前时间；距离用三维位移，方位用 FLU 中 atan2(y,x)。
  const double peer_age=(state_.sim_time_us-peer.sim_time_us)*1e-6;
  const Eigen::Vector3d delta=Eigen::Vector3d(peer.pose.x_m-state_.pose.x_m,
      peer.pose.y_m-state_.pose.y_m,peer.pose.z_m-state_.pose.z_m)+peer.velocity_enu_mps*peer_age;
  const double range=delta.norm();
  const Eigen::Matrix3d body_rotation=rotation(state_.pose.roll_rad,state_.pose.pitch_rad,state_.pose.yaw_rad);
  const Eigen::Vector3d body=body_rotation.transpose()*delta;
  if(range<.1 || body.head<2>().squaredNorm()<.01) {
    report.gated=true;report.reason="uwb_geometry";return report;
  }
  const double bearing=std::atan2(body.y(),body.x());
  const Eigen::Vector2d residual(sample.range_m-range,wrapAngle(sample.bearing_rad-bearing));
  // 3. H 对自身位置/航向求导；j 对队友位置求导，将队友协方差传到距离和方位噪声。
  Eigen::Matrix<double,2,7> h=Eigen::Matrix<double,2,7>::Zero();
  h.block<1,3>(0,0)=-delta.transpose()/range;
  const Eigen::RowVector3d d_body(-body.y()/body.head<2>().squaredNorm(),body.x()/body.head<2>().squaredNorm(),0);
  const Eigen::RowVector3d d_world=d_body*body_rotation.transpose();
  h.block<1,3>(1,0)=-d_world;
  h(1,3)=d_body.dot(body_rotation.transpose()*Eigen::Vector3d(delta.y(),-delta.x(),0));
  Eigen::Matrix<double,2,3> j;
  j.row(0)=delta.transpose()/range;j.row(1)=d_world;
  // 队友估计可能已经共享过同一参考源；放大噪声以降低重复信息权重，不把它宣称为新基站。
  Eigen::Matrix2d noise=2*j*peer.pose_covariance*j.transpose();
  noise(0,0)+=(std::pow(config_.uwb_range_sigma_m,2)+.25*peer_age*peer_age)/sample.quality;
  noise(1,1)+=std::pow(config_.uwb_bearing_sigma_rad,2)/sample.quality;
  const Eigen::Matrix2d innovation=h*state_.covariance*h.transpose()+noise;
  // 马氏距离联合检查两种单位的创新，同时保留距离 1 m、角度 0.65 rad 的独立门限。
  const double mahalanobis=residual.dot(innovation.ldlt().solve(residual));
  if(!std::isfinite(mahalanobis)||mahalanobis>13.82||std::abs(residual(0))>1||std::abs(residual(1))>.65) {
    report.gated=true;report.reason="uwb_innovation_gated";return report;
  }
  update(h,residual,noise);
  state_.covariance(0,0)=std::max(state_.covariance(0,0),config_.uwb_xy_covariance_floor_m2);
  state_.covariance(1,1)=std::max(state_.covariance(1,1),config_.uwb_xy_covariance_floor_m2);
  // 4. 传播原始 origin/seq/time，增加一跳并追加本机；本次接收时间用于本地队友约束有效期。
  if (has_reference && (state_.reference_origin!=peer.reference_origin || peer.reference_time_us>state_.reference_time_us || peer.reference_seq>state_.reference_seq)) {
    state_.reference_origin=peer.reference_origin; state_.reference_seq=peer.reference_seq; state_.reference_time_us=peer.reference_time_us;
    state_.reference_hops=peer.reference_hops+1; state_.reference_path=peer.reference_path; state_.reference_path.push_back(config_.vehicle_id);
    last_peer_xy_us_=sample.sim_time_us;
  } else if (peer.xy_structural_valid) {
    last_peer_xy_us_=sample.sim_time_us;
  }
  if (has_reference) seen_reference_seq_[peer.reference_origin]=std::max(seen_reference_seq_[peer.reference_origin],peer.reference_seq);
  report.accepted=true;report.innovation_norm=residual.norm();
  report.reason="peer_estimate_relative_constraint";return report;
}
// 高度辅助直接替换 z 并清除其交叉协方差，以独立测量方差重建 Pzz；
// 连续高度间隔在 (1 ms,250 ms) 内时差分得到 vz，保持其来源标记。
FusionReport Estimator::fuseHeightAssist(const HeightAssistSample&s){FusionReport q;if(s.source!="SIM_HEIGHT_ASSIST"||!std::isfinite(s.z_world_m)||s.sim_time_us<=last_height_us_){q.stale=true;q.reason="height_invalid_stale_source";return q;}advanceTo(s.sim_time_us);q.innovation_norm=std::abs(s.z_world_m-state_.pose.z_m);state_.pose.z_m=s.z_world_m;state_.covariance.row(2).setZero();state_.covariance.col(2).setZero();state_.covariance(2,2)=std::max(1e-6,config_.height_sigma_m*config_.height_sigma_m);if(previous_height_us_&&s.sim_time_us>previous_height_us_){double dt=(s.sim_time_us-previous_height_us_)*1e-6;if(dt>.001&&dt<.25){state_.velocity_enu_mps.z()=(s.z_world_m-previous_height_m_)/dt;state_.vz_derived_from_height=true;}}previous_height_m_=s.z_world_m;previous_height_us_=last_height_us_=s.sim_time_us;state_.height_assisted=state_.z_valid=true;q.accepted=true;q.reason="SIM_HEIGHT_ASSIST";return q;}
// 首个有效气压绑定当时的估计高度；气压比换算相对高度后，以 z 的标量观测融合。
// 辅助高度仍新鲜时跳过气压更新，避免两类高度来源反复拉动同一状态。
FusionReport Estimator::fuseBaroPressure(const ImuSample&s){FusionReport q;if(!config_.use_baro_fallback||!s.pressure_valid||!std::isfinite(s.absolute_pressure_hpa)||s.absolute_pressure_hpa<100||s.sim_time_us<=last_baro_us_){q.reason="baro_invalid";return q;}advanceTo(s.sim_time_us);if(!baro_reference_hpa_){baro_reference_hpa_=s.absolute_pressure_hpa;baro_reference_z_=state_.pose.z_m;}if(fresh(last_height_us_,config_.height_timeout_us)){q.reason="baro_skipped_height_assist";return q;}double z=baro_reference_z_+44330*(1-std::pow(s.absolute_pressure_hpa/baro_reference_hpa_,.190294957));Eigen::Matrix<double,1,7>h=Eigen::Matrix<double,1,7>::Zero();h(0,2)=1;Eigen::VectorXd r(1);r(0)=z-state_.pose.z_m;Eigen::MatrixXd rc(1,1);rc(0,0)=config_.baro_sigma_m*config_.baro_sigma_m;update(h,r,rc);last_baro_us_=s.sim_time_us;q.accepted=true;q.reason="baro_relative_to_initial_prior";return q;}
}
