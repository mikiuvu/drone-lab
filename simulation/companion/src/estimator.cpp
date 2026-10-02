#include "companion/estimator.hpp"
#include "companion/terrain.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

namespace companion { namespace {
Eigen::Matrix3d rotation(double r,double p,double y) {
  return (Eigen::AngleAxisd(y,Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(p,Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(r,Eigen::Vector3d::UnitX())).toRotationMatrix();
}
double huber(double e,double s) { const double a=std::abs(e)/s; return a<=2.5?1.0:2.5/a; }
}

Estimator::Estimator(EstimatorConfig c):config_(std::move(c)) {
  if(config_.vehicle_id<0||config_.vehicle_id>=4||config_.tof_sigma_m<=0||!config_.initial_covariance.allFinite()) throw std::invalid_argument("invalid estimator config");
  reset();
}
void Estimator::reset(uint64_t now) {
  state_=Estimate{}; state_.vehicle_id=config_.vehicle_id; state_.pose=config_.initial_pose; state_.covariance=config_.initial_covariance;
  state_.covariance(2,2)=std::max(state_.covariance(2,2),config_.initial_z_sigma_m*config_.initial_z_sigma_m); state_.initialized=true; state_.xy_valid=true; state_.sim_time_us=now;
  last_imu_us_=last_height_us_=last_baro_us_=last_absolute_us_=0; startup_sim_us_=now; last_local_tof_us_=last_peer_xy_us_=0; local_tof_rank_=0; previous_height_us_=0; baro_reference_hpa_=0; last_imu_yaw_=0; last_tof_us_.fill(0); last_uwb_us_.fill(0); tof_cache_={}; structural_information_.setZero(); structural_information_us_=0; held_acceleration_.setZero();
}
bool Estimator::fresh(uint64_t t,uint64_t timeout) const { return t&&t<=state_.sim_time_us&&state_.sim_time_us-t<=timeout; }
Estimate Estimator::estimate() const { Estimate e=state_; e.xy_structural_valid=fresh(last_absolute_us_,config_.terrain?std::min<uint64_t>(config_.prediction_timeout_us,500000):config_.prediction_timeout_us)&&e.covariance(0,0)+e.covariance(1,1)<2.0; const bool within_spawn_window=startup_sim_us_!=0&&state_.sim_time_us>=startup_sim_us_&&state_.sim_time_us-startup_sim_us_<=config_.spawn_prior_timeout_us; e.spawn_prior_valid=!e.xy_structural_valid&&within_spawn_window&&e.covariance(0,0)+e.covariance(1,1)<=config_.spawn_prior_xy_variance_limit; e.xy_peer_valid=!e.xy_structural_valid&&fresh(last_peer_xy_us_,config_.peer_timeout_us*2)&&local_tof_rank_>=1&&fresh(last_local_tof_us_,config_.tof_timeout_us*2)&&fresh(last_imu_us_,200000)&&e.covariance(0,0)+e.covariance(1,1)<=config_.spawn_prior_xy_variance_limit; e.xy_valid=e.xy_structural_valid||e.xy_peer_valid||e.spawn_prior_valid; e.z_valid=fresh(last_height_us_,config_.height_timeout_us)||fresh(last_baro_us_,500000); e.height_assisted=fresh(last_height_us_,config_.height_timeout_us); e.vz_derived_from_height=e.height_assisted&&state_.vz_derived_from_height; return e; }
void Estimator::advanceTo(uint64_t now) {
  if(!now||now<=state_.sim_time_us) return;
  if(state_.sim_time_us==0){state_.sim_time_us=now;if(!startup_sim_us_)startup_sim_us_=now;return;}
  double remain=(now-state_.sim_time_us)*1e-6;
  if(remain>2.0){ state_.covariance.diagonal().array()+=remain; state_.sim_time_us=now; held_acceleration_.setZero(); return; }
  while(remain>1e-8){ double dt=std::min(remain,0.02); Eigen::Vector3d a=fresh(last_imu_us_,100000)?held_acceleration_:Eigen::Vector3d::Zero(); Eigen::Vector3d p(state_.pose.x_m,state_.pose.y_m,state_.pose.z_m); p+=state_.velocity_enu_mps*dt+.5*a*dt*dt; state_.pose.x_m=p.x();state_.pose.y_m=p.y();state_.pose.z_m=p.z();state_.velocity_enu_mps+=a*dt; Eigen::Matrix<double,7,7> f=Eigen::Matrix<double,7,7>::Identity();f(0,4)=f(1,5)=f(2,6)=dt;Eigen::Matrix<double,7,7> q=Eigen::Matrix<double,7,7>::Zero();for(int i=0;i<3;i++){int v=4+i;q(i,i)=.25*dt*dt*dt/3;q(v,v)=.25*dt;q(i,v)=q(v,i)=.25*dt*dt/2;}q(3,3)=.003*dt;state_.covariance=f*state_.covariance*f.transpose()+q;remain-=dt;state_.sim_time_us+=static_cast<uint64_t>(std::llround(dt*1e6));} state_.sim_time_us=now;
}
void Estimator::predict(const ImuSample& s) {
  if(!s.sim_time_us||s.sim_time_us<=last_imu_us_||!s.acceleration_enu_mps2.allFinite()) return;
  advanceTo(s.sim_time_us);
  if(s.attitude_valid && std::isfinite(s.yaw_enu_rad) && std::isfinite(s.roll_flu_rad) && std::isfinite(s.pitch_flu_rad)){
    if(last_imu_us_)state_.pose.yaw_rad=wrapAngle(state_.pose.yaw_rad+wrapAngle(s.yaw_enu_rad-last_imu_yaw_));
    last_imu_yaw_=s.yaw_enu_rad;state_.pose.roll_rad=s.roll_flu_rad;state_.pose.pitch_rad=s.pitch_flu_rad;
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
bool Estimator::lineObservation(const TofFrame& f,int cell,const Pose3& pose,LineObservation& o) const {
  if(f.sensor_index<0||f.sensor_index>=3||!f.valid[cell])return false;const auto&m=config_.tof_mounts[f.sensor_index];double d=f.distance_m[cell];if(!std::isfinite(d)||d<m.min_range_m||d>m.max_range_m)return false;auto r=rotation(pose.roll_rad,pose.pitch_rad,pose.yaw_rad);Eigen::Vector3d off=r*(m.translation_body_m+d*tofRayBody(m,cell/8,cell%8)), ep=Eigen::Vector3d(pose.x_m,pose.y_m,pose.z_m)+off;if(ep.z()<config_.floor_z_m+.12)return false;double best=.6;bool found=false;for(const auto&w:config_.map_lines){Eigen::Vector2d line=w.end_m-w.start_m;double len=line.norm();if(len<1e-5||ep.z()<w.z_min_m||ep.z()>w.z_max_m)continue;Eigen::Vector2d t=line/len,n(-t.y(),t.x());double along=(ep.head<2>()-w.start_m).dot(t),res=n.dot(ep.head<2>()-w.start_m);if(along>=0&&along<=len&&std::abs(res)<best){best=std::abs(res);found=true;o.residual=res;o.jacobian<<n.x(),n.y(),n.dot(Eigen::Vector2d(-off.y(),off.x()));}}return found;
}
bool Estimator::terrainObservation(const TofFrame& f,int cell,const Pose3& pose,LineObservation& o) const {
  if(!config_.terrain || f.sensor_index<0 || f.sensor_index>=3 || !f.valid[cell]) return false;
  const auto&m=config_.tof_mounts[f.sensor_index]; const double d=f.distance_m[cell];
  if(!std::isfinite(d)||d<m.min_range_m||d>m.max_range_m) return false;
  const auto body_ray=tofRayBody(m,cell/8,cell%8);
  const auto r=rotation(pose.roll_rad,pose.pitch_rad,pose.yaw_rad); const Eigen::Vector3d off=r*m.translation_body_m;
  const Eigen::Vector3d ray=r*body_ray, origin=Eigen::Vector3d(pose.x_m,pose.y_m,pose.z_m)+off;
  TerrainMap::Hit hit; if(!config_.terrain->raycast(origin,ray,d+config_.terrain_range_gate_m,hit)) return false;
  if(std::abs(d-hit.depth_m)>config_.terrain_range_gate_m) return false;
  const Eigen::Vector3d endpoint=origin+d*ray; o.residual=hit.normal.dot(endpoint-hit.point);
  const Eigen::Vector3d lever=off+d*ray, yaw_derivative(-lever.y(),lever.x(),0.0);
  o.jacobian<<hit.normal.x(),hit.normal.y(),hit.normal.dot(yaw_derivative); o.normal=hit.normal;
  return std::isfinite(o.residual) && std::abs(o.residual)<=config_.terrain_plane_gate_m && o.jacobian.allFinite();
}
void Estimator::update(const Eigen::MatrixXd&h,const Eigen::VectorXd&r,const Eigen::MatrixXd&rc){
  const Eigen::Matrix<double,7,7> p=state_.covariance;
  const Eigen::MatrixXd s=h*p*h.transpose()+rc;
  const Eigen::MatrixXd k=p*h.transpose()*s.ldlt().solve(Eigen::MatrixXd::Identity(s.rows(),s.cols()));
  const Eigen::VectorXd d=k*r;
  state_.pose.x_m+=d(0);state_.pose.y_m+=d(1);state_.pose.z_m+=d(2);state_.pose.yaw_rad=wrapAngle(state_.pose.yaw_rad+d(3));state_.velocity_enu_mps+=d.segment<3>(4);
  const Eigen::Matrix<double,7,7> a=Eigen::Matrix<double,7,7>::Identity()-k*h;
  state_.covariance=a*p*a.transpose()+k*rc*k.transpose();state_.covariance=.5*(state_.covariance+state_.covariance.transpose()).eval();
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
  const Pose3 prior=state_.pose;
  Pose3 working=prior;
  Eigen::Matrix3d info=Eigen::Matrix3d::Zero(), modes=Eigen::Matrix3d::Identity();
  Eigen::Vector3d values=Eigen::Vector3d::Zero();
  int count=0,rank=0;
  double sum_squared=0;
  for(int iteration=0;iteration<5;++iteration) {
    info.setZero(); Eigen::Vector3d gradient=Eigen::Vector3d::Zero();
    count=0;sum_squared=0; int independent_normals=0; Eigen::Vector3d first_normal=Eigen::Vector3d::Zero();
    for(const auto& cached:tof_cache_) {
      if(cached.sensor_index<0 || !fresh(cached.sim_time_us,config_.tof_timeout_us))continue;
      // Avoid fitting a moving scene at different poses as though it were simultaneous.
      const double age=(state_.sim_time_us-cached.sim_time_us)*1e-6;
      Pose3 exposure_pose=working;
      exposure_pose.x_m-=state_.velocity_enu_mps.x()*age;
      exposure_pose.y_m-=state_.velocity_enu_mps.y()*age;
      exposure_pose.z_m-=state_.velocity_enu_mps.z()*age;
      for(int cell=0;cell<64;++cell) {
        LineObservation observation;
        const bool observed=config_.terrain ? terrainObservation(cached,cell,exposure_pose,observation) : lineObservation(cached,cell,exposure_pose,observation);
        if(!observed)continue;
        if(config_.terrain) {
          if(first_normal.squaredNorm()<0.5) first_normal=observation.normal;
          else if(independent_normals<2 && first_normal.cross(observation.normal).norm()>0.20) independent_normals=2;
        }
        const double weight=huber(observation.residual,config_.tof_sigma_m)/std::pow(config_.tof_sigma_m,2);
        info+=weight*observation.jacobian.transpose()*observation.jacobian;
        gradient+=weight*observation.jacobian.transpose()*observation.residual;
        sum_squared+=observation.residual*observation.residual;++count;
      }
    }
    if(count<(config_.terrain?8:2)){report.degenerate=true;report.reason="too_few_structural_returns";return report;}
    // Adjacent depth pixels share systematic errors: cap effective sample information.
    const double correlated=std::max(1.0,count/8.0);
    info/=correlated;gradient/=correlated;
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
  const Eigen::Vector3d shift(working.x_m-prior.x_m,working.y_m-prior.y_m,wrapAngle(working.yaw_rad-prior.yaw_rad));
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
  Eigen::MatrixXd h=Eigen::MatrixXd::Zero(rank,7),noise=Eigen::MatrixXd::Zero(rank,rank);
  Eigen::VectorXd residual(rank);int row=0;
  const double threshold=std::max(0.1,values(2)*(config_.terrain?1e-3:1e-4));
  for(int axis=0;axis<3;++axis)if(values(axis)>threshold) {
    h(row,0)=modes(0,axis);h(row,1)=modes(1,axis);h(row,3)=modes(2,axis);
    residual(row)=modes.col(axis).dot(shift);noise(row,row)=std::max(1.0/values(axis),1e-4);++row;
  }
  update(h,residual,noise);state_.degenerate_geometry=rank<3;
  // Old information cannot turn a currently unobservable axis into a fresh
  // absolute reference for the whole swarm.
  if(rank>=3)last_absolute_us_=frame.sim_time_us;
  local_tof_rank_=rank; last_local_tof_us_=frame.sim_time_us;
  report.accepted=true;report.degenerate=rank<3;report.inliers=count;
  report.innovation_norm=std::sqrt(sum_squared/count);report.reason=(config_.terrain?"terrain_point_to_plane_rank_":"point_to_line_rank_")+std::to_string(rank)+"_cumulative_"+std::to_string(cumulative_rank);
  return report;
}
FusionReport Estimator::fuseUwb(const UwbPeerSample& sample,const PeerEstimate& peer) {
  FusionReport report;
  // A currently well-constrained map solution takes precedence over reused,
  // correlated peer poses. Peers support gaps instead of repeatedly shifting
  // every independently localized member with the same common error.
  if(config_.terrain && estimate().xy_structural_valid && local_tof_rank_>=3) {
    report.gated=true;report.reason="independent_map_preferred";return report;
  }
  if(!peer.xy_structural_valid){report.gated=true;report.reason="peer_without_independent_structure";return report;}
  if(!sample.valid || peer.source!="ESTIMATE" || peer.vehicle_id==config_.vehicle_id ||
     sample.peer_id!=peer.vehicle_id || sample.peer_id<0 || sample.peer_id>=4 ||
     sample.sim_time_us<=last_uwb_us_[sample.peer_id] || !peer.pose_covariance.allFinite() ||
     !peer.velocity_enu_mps.allFinite() || !std::isfinite(sample.range_m) ||
     !std::isfinite(sample.bearing_rad)) {
    report.stale=true;report.reason="uwb_or_peer_invalid";return report;
  }
  advanceTo(sample.sim_time_us);
  if(!fresh(sample.sim_time_us,config_.peer_timeout_us) || !fresh(peer.sim_time_us,config_.peer_timeout_us)) {
    report.stale=true;report.reason="uwb_or_peer_stale";return report;
  }
  last_uwb_us_[sample.peer_id]=sample.sim_time_us;
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
  Eigen::Matrix<double,2,7> h=Eigen::Matrix<double,2,7>::Zero();
  h.block<1,3>(0,0)=-delta.transpose()/range;
  const Eigen::RowVector3d d_body(-body.y()/body.head<2>().squaredNorm(),body.x()/body.head<2>().squaredNorm(),0);
  const Eigen::RowVector3d d_world=d_body*body_rotation.transpose();
  h.block<1,3>(1,0)=-d_world;
  h(1,3)=d_body.dot(body_rotation.transpose()*Eigen::Vector3d(delta.y(),-delta.x(),0));
  Eigen::Matrix<double,2,3> j;
  j.row(0)=delta.transpose()/range;j.row(1)=d_world;
  // Reused peer estimates can be correlated; conservative inflation does not
  // claim a new absolute anchor or guaranteed covariance consistency.
  Eigen::Matrix2d noise=2*j*peer.pose_covariance*j.transpose();
  noise(0,0)+=std::pow(config_.uwb_range_sigma_m,2)+.25*peer_age*peer_age;
  noise(1,1)+=std::pow(config_.uwb_bearing_sigma_rad,2);
  const Eigen::Matrix2d innovation=h*state_.covariance*h.transpose()+noise;
  const double mahalanobis=residual.dot(innovation.ldlt().solve(residual));
  if(!std::isfinite(mahalanobis)||mahalanobis>13.82||std::abs(residual(0))>1||std::abs(residual(1))>.65) {
    report.gated=true;report.reason="uwb_innovation_gated";return report;
  }
  update(h,residual,noise);
  state_.covariance(0,0)=std::max(state_.covariance(0,0),config_.uwb_xy_covariance_floor_m2);
  state_.covariance(1,1)=std::max(state_.covariance(1,1),config_.uwb_xy_covariance_floor_m2);
  const bool trusted_peer=peer.xy_structural_valid && !peer.xy_peer_valid &&
      peer.pose_covariance(0,0)+peer.pose_covariance(1,1)<=config_.spawn_prior_xy_variance_limit;
  const bool own_baseline=local_tof_rank_>=1 && fresh(last_local_tof_us_,config_.tof_timeout_us*2);
  const bool fresh_heading=fresh(last_imu_us_,200000);
  if(trusted_peer && own_baseline && fresh_heading) last_peer_xy_us_=sample.sim_time_us;
  report.accepted=true;report.innovation_norm=residual.norm();
  report.reason="peer_estimate_relative_constraint";return report;
}
FusionReport Estimator::fuseHeightAssist(const HeightAssistSample&s){FusionReport q;if(s.source!="SIM_HEIGHT_ASSIST"||!std::isfinite(s.z_world_m)||s.sim_time_us<=last_height_us_){q.stale=true;q.reason="height_invalid_stale_source";return q;}advanceTo(s.sim_time_us);q.innovation_norm=std::abs(s.z_world_m-state_.pose.z_m);state_.pose.z_m=s.z_world_m;state_.covariance.row(2).setZero();state_.covariance.col(2).setZero();state_.covariance(2,2)=std::max(1e-6,config_.height_sigma_m*config_.height_sigma_m);if(previous_height_us_&&s.sim_time_us>previous_height_us_){double dt=(s.sim_time_us-previous_height_us_)*1e-6;if(dt>.001&&dt<.25){state_.velocity_enu_mps.z()=(s.z_world_m-previous_height_m_)/dt;state_.vz_derived_from_height=true;}}previous_height_m_=s.z_world_m;previous_height_us_=last_height_us_=s.sim_time_us;state_.height_assisted=state_.z_valid=true;q.accepted=true;q.reason="SIM_HEIGHT_ASSIST";return q;}
FusionReport Estimator::fuseBaroPressure(const ImuSample&s){FusionReport q;if(!config_.use_baro_fallback||!s.pressure_valid||!std::isfinite(s.absolute_pressure_hpa)||s.absolute_pressure_hpa<100||s.sim_time_us<=last_baro_us_){q.reason="baro_invalid";return q;}advanceTo(s.sim_time_us);if(!baro_reference_hpa_){baro_reference_hpa_=s.absolute_pressure_hpa;baro_reference_z_=state_.pose.z_m;}if(fresh(last_height_us_,config_.height_timeout_us)){q.reason="baro_skipped_height_assist";return q;}double z=baro_reference_z_+44330*(1-std::pow(s.absolute_pressure_hpa/baro_reference_hpa_,.190294957));Eigen::Matrix<double,1,7>h=Eigen::Matrix<double,1,7>::Zero();h(0,2)=1;Eigen::VectorXd r(1);r(0)=z-state_.pose.z_m;Eigen::MatrixXd rc(1,1);rc(0,0)=config_.baro_sigma_m*config_.baro_sigma_m;update(h,r,rc);last_baro_us_=s.sim_time_us;q.accepted=true;q.reason="baro_relative_to_initial_prior";return q;}
}
