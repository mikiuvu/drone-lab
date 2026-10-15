#include <cassert>
#include <cmath>
#include <limits>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include "companion/estimator.hpp"
#include "companion/protocol.hpp"
#include "companion/terrain.hpp"

namespace {
companion::EstimatorConfig config(int id) {
  companion::EstimatorConfig c;
  c.vehicle_id=id;
  c.initial_covariance.setIdentity();
  c.initial_pose.z_m=1.0;
  return c;
}

void structuralReferencePropagation() {
  using namespace companion;
  using T=TerrainMap::Triangle;
  auto c=config(0);c.initial_pose.x_m=.03;c.initial_covariance*=.02;
  c.tof_mounts[0].horizontal_fov_rad=c.tof_mounts[0].vertical_fov_rad=kPi/4;
  c.terrain=std::make_shared<TerrainMap>(std::vector<T>{
      T{Eigen::Vector3f(2,-3,-2),Eigen::Vector3f(2,3,-2),Eigen::Vector3f(2,3,2)},
      T{Eigen::Vector3f(2,-3,-2),Eigen::Vector3f(2,3,2),Eigen::Vector3f(2,-3,2)},
      T{Eigen::Vector3f(0,.3,-2),Eigen::Vector3f(3,.3,-2),Eigen::Vector3f(3,.3,2)},
      T{Eigen::Vector3f(0,.3,-2),Eigen::Vector3f(3,.3,2),Eigen::Vector3f(0,.3,2)}});
  TofFrame frame;frame.sensor_index=0;
  for(int cell=0;cell<64;++cell) {
    TerrainMap::Hit hit;
    const auto ray=Estimator::tofRayBody(c.tof_mounts[0],cell/8,cell%8);
    if(c.terrain->raycast({0,0,1},ray,4,hit)) {
      frame.valid[cell]=1;frame.distance_m[cell]=hit.depth_m;
    }
  }
  Estimator leader(c);
  std::vector<Estimator> followers;
  for(int id=1;id<=6;++id) {
    auto f=config(id);f.initial_pose.x_m=id+.3;f.initial_pose.y_m=.2;
    f.initial_covariance*=.1;followers.emplace_back(f);
  }
  for(uint64_t step=1;step<=30;++step) {
    frame.sim_time_us=step*20000;
    leader.fuseOpticalFlow({frame.sim_time_us,0,0,.95,.0004});
    assert(leader.fuseTof(frame).accepted);
    const auto estimate=leader.estimate();
    assert(estimate.xy_structural_valid && estimate.reference_origin==0);
    const auto peer=parsePeerEstimateDatagram(serializeEstimateDatagram(estimate),1);
    assert(peer);
    for(int id=1;id<=6;++id) {
      auto& follower=followers[id-1];
      follower.fuseOpticalFlow({frame.sim_time_us,0,0,.95,.0004});
      assert(follower.fuseUwb({frame.sim_time_us,0,double(id),kPi,true,.95,false},*peer).accepted);
    }
  }
  for(int id=1;id<=6;++id) {
    const auto e=followers[id-1].estimate();
    assert(std::hypot(e.pose.x_m-id,e.pose.y_m)<.2);
    assert(e.reference_origin==0 && e.xy_peer_valid);
  }
  const auto before=leader.estimate().covariance;
  TofFrame empty;empty.sensor_index=1;empty.sim_time_us=frame.sim_time_us;
  assert(!leader.fuseTof(empty).accepted);
  assert((leader.estimate().covariance-before).norm()<1e-12);
  assert(!leader.fuseTof(frame).accepted);
}

void flowYawLinearization() {
  using namespace companion;
  for (const double yaw : {-2.8, -.4, .4, 2.8}) {
    auto c = config(30);
    c.initial_pose.yaw_rad = yaw;
    c.initial_pose.roll_rad = .2;
    c.initial_pose.pitch_rad = -.15;
    c.initial_covariance *= .0001;
    c.initial_covariance(3,3) = .04;
    Estimator estimator(c);
    ImuSample imu;
    imu.sim_time_us = 1000000;
    imu.attitude_valid = true;
    imu.roll_flu_rad = .2;
    imu.pitch_flu_rad = -.15;
    imu.yaw_enu_rad = yaw;
    imu.acceleration_valid = true;
    imu.acceleration_enu_mps2 = {10, 3, 2};
    estimator.predict(imu);
    estimator.advanceTo(1100000);
    const auto before = estimator.estimate();
    const auto bodyVelocity = [&](double heading, const Eigen::Vector3d& velocity) -> Eigen::Vector2d {
      const Eigen::Matrix3d r = (Eigen::AngleAxisd(heading, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(before.pose.pitch_rad, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(before.pose.roll_rad, Eigen::Vector3d::UnitX())).toRotationMatrix();
      return (r.transpose() * velocity).head<2>();
    };
    Eigen::Matrix<double,2,7> jacobian = Eigen::Matrix<double,2,7>::Zero();
    constexpr double step = 1e-6;
    jacobian.col(3) = (bodyVelocity(yaw + step, before.velocity_enu_mps) -
                       bodyVelocity(yaw - step, before.velocity_enu_mps)) / (2 * step);
    for (int axis = 0; axis < 3; ++axis) {
      Eigen::Vector3d delta = Eigen::Vector3d::Zero();
      delta(axis) = step;
      jacobian.col(4 + axis) = (bodyVelocity(yaw, before.velocity_enu_mps + delta) -
                               bodyVelocity(yaw, before.velocity_enu_mps - delta)) / (2 * step);
    }
    // 世界航向和速度共同旋转，不应改变机体系速度观测。
    Eigen::Matrix<double,7,1> gauge = Eigen::Matrix<double,7,1>::Zero();
    gauge(3) = 1;
    gauge(4) = -before.velocity_enu_mps.y();
    gauge(5) = before.velocity_enu_mps.x();
    assert((jacobian * gauge).norm() < 1e-8);
    const Eigen::Vector2d measured = bodyVelocity(yaw - .12, before.velocity_enu_mps);
    const Eigen::Matrix2d noise = Eigen::Matrix2d::Identity() * .0004 / .95;
    const Eigen::Matrix<double,7,2> gain = before.covariance * jacobian.transpose() *
        (jacobian * before.covariance * jacobian.transpose() + noise).inverse();
    const Eigen::Matrix<double,7,1> correction = gain * (measured - bodyVelocity(yaw, before.velocity_enu_mps));
    const Eigen::Matrix<double,7,7> a = Eigen::Matrix<double,7,7>::Identity() - gain * jacobian;
    const Eigen::Matrix<double,7,7> expected_covariance = a * before.covariance * a.transpose() + gain * noise * gain.transpose();
    assert(estimator.fuseOpticalFlow({1100000, measured.x(), measured.y(), .95, .0004}).accepted);
    const auto after = estimator.estimate();
    assert(std::abs(wrapAngle(after.pose.yaw_rad - yaw) - correction(3)) < 1e-8);
    assert((after.covariance - expected_covariance).norm() < 1e-8);
    assert(std::abs(wrapAngle(after.pose.yaw_rad - (yaw - .12))) < .09);
    assert((after.covariance - after.covariance.transpose()).norm() < 1e-12);
    assert((Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,7,7>>(after.covariance).eigenvalues().minCoeff() >= -1e-12));
  }
}

void delayedFlowUsesExposureAttitude() {
  using namespace companion;
  const auto rotate=[](double roll,double pitch,double yaw) {
    return (Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ())*
        Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY())*
        Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX())).toRotationMatrix();
  };
  for (const double turn : {-.8, .8}) {
    auto c=config(31);
    c.initial_pose.yaw_rad=.4;
    c.initial_pose.roll_rad=.2;
    c.initial_pose.pitch_rad=-.15;
    c.initial_covariance(3,3)=1e-12;
    Estimator estimator(c);
    const Eigen::Vector3d initial_body=rotate(.2,-.15,.4).transpose()*Eigen::Vector3d(.6,-.3,0);
    assert(estimator.fuseOpticalFlow({900000,initial_body.x(),initial_body.y(),1,.0001}).accepted);
    ImuSample imu;
    imu.sim_time_us=1000000;imu.attitude_valid=true;
    imu.yaw_enu_rad=-.7;imu.roll_flu_rad=.2;imu.pitch_flu_rad=-.15;
    estimator.predict(imu);
    imu.sim_time_us=1100000;imu.yaw_enu_rad+=turn;
    estimator.predict(imu);
    const auto before=estimator.estimate();
    // Raw IMU yaw has a different origin; the known map yaw at mid-exposure is .4+turn/2.
    const Eigen::Vector3d exposure_body=rotate(.2,-.15,.4+turn*.5).transpose()*before.velocity_enu_mps;
    const auto fused=estimator.fuseOpticalFlow({1050000,exposure_body.x(),exposure_body.y(),1,.0001});
    assert(fused.accepted);
    assert(fused.innovation_norm<1e-9);
    assert((estimator.estimate().velocity_enu_mps-before.velocity_enu_mps).norm()<1e-9);
    assert(std::abs(wrapAngle(estimator.estimate().pose.yaw_rad-before.pose.yaw_rad))<1e-12);
    assert(estimator.fuseOpticalFlow({1050000,0,0,1,.0001}).stale);
  }
}

void delayedFlowArbitraryRotation() {
  using namespace companion;
  const Eigen::Quaterniond origin(Eigen::AngleAxisd(-.7,Eigen::Vector3d::UnitZ())*
      Eigen::AngleAxisd(-.15,Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(.2,Eigen::Vector3d::UnitX()));
  for (const auto axis : {Eigen::Vector3d(1,0,0),Eigen::Vector3d(0,1,0),Eigen::Vector3d(1,2,-1).normalized().eval()}) {
    for (double map_offset : {1.1,-2.0}) {
      auto c=config(32);c.initial_pose.yaw_rad=-.7+map_offset;
      c.initial_pose.roll_rad=.2;c.initial_pose.pitch_rad=-.15;c.initial_covariance(3,3)=.04;
      Estimator estimator(c);
      const Eigen::Matrix3d map_rotation=Eigen::AngleAxisd(map_offset,Eigen::Vector3d::UnitZ()).toRotationMatrix();
      const Eigen::Vector3d seed=(map_rotation*origin.toRotationMatrix()).transpose()*Eigen::Vector3d(.7,-.4,0);
      assert(estimator.fuseOpticalFlow({900000,seed.x(),seed.y(),1,.0001}).accepted);
      for (int i=0;i<=10;++i) {
        const Eigen::Matrix3d r=(Eigen::Quaterniond(Eigen::AngleAxisd(i*.08,axis))*origin).toRotationMatrix();
        ImuSample sample;sample.sim_time_us=1000000+i*20000;sample.attitude_valid=true;
        sample.roll_flu_rad=std::atan2(r(2,1),r(2,2));
        sample.pitch_flu_rad=std::atan2(-r(2,0),std::hypot(r(0,0),r(1,0)));
        sample.yaw_enu_rad=std::atan2(r(1,0),r(0,0));
        estimator.predict(sample);
      }
      const auto before=estimator.estimate();
      Estimator perturbed=estimator;
      const Eigen::Quaterniond exposure=Eigen::Quaterniond(Eigen::AngleAxisd(.44,axis))*origin;
      const Eigen::Vector3d body=(map_rotation*exposure.toRotationMatrix()).transpose()*before.velocity_enu_mps;
      const auto report=estimator.fuseOpticalFlow({1110000,body.x(),body.y(),1,.0001});
      assert(report.accepted&&report.innovation_norm<1e-9);
      const auto after=estimator.estimate();
      assert((after.velocity_enu_mps-before.velocity_enu_mps).norm()<1e-9);
      assert((after.covariance-after.covariance.transpose()).norm()<1e-12);
      assert((Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,7,7>>(after.covariance).eigenvalues().minCoeff()>=-1e-12));
      const Eigen::Matrix3d exposure_rotation=map_rotation*exposure.toRotationMatrix();
      const auto observation=[&](double yaw_delta,const Eigen::Vector3d& velocity)->Eigen::Vector2d {
        const Eigen::Matrix3d r=Eigen::AngleAxisd(yaw_delta,Eigen::Vector3d::UnitZ()).toRotationMatrix()*exposure_rotation;
        return (r.transpose()*velocity).head<2>();
      };
      constexpr double epsilon=1e-6;
      Eigen::Matrix<double,2,7> jacobian=Eigen::Matrix<double,2,7>::Zero();
      jacobian.col(3)=(observation(epsilon,before.velocity_enu_mps)-observation(-epsilon,before.velocity_enu_mps))/(2*epsilon);
      for(int component=0;component<3;++component) {
        Eigen::Vector3d delta=Eigen::Vector3d::Zero();delta(component)=epsilon;
        jacobian.col(4+component)=(observation(0,before.velocity_enu_mps+delta)-observation(0,before.velocity_enu_mps-delta))/(2*epsilon);
      }
      const Eigen::Vector2d residual(.03,-.02);
      const Eigen::Matrix2d noise=Eigen::Matrix2d::Identity()*.0001;
      const Eigen::Matrix<double,7,2> gain=before.covariance*jacobian.transpose()*
          (jacobian*before.covariance*jacobian.transpose()+noise).inverse();
      const Eigen::Matrix<double,7,1> correction=gain*residual;
      const Eigen::Matrix<double,7,7> a=Eigen::Matrix<double,7,7>::Identity()-gain*jacobian;
      const Eigen::Matrix<double,7,7> covariance=a*before.covariance*a.transpose()+gain*noise*gain.transpose();
      assert(perturbed.fuseOpticalFlow({1110000,body.x()+residual.x(),body.y()+residual.y(),1,.0001}).accepted);
      const auto corrected=perturbed.estimate();
      assert((corrected.velocity_enu_mps-before.velocity_enu_mps-correction.tail<3>()).norm()<1e-8);
      assert(std::abs(wrapAngle(corrected.pose.yaw_rad-before.pose.yaw_rad)-correction(3))<1e-8);
      assert((corrected.covariance-covariance).norm()<1e-8);
    }
  }
}

void flowHistoryCoverage() {
  using namespace companion;
  for (bool before_history : {false,true}) {
    Estimator estimator(config(33));
    ImuSample sample;sample.attitude_valid=true;
    sample.sim_time_us=before_history?1100000:1000000;
    estimator.predict(sample);
    sample.sim_time_us=before_history?1200000:1100001;
    estimator.predict(sample);
    const auto before=estimator.estimate();
    const auto rejected=estimator.fuseOpticalFlow({1050000,1,0,1,.0001});
    assert(!rejected.accepted&&rejected.stale&&rejected.reason=="flow_attitude_history_missing");
    assert((estimator.estimate().velocity_enu_mps-before.velocity_enu_mps).norm()==0);
    assert((estimator.estimate().covariance-before.covariance).norm()==0);
    assert(estimator.fuseOpticalFlow({sample.sim_time_us,0,0,1,.0001}).accepted);
    assert(estimator.fuseOpticalFlow({sample.sim_time_us+1000,0,0,1,.0001}).accepted);
  }
}
}

int main() {
  using namespace companion;
  EstimatorConfig flow_config=config(6);
  Estimator flow(flow_config);
  assert(flow.fuseOpticalFlow({100000, 1.0, 0.0, 0.95, 0.0004}).accepted);
  assert(std::abs(flow.estimate().velocity_enu_mps.z()) < 1e-9);
  flow.advanceTo(700001);
  assert(!flow.estimate().flow_valid);
  assert(flow.fuseOpticalFlow({800000, 0.0, 0.0, 0.25, 0.0004}).gated);
  Estimator continuous(config(20));
  for(uint64_t stamp=1000000;stamp<=11000000;stamp+=20000){
    continuous.fuseHeightAssist({stamp,1.0,"SIM_HEIGHT_ASSIST"});
    assert(continuous.fuseOpticalFlow({stamp,1,0,.95,.0004}).accepted);
  }
  assert(continuous.estimate().flow_valid&&continuous.estimate().xy_valid);
  assert(std::abs(continuous.estimate().pose.x_m-10)<.05);
  assert(!continuous.estimate().xy_structural_valid);
  Estimator vertical(config(21));
  vertical.fuseHeightAssist({1000000,1,"SIM_HEIGHT_ASSIST"});
  vertical.fuseHeightAssist({1020000,1.02,"SIM_HEIGHT_ASSIST"});
  const double vz=vertical.estimate().velocity_enu_mps.z();
  vertical.fuseOpticalFlow({1020000,0,0,.95,.0004});
  assert(std::abs(vertical.estimate().velocity_enu_mps.z()-vz)<1e-9);

  auto parsed_flow=parseOpticalFlowDatagram(R"({"schema":1,"kind":"optical_flow","vehicle":6,"sim_time_us":1,"vx_body_mps":1,"vy_body_mps":2,"quality":0.95,"variance_m2s2":0.0004,"source":"SIM_FLOW_MODULE"})",6);
  assert(parsed_flow && parsed_flow->quality==0.95);
  assert(parseOpticalFlowDatagram(R"({"schema":1,"kind":"optical_flow","vehicle":6,"sim_time_us":1,"vx_body_mps":1,"vy_body_mps":2,"quality":0.95,"variance_m2s2":0.0004,"source":"MODULE"})",6));
  assert(!parseOpticalFlowDatagram(R"({"schema":1,"kind":"optical_flow","vehicle":6,"sim_time_us":-1,"vx_body_mps":1,"vy_body_mps":2,"quality":0.95,"variance_m2s2":0.0004})",6));
  auto parsed_anchor=parseAnchorRangeDatagram(R"({"schema":1,"kind":"uwb_anchor","vehicle":6,"sim_time_us":1,"anchor_id":4,"range_m":3,"variance_m2":0.0025,"quality":0.25,"valid":true})",6);
  assert(parsed_anchor && parsed_anchor->anchor_id==4 && parsed_anchor->quality==0.25);

  EstimatorConfig anchors=config(1);
  anchors.uwb_anchors={{0,Eigen::Vector3d(10,0,1)},{1,Eigen::Vector3d(0,10,1)}};
  Estimator anchored(anchors);
  assert(anchored.fuseAnchorRange({100000,0,10.0,.0025,.95,true}).accepted);
  assert(!anchored.estimate().xy_structural_valid);
  assert(anchored.fuseAnchorRange({200000,1,10.0,.0025,.25,true}).accepted);
  assert(!anchored.estimate().xy_structural_valid);

  assert(anchored.fuseAnchorRange({300000,1,10.0,.0025,.95,true}).accepted);
  assert(anchored.estimate().xy_structural_valid);
  assert(!anchored.fuseAnchorRange({400000,1,std::numeric_limits<double>::quiet_NaN(),.0025,.95,true}).accepted);
  Estimator one_anchor(anchors);
  for(uint64_t t=100000;t<800000;t+=100000)one_anchor.fuseAnchorRange({t,0,10,.0025,.95,true});
  assert(!one_anchor.estimate().xy_structural_valid);
  one_anchor.advanceTo(2000000);
  assert(one_anchor.fuseAnchorRange({800000,1,10,.0025,.95,true}).stale);

  Estimator receiver(config(0));
  PeerEstimate peer;
  peer.vehicle_id=6; peer.sim_time_us=100000; peer.pose.x_m=2.0;
  peer.xy_structural_valid=true; peer.source="ESTIMATE"; peer.pose_covariance.setIdentity();
  peer.reference_origin=6; peer.reference_seq=7; peer.reference_time_us=90000; peer.reference_path={6};
  assert(receiver.fuseUwb({100000,6,2.0,0.0,true,.95,false},peer).accepted);
  const double x=receiver.estimate().pose.x_m;
  PeerEstimate duplicate=peer; duplicate.vehicle_id=5;
  assert(receiver.fuseUwb({200000,5,2.0,0.0,true,.95,false},duplicate).gated);
  assert(receiver.estimate().pose.x_m==x);
  auto loop=peer;loop.reference_seq=8;loop.reference_path={6,0,5};loop.vehicle_id=5;loop.sim_time_us=200000;loop.reference_time_us=190000;
  assert(receiver.fuseUwb({200000,5,2,0,true,.95,false},loop).gated);
  assert(receiver.estimate().pose.x_m==x);

  // One independently referenced peer corrects every follower's shared drift.
  for(int id=1;id<=6;++id){
    auto cfg=config(id);cfg.initial_pose.x_m=id+.3;cfg.initial_pose.y_m=.2;
    cfg.initial_covariance*=.1;
    Estimator follower(cfg);
    PeerEstimate leader;leader.vehicle_id=0;leader.pose.z_m=1;leader.source="ESTIMATE";
    leader.xy_structural_valid=true;leader.pose_covariance=Eigen::Matrix3d::Identity()*.0001;
    leader.reference_origin=0;leader.reference_path={0};
    for(uint64_t step=1;step<=30;++step){
      const auto stamp=step*20000;leader.sim_time_us=stamp;leader.reference_time_us=stamp;leader.reference_seq=step;
      follower.fuseOpticalFlow({stamp,0,0,.95,.0004});
      assert(follower.fuseUwb({stamp,0,double(id),kPi,true,.95,false},leader).accepted);
    }
    const auto estimate=follower.estimate();
    assert(std::hypot(estimate.pose.x_m-id,estimate.pose.y_m)<.2);
    assert(estimate.reference_origin==0&&estimate.xy_peer_valid);
  }
  structuralReferencePropagation();
  flowYawLinearization();
  delayedFlowUsesExposureAttitude();
  delayedFlowArbitraryRotation();
  flowHistoryCoverage();
  return 0;
}
