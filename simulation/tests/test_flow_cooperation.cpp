#include <cassert>
#include <cmath>
#include <limits>

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
  return 0;
}
