#include "companion/estimator.hpp"
#include "companion/terrain.hpp"
#include <cassert>
#include <cmath>

using namespace companion;
namespace {
EstimatorConfig corner() {
  using T=TerrainMap::Triangle;
  EstimatorConfig c;c.initial_pose.x_m=.16;c.initial_pose.y_m=0;
  c.initial_covariance*=.02;
  c.terrain=std::make_shared<TerrainMap>(std::vector<T>{
      T{Eigen::Vector3f(2,-3,-2),Eigen::Vector3f(2,3,-2),Eigen::Vector3f(2,3,2)},
      T{Eigen::Vector3f(2,-3,-2),Eigen::Vector3f(2,3,2),Eigen::Vector3f(2,-3,2)},
      T{Eigen::Vector3f(0,.3,-2),Eigen::Vector3f(3,.3,-2),Eigen::Vector3f(3,.3,2)},
      T{Eigen::Vector3f(0,.3,-2),Eigen::Vector3f(3,.3,2),Eigen::Vector3f(0,.3,2)}});
  c.tof_mounts[0].horizontal_fov_rad=c.tof_mounts[0].vertical_fov_rad=kPi/4;
  return c;
}
TofFrame frameAt(const EstimatorConfig& config,const Eigen::Vector3d& actual,uint64_t stamp) {
  TofFrame frame;frame.sensor_index=0;frame.sim_time_us=stamp;
  for(int cell=0;cell<64;++cell){
    TerrainMap::Hit hit;
    if(config.terrain->raycast(actual,Estimator::tofRayBody(config.tof_mounts[0],cell/8,cell%8),4,hit)){
      frame.valid[cell]=1;frame.distance_m[cell]=hit.depth_m;
    }
  }
  return frame;
}
void requiresIndependentConsistentExposuresAndKeepsStepBound() {
  auto c=corner();const Eigen::Vector3d actual=Eigen::Vector3d::Zero();
  const auto frame=frameAt(c,actual,1000000);
  Estimator strict(c);const auto rejected=strict.fuseTof(frame);
  assert(!rejected.accepted&&rejected.reason=="terrain_update_trust_region");
  assert(!rejected.bounded_recovery_requested&&rejected.recovery_gate=="disabled");
  assert(rejected.fit_rank==3&&std::abs(rejected.fit_translation_m-.16)<1e-4);
  assert(std::abs(rejected.fit_yaw_rad)<1e-4);
  Estimator estimator(c);
  const auto pending=estimator.fuseTof(frame,true);
  assert(!pending.accepted&&pending.reason=="terrain_recovery_pending");
  assert(pending.bounded_recovery_requested&&pending.recovery_gate=="confirmation");
  const auto before=estimator.estimate();
  assert(before.pose.x_m==c.initial_pose.x_m&&before.pose.y_m==c.initial_pose.y_m);
  assert(!estimator.fuseTof(frame,true).accepted);
  assert(!estimator.fuseTof(frameAt(c,actual,1020000),true).accepted);
  const auto accepted=estimator.fuseTof(frameAt(c,actual,1060000),true);
  assert(accepted.accepted&&accepted.reason.find("terrain_bounded_recovery_rank_")==0);
  const auto after=estimator.estimate();
  assert(std::hypot(after.pose.x_m-before.pose.x_m,after.pose.y_m-before.pose.y_m)<=c.terrain_max_translation_update_m+1e-9);
  assert(std::hypot(after.pose.x_m,after.pose.y_m)<.03);
  assert(std::abs(after.pose.yaw_rad-before.pose.yaw_rad)<=c.terrain_max_yaw_update_rad);
  assert(accepted.innovation_norm<=c.tof_sigma_m&&after.xy_structural_valid);
}
void changingCandidatesAndLeavingRecoveryResetConfirmation() {
  auto c=corner();Estimator estimator(c);const Eigen::Vector3d origin=Eigen::Vector3d::Zero();
  assert(!estimator.fuseTof(frameAt(c,origin,1000000),true).accepted);
  assert(!estimator.fuseTof(frameAt(c,{-0.06,0,0},1060000),true).accepted);
  assert(!estimator.fuseTof(frameAt(c,origin,1080000),true).accepted);
  assert(!estimator.fuseTof(frameAt(c,origin,1100000),false).accepted);
  assert(!estimator.fuseTof(frameAt(c,origin,1160000),true).accepted);
  assert(estimator.fuseTof(frameAt(c,origin,1220000),true).accepted);
}
void largeJumpAndStaleSequenceCannotTriggerRecovery() {
  auto c=corner();c.initial_pose.x_m=.3;c.initial_pose.y_m=-.3;Estimator estimator(c);
  for(uint64_t stamp:{1000000,1060000,1120000})assert(!estimator.fuseTof(frameAt(c,{0,0,0},stamp),true).accepted);
  assert(estimator.estimate().pose.x_m==c.initial_pose.x_m&&estimator.estimate().pose.y_m==c.initial_pose.y_m);
  c=corner();Estimator stale(c);
  assert(!stale.fuseTof(frameAt(c,{0,0,0},1000000),true).accepted);
  assert(!stale.fuseTof(frameAt(c,{0,0,0},1400000),true).accepted);
  assert(stale.estimate().reference_origin==-1);
}
}
int main(){
  requiresIndependentConsistentExposuresAndKeepsStepBound();
  changingCandidatesAndLeavingRecoveryResetConfirmation();largeJumpAndStaleSequenceCannotTriggerRecovery();
}
