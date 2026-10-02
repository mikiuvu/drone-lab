#include <cassert>
#include <cmath>
#include "companion/estimator.hpp"

using namespace companion;

static EstimatorConfig config() {
  EstimatorConfig c;
  c.initial_covariance.setIdentity();
  c.initial_covariance *= 0.02;
  c.initial_pose.z_m = 0.2;
  return c;
}

int main() {
  Estimator estimator(config());
  estimator.reset(1000000);

  ImuSample imu;
  imu.sim_time_us = 1000000;
  imu.acceleration_valid = true;
  imu.acceleration_enu_mps2 = Eigen::Vector3d(0.8, -0.3, 0.0);
  estimator.predict(imu);
  estimator.advanceTo(2000000);
  const auto before = estimator.estimate();
  assert(before.velocity_enu_mps.head<2>().norm() > 0.1);

  const auto xy_before = before.pose;
  estimator.resetInertialVelocityWhileLanded(2100000);
  const auto landed = estimator.estimate();
  assert(landed.velocity_enu_mps.norm() == 0.0);
  assert(landed.pose.x_m == xy_before.x_m);
  assert(landed.pose.y_m == xy_before.y_m);
  assert(landed.pose.z_m == xy_before.z_m);

  estimator.advanceTo(3000000);
  const auto after = estimator.estimate();
  assert(after.velocity_enu_mps.norm() == 0.0);
  assert(std::abs(after.pose.x_m - landed.pose.x_m) < 1e-12);
  assert(std::abs(after.pose.y_m - landed.pose.y_m) < 1e-12);
  assert(after.covariance.allFinite());
  imu.sim_time_us = 3010000;
  estimator.predict(imu);
  estimator.advanceTo(3030000);
  assert(estimator.estimate().velocity_enu_mps.x() > 0.0);
  estimator.resetInertialVelocityWhileLanded(3000000);
  assert(estimator.estimate().velocity_enu_mps.x() > 0.0);
  estimator.resetInertialVelocityWhileLanded();
  assert(estimator.estimate().velocity_enu_mps.norm() == 0.0);
  const auto pre_contact=estimator.estimate();
  assert(estimator.fuseMapContact(pre_contact.sim_time_us,Eigen::Vector3d::UnitX(),pre_contact.pose.x_m+.05).accepted);
  const auto contact=estimator.estimate();
  assert(contact.pose.x_m>pre_contact.pose.x_m);
  assert(contact.pose.y_m==pre_contact.pose.y_m);
  assert(contact.pose.yaw_rad==pre_contact.pose.yaw_rad);
  assert(!estimator.fuseMapContact(pre_contact.sim_time_us,Eigen::Vector3d(2,0,0),1).accepted);
  return 0;
}
