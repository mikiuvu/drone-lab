#include "companion/estimator.hpp"
#include <cassert>
#include <cmath>

using namespace companion;
int main() {
  EstimatorConfig config;
  config.initial_pose={.5,0,1.5,0,0,.4};
  config.initial_covariance.setIdentity();config.initial_covariance*=.01;
  config.initial_covariance(0,3)=config.initial_covariance(3,0)=.004;
  Estimator estimator(config);
  ImuSample sample;sample.sim_time_us=1000000;sample.attitude_valid=true;sample.yaw_enu_rad=3.1;
  assert(estimator.yawAfterImu(sample)==.4);
  estimator.predict(sample);
  assert(estimator.estimate().pose.yaw_rad==.4);

  // 位置与航向的交叉协方差使已知平面观测同时修正地图航向。
  assert(estimator.fuseMapContact(sample.sim_time_us,Eigen::Vector3d::UnitX(),.4).accepted);
  const auto before=estimator.estimate();
  assert(std::abs(before.pose.yaw_rad-.4)>.001);
  sample.sim_time_us+=10000;sample.yaw_enu_rad=-3.1;
  const double expected=wrapAngle(before.pose.yaw_rad+(2*kPi-6.2));
  assert(std::abs(estimator.yawAfterImu(sample)-expected)<1e-12);
  assert(estimator.estimate().pose.yaw_rad==before.pose.yaw_rad);
  assert((estimator.estimate().covariance-before.covariance).norm()==0);
  sample.acceleration_valid=true;
  sample.acceleration_enu_mps2={std::cos(expected),std::sin(expected),0};
  estimator.predict(sample);
  assert(std::abs(estimator.estimate().pose.yaw_rad-expected)<1e-12);
  estimator.advanceTo(sample.sim_time_us+100000);
  assert((estimator.estimate().velocity_enu_mps-.1*sample.acceleration_enu_mps2).norm()<1e-12);

  const double current=estimator.estimate().pose.yaw_rad;
  sample.yaw_enu_rad=0;
  assert(estimator.yawAfterImu(sample)==current);
  sample.sim_time_us+=200000;sample.attitude_valid=false;
  assert(estimator.yawAfterImu(sample)==current);
  sample.attitude_valid=true;sample.roll_flu_rad=std::numeric_limits<double>::quiet_NaN();
  assert(estimator.yawAfterImu(sample)==current);
  sample.roll_flu_rad=0;sample.acceleration_enu_mps2.x()=std::numeric_limits<double>::infinity();
  assert(estimator.yawAfterImu(sample)==current);
  estimator.reset();sample.acceleration_enu_mps2.setZero();sample.yaw_enu_rad=1.2;
  assert(estimator.yawAfterImu(sample)==config.initial_pose.yaw_rad);
}
