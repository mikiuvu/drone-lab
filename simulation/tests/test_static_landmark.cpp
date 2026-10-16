#include "companion/estimator.hpp"
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <cassert>
#include <cmath>
#include <limits>

using namespace companion;
namespace {
Eigen::Matrix3d rotation(double roll, double pitch, double yaw) {
  return (Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ())*
      Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY())*
      Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX())).toRotationMatrix();
}
EstimatorConfig config() {
  EstimatorConfig c;c.vehicle_id=2;c.initial_pose={10.47,-.25,.6,0,0,0};
  c.initial_covariance.setIdentity();c.initial_covariance*=.02;
  c.initial_covariance(3,3)=.01;return c;
}
void correctsTranslationWithoutUsingOwnPositionAsMeasurement() {
  auto c=config();Estimator estimator(c);
  const Eigen::Vector3d landmark(11.172,-.262,.341),actual(10.70,-.25,.6);
  const auto report=estimator.fuseStaticLandmark(1000000,landmark,landmark-actual,.0009);
  assert(report.accepted);
  const auto after=estimator.estimate();
  assert(std::hypot(after.pose.x_m-actual.x(),after.pose.y_m-actual.y())<.025);
  assert(after.xy_structural_valid&&after.reference_origin==2&&after.reference_path==std::vector<int>{2});
  assert(!estimator.fuseStaticLandmark(1000000,landmark,landmark-actual,.0009).accepted);
  assert((estimator.estimate().covariance-after.covariance).norm()==0);
  estimator.reset();
  assert(estimator.fuseStaticLandmark(1000000,landmark,landmark-actual,.0009).accepted);
}
void usesExposureAttitudeAndMotion() {
  auto c=config();c.initial_pose.roll_rad=.2;c.initial_pose.pitch_rad=-.15;c.initial_pose.yaw_rad=.4;
  Estimator estimator(c);
  const Eigen::Vector3d velocity(.4,.2,0);
  const auto body=rotation(.2,-.15,.4).transpose()*velocity;
  assert(estimator.fuseOpticalFlow({900000,body.x(),body.y(),1,.0001}).accepted);
  ImuSample imu;imu.attitude_valid=true;imu.roll_flu_rad=.2;imu.pitch_flu_rad=-.15;
  imu.sim_time_us=1000000;imu.yaw_enu_rad=-.7;estimator.predict(imu);
  imu.sim_time_us=1100000;imu.yaw_enu_rad=-.4;estimator.predict(imu);
  const auto before=estimator.estimate();
  const Eigen::Vector3d landmark(11.172,-.262,.341);
  const Eigen::Vector3d exposure=Eigen::Vector3d(before.pose.x_m,before.pose.y_m,before.pose.z_m)-.1*before.velocity_enu_mps;
  const Eigen::Vector3d relative=rotation(.2,-.15,.4).transpose()*(landmark-exposure);
  const auto report=estimator.fuseStaticLandmark(1000000,landmark,relative,.0009);
  assert(report.accepted&&report.innovation_norm<1e-9);
  const auto after=estimator.estimate();
  assert(std::hypot(after.pose.x_m-before.pose.x_m,after.pose.y_m-before.pose.y_m)<1e-9);
  assert((after.velocity_enu_mps-before.velocity_enu_mps).norm()<1e-9);
}
void linearizationMatchesNumericalObservation() {
  auto c=config();c.initial_pose.roll_rad=.2;c.initial_pose.pitch_rad=-.15;c.initial_pose.yaw_rad=.4;
  Estimator estimator(c);estimator.advanceTo(1100000);
  const auto before=estimator.estimate();
  const Eigen::Vector3d landmark(11.172,-.262,.341);
  Eigen::Matrix<double,7,1> state;
  state<<before.pose.x_m,before.pose.y_m,before.pose.z_m,before.pose.yaw_rad,before.velocity_enu_mps;
  const auto predict=[&](const Eigen::Matrix<double,7,1>& x)->Eigen::Vector3d {
    return rotation(.2,-.15,x(3)).transpose()*(landmark-x.head<3>()+.1*x.tail<3>());
  };
  Eigen::Matrix<double,3,7> h;
  for(int axis=0;axis<7;++axis){
    Eigen::Matrix<double,7,1> delta=Eigen::Matrix<double,7,1>::Zero();delta(axis)=1e-6;
    h.col(axis)=(predict(state+delta)-predict(state-delta))/(2e-6);
  }
  const Eigen::Vector3d residual(.01,-.02,.015);
  const Eigen::Matrix3d noise=Eigen::Matrix3d::Identity()*(.0009+2.25*std::pow(.1,4));
  const Eigen::Matrix<double,7,3> gain=before.covariance*h.transpose()*
      (h*before.covariance*h.transpose()+noise).inverse();
  const Eigen::Matrix<double,7,1> expected=state+gain*residual;
  assert(estimator.fuseStaticLandmark(1000000,landmark,predict(state)+residual,.0009).accepted);
  const auto after=estimator.estimate();
  Eigen::Matrix<double,7,1> actual;
  actual<<after.pose.x_m,after.pose.y_m,after.pose.z_m,after.pose.yaw_rad,after.velocity_enu_mps;
  assert((actual-expected).norm()<1e-8);
  const Eigen::Matrix<double,7,7> a=Eigen::Matrix<double,7,7>::Identity()-gain*h;
  assert((after.covariance-(a*before.covariance*a.transpose()+gain*noise*gain.transpose())).norm()<1e-8);
  assert((Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,7,7>>(after.covariance).eigenvalues().minCoeff()>-1e-12));
}
void rejectsOutliersOldFramesAndInvalidValues() {
  Estimator estimator(config());estimator.advanceTo(1200000);
  const Eigen::Vector3d landmark(11.172,-.262,.341),relative(.5,0,-.2);
  const auto before=estimator.estimate();
  assert(estimator.fuseStaticLandmark(1200000,landmark+Eigen::Vector3d(5,0,0),relative,.0009).gated);
  assert(estimator.fuseStaticLandmark(1000000,landmark,relative,.0009).stale);
  assert(estimator.fuseStaticLandmark(1201000,landmark,relative,-1).stale);
  assert(estimator.fuseStaticLandmark(1201000,landmark,relative,std::numeric_limits<double>::quiet_NaN()).stale);
  assert(estimator.fuseStaticLandmark(1201000,landmark,{9,0,0},.0009).stale);
  assert((estimator.estimate().covariance-before.covariance).norm()==0);
  assert(estimator.estimate().reference_origin==-1);
}
}
int main() {
  correctsTranslationWithoutUsingOwnPositionAsMeasurement();usesExposureAttitudeAndMotion();
  linearizationMatchesNumericalObservation();rejectsOutliersOldFramesAndInvalidValues();
}
