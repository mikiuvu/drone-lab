/**
 * @file armor_tracker.hpp
 * @brief 滑轨装甲板的世界系匀速卡尔曼跟踪器。
 * 借鉴 rm_auto_aim 的预测→关联→更新及 DETECTING/TRACKING/TEMP_LOST 状态组织；
 * 滑轨板不使用旋转机器人半径模型。位置输入必须由图像 PnP 加曝光时刻自身位姿得到。
 */
#pragma once
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <cmath>
#include <cstdint>

namespace companion {
struct ArmorTrack {
  bool valid{false}, measured{false};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()}, velocity{Eigen::Vector3d::Zero()};
  double age_s{0};
};

class ArmorTracker {
 public:
  /** @brief 新曝光时刻的世界位置；先传播再用创新距离关联，同一帧不重复计入。 */
  bool observe(uint64_t stamp, const Eigen::Vector3d& point, double variance=.0009) {
    if(!stamp||stamp<=last_frame_||!point.allFinite()||!std::isfinite(variance)||variance<=0)return false;
    if(!last_good_||stamp-last_good_>500000){
      state_.setZero();state_.head<3>()=point;cov_.setIdentity();cov_.topLeftCorner<3,3>()*=variance;
      clock_=last_frame_=last_good_=stamp;hits_=1;return true;
    }
    predict(stamp);last_frame_=stamp;
    const Eigen::Vector3d residual=point-state_.head<3>();
    if(residual.norm()>.6)return false;
    Eigen::Matrix<double,3,6> h=Eigen::Matrix<double,3,6>::Zero();h.leftCols<3>().setIdentity();
    const Eigen::Matrix3d noise=Eigen::Matrix3d::Identity()*variance;
    const Eigen::Matrix3d innovation=h*cov_*h.transpose()+noise;
    const Eigen::Matrix<double,6,3> gain=cov_*h.transpose()*innovation.ldlt().solve(Eigen::Matrix3d::Identity());
    state_+=gain*residual;
    const auto a=(Eigen::Matrix<double,6,6>::Identity()-gain*h).eval();
    cov_=a*cov_*a.transpose()+gain*noise*gain.transpose();
    for(int i=3;i<6;++i)state_(i)=std::max(-1.0,std::min(1.0,state_(i)));
    last_good_=stamp;++hits_;return true;
  }

  /** @brief 对当前时刻作有界预测，不修改滤波时钟，允许随后融合延迟到达的曝光帧。 */
  ArmorTrack at(uint64_t now) const {
    ArmorTrack result;
    if(!last_good_||now<clock_)return result;
    result.age_s=(now-last_good_)*1e-6;
    result.valid=hits_>=3&&result.age_s<=.8;
    result.measured=result.valid&&result.age_s<=.15;
    result.position=state_.head<3>()+state_.tail<3>()*((now-clock_)*1e-6);
    result.velocity=state_.tail<3>();return result;
  }
 private:
  void predict(uint64_t now) {
    const double dt=(now-clock_)*1e-6;
    Eigen::Matrix<double,6,6> f=Eigen::Matrix<double,6,6>::Identity();f.topRightCorner<3,3>()=Eigen::Matrix3d::Identity()*dt;
    Eigen::Matrix<double,6,6> q=Eigen::Matrix<double,6,6>::Zero();
    for(int i=0;i<3;++i){q(i,i)=dt*dt*dt/3;q(i+3,i+3)=dt;q(i,i+3)=q(i+3,i)=dt*dt/2;}
    state_=f*state_;cov_=f*cov_*f.transpose()+q;clock_=now;
  }
  Eigen::Matrix<double,6,1> state_{Eigen::Matrix<double,6,1>::Zero()};
  Eigen::Matrix<double,6,6> cov_{Eigen::Matrix<double,6,6>::Identity()};
  uint64_t clock_{0},last_good_{0},last_frame_{0};
  unsigned hits_{0};
};
} // namespace companion
