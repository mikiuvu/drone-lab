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
#include <stdexcept>

namespace companion {
struct ArmorTrack {
  bool valid{false}, measured{false};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()}, velocity{Eigen::Vector3d::Zero()};
  double age_s{0};
  uint64_t measurement_time_us{0};  ///< 最后一次通过关联的曝光时间，预测不生成新曝光。
  Eigen::Vector3d measurement_position{Eigen::Vector3d::Zero()};  ///< 对应曝光的原始世界位置，不随预测移动。
};

class ArmorTracker {
 public:
  /** @param process_noise_density 白噪声加速度谱密度，m²/s³。
   * @param initial_velocity_variance 重捕获后的速度先验方差，m²/s²。
   */
  explicit ArmorTracker(double process_noise_density=1.0,double initial_velocity_variance=1.0)
      :process_noise_density_(process_noise_density),initial_velocity_variance_(initial_velocity_variance){
    if(!std::isfinite(process_noise_density)||process_noise_density<=0||
       !std::isfinite(initial_velocity_variance)||initial_velocity_variance<=0)
      throw std::invalid_argument("invalid armor motion noise");
  }
  /** @brief 新曝光时刻的世界位置；先传播再用创新距离关联，同一帧不重复计入。 */
  bool observe(uint64_t stamp, const Eigen::Vector3d& point, double variance=.0009) {
    if(!std::isfinite(variance)||variance<=0)return false;
    return observe(stamp,point,Eigen::Matrix3d::Identity()*variance);
  }
  /** @brief 世界位置及其完整观测协方差 m²；矩阵须有限、对称且正定。 */
  bool observe(uint64_t stamp,const Eigen::Vector3d& point,const Eigen::Matrix3d& covariance){
    if(!stamp||stamp<=last_frame_||!point.allFinite()||!covariance.allFinite())return false;
    const double scale=covariance.cwiseAbs().maxCoeff();
    if(scale>0&&(covariance/scale-covariance.transpose()/scale).cwiseAbs().maxCoeff()>1e-12)return false;
    const Eigen::Matrix3d noise=.5*covariance+.5*covariance.transpose();
    if(Eigen::LLT<Eigen::Matrix3d>(noise).info()!=Eigen::Success)return false;
    if(!last_good_||stamp-last_good_>500000){
      state_.setZero();state_.head<3>()=point;cov_.setIdentity();cov_.topLeftCorner<3,3>()=noise;
      cov_.bottomRightCorner<3,3>()*=initial_velocity_variance_;
      clock_=last_frame_=last_good_=stamp;last_point_=point;hits_=1;return true;
    }
    if(!predict(stamp))return false;
    const Eigen::Vector3d residual=point-state_.head<3>();
    if(residual.norm()>.6){last_frame_=stamp;return false;}
    Eigen::Matrix<double,3,6> h=Eigen::Matrix<double,3,6>::Zero();h.leftCols<3>().setIdentity();
    const Eigen::Matrix3d innovation=h*cov_*h.transpose()+noise;
    if(!innovation.allFinite())return false;
    const Eigen::LDLT<Eigen::Matrix3d> factor(innovation);
    if(factor.info()!=Eigen::Success||!factor.isPositive())return false;
    const Eigen::Matrix<double,6,3> gain=cov_*h.transpose()*factor.solve(Eigen::Matrix3d::Identity());
    const Eigen::Matrix<double,6,1> updated=state_+gain*residual;
    const auto a=(Eigen::Matrix<double,6,6>::Identity()-gain*h).eval();
    Eigen::Matrix<double,6,6> updated_covariance=a*cov_*a.transpose()+gain*noise*gain.transpose();
    updated_covariance=(.5*updated_covariance+.5*updated_covariance.transpose()).eval();
    if(!gain.allFinite()||!updated.allFinite()||!updated_covariance.allFinite())return false;
    state_=updated;cov_=updated_covariance;
    for(int i=3;i<6;++i)state_(i)=std::max(-1.0,std::min(1.0,state_(i)));
    last_frame_=last_good_=stamp;last_point_=point;++hits_;return true;
  }

  /** @brief 对当前时刻作有界预测，不修改滤波时钟，允许随后融合延迟到达的曝光帧。 */
  ArmorTrack at(uint64_t now) const {
    ArmorTrack result;
    if(!last_good_||now<clock_)return result;
    result.age_s=(now-last_good_)*1e-6;
    result.measurement_time_us=last_good_;
    result.measurement_position=last_point_;
    result.valid=hits_>=3&&result.age_s<=.8;
    result.measured=result.valid&&result.age_s<=.15;
    result.position=state_.head<3>()+state_.tail<3>()*((now-clock_)*1e-6);
    result.velocity=state_.tail<3>();return result;
  }
 private:
  bool predict(uint64_t now) {
    const double dt=(now-clock_)*1e-6;
    Eigen::Matrix<double,6,6> f=Eigen::Matrix<double,6,6>::Identity();f.topRightCorner<3,3>()=Eigen::Matrix3d::Identity()*dt;
    Eigen::Matrix<double,6,6> q=Eigen::Matrix<double,6,6>::Zero();
    for(int i=0;i<3;++i){q(i,i)=dt*dt*dt/3;q(i+3,i+3)=dt;q(i,i+3)=q(i+3,i)=dt*dt/2;}
    const Eigen::Matrix<double,6,1> predicted=f*state_;
    const Eigen::Matrix<double,6,6> covariance=f*cov_*f.transpose()+q*process_noise_density_;
    if(!predicted.allFinite()||!covariance.allFinite())return false;
    state_=predicted;cov_=covariance;clock_=now;return true;
  }
  Eigen::Matrix<double,6,1> state_{Eigen::Matrix<double,6,1>::Zero()};
  Eigen::Matrix<double,6,6> cov_{Eigen::Matrix<double,6,6>::Identity()};
  uint64_t clock_{0},last_good_{0},last_frame_{0};
  Eigen::Vector3d last_point_{Eigen::Vector3d::Zero()};
  double process_noise_density_,initial_velocity_variance_;
  unsigned hits_{0};
};
} // namespace companion
