#pragma once
#include "companion/types.hpp"

namespace companion {

class Estimator {
 public:
  explicit Estimator(EstimatorConfig config);
  void reset(uint64_t now_us = 0);
  void advanceTo(uint64_t now_us);
  void predict(const ImuSample& sample);
  // Call when the vehicle is confirmed landed and disarmed. This clears the
  // inertial motion state without changing the position or map reference.
  void resetInertialVelocityWhileLanded(uint64_t now_us = 0);
  FusionReport fuseMapContact(uint64_t now_us, const Eigen::Vector3d& normal, double plane_offset_m);
  FusionReport fuseTof(const TofFrame& frame);
  FusionReport fuseUwb(const UwbPeerSample& sample, const PeerEstimate& peer);
  FusionReport fuseHeightAssist(const HeightAssistSample& sample);
  FusionReport fuseBaroPressure(const ImuSample& sample);
  Estimate estimate() const;
  const EstimatorConfig& config() const { return config_; }
  static Eigen::Vector3d tofRayBody(const TofMount& mount, int row, int col);

 private:
  struct LineObservation { double residual{0}; Eigen::RowVector3d jacobian{Eigen::RowVector3d::Zero()}; Eigen::Vector3d normal{Eigen::Vector3d::Zero()}; };
  bool lineObservation(const TofFrame&, int, const Pose3&, LineObservation&) const;
  bool terrainObservation(const TofFrame&, int, const Pose3&, LineObservation&) const;
  void update(const Eigen::MatrixXd&, const Eigen::VectorXd&, const Eigen::MatrixXd&);
  bool fresh(uint64_t stamp, uint64_t timeout) const;
  EstimatorConfig config_;
  Estimate state_;
  std::array<TofFrame,3> tof_cache_{};
  std::array<uint64_t,3> last_tof_us_{};
  std::array<uint64_t,4> last_uwb_us_{};
  Eigen::Matrix3d structural_information_{Eigen::Matrix3d::Zero()};
  uint64_t structural_information_us_{0};
  uint64_t last_imu_us_{0}, last_height_us_{0}, last_baro_us_{0}, last_absolute_us_{0};
  uint64_t startup_sim_us_{0};
  uint64_t last_local_tof_us_{0}, last_peer_xy_us_{0};
  int local_tof_rank_{0};
  double last_imu_yaw_{0}, baro_reference_hpa_{0}, baro_reference_z_{0}, previous_height_m_{0};
  uint64_t previous_height_us_{0};
  Eigen::Vector3d held_acceleration_{Eigen::Vector3d::Zero()};
};
}
