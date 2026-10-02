#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace companion {
struct TeamPoint { double x = 0, y = 0, z = 0, yaw = 0; };
struct TeamMissionConfig {
  double cruise_z_m = 1.2;
  std::vector<TeamPoint> homes{{-1.5,0,.5125,0},{-.5,0,.5125,0},{.5,0,.5125,0},{1.5,0,.5125,0}}, assembly{{-1.5,4,1.2,0},{-.5,4,1.2,0},{.5,4,1.2,0},{1.5,4,1.2,0}};
  TeamPoint approach{}, target{};
  std::vector<TeamPoint> transit_waypoints;
  std::vector<TeamPoint> formation_offsets;
  bool explicit_formation_offsets = false;
  std::vector<int> order{0, 1, 2, 3};
  double contact_speed_mps = .2, return_tolerance_m = .10;
  double route_tolerance_m = .10, route_speed_tolerance_mps = .30;
  bool fly_through_route = false;
  bool pipelined_contact = false;
  double contact_exit_side = 0;
  std::vector<TeamPoint> completed_holding;
  double home_ground_z_m = .5;
  double body_clearance_m = .0125;
  uint64_t freshness_us = 1000000, stable_us = 500000;
  // Zero keeps a healthy vehicle working without a stage deadline.
  uint64_t stage_timeout_us = 0, loss_abort_us = 5000000;
};
struct TeamVehicleStatus {
  int vehicle = -1;
  uint64_t sim_time_us = 0, impact_seq = 0;
  double x_m = 0, y_m = 0, z_m = 0;
  double vx_mps = 0, vy_mps = 0, vz_mps = 0, yaw_rad = 0;
  bool xy_valid = false, z_valid = false, ready = false, armed = false, landed = false;
  bool flow_valid = false;
  std::string xy_reason;
};
enum class TeamPhase { WAIT_READY, TAKEOFF, TRANSIT, ASSEMBLE, APPROACH, CONTACT,
                       RETREAT, RETURN_ALL, LAND_ALL, COMPLETE, ABORT_RETURN, ABORTED };
const char* teamPhaseName(TeamPhase phase);
struct TeamCommand {
  int vehicle = -1, active_vehicle = -1;
  uint64_t sim_time_us = 0, sequence = 0;
  TeamPhase phase = TeamPhase::WAIT_READY;
  double target_x_m = 0, target_y_m = 0, target_z_m = 0, target_yaw_rad = 0;
  double contact_speed_mps = 0;
  bool paused = false, degraded = false, skipped = false, disarm = false, complete = false;
  std::string degraded_reason;
};
// 输入为定位估计及物理碰撞事件计数；wall_time_us 使用单调时钟。
class TeamMission {
 public:
  explicit TeamMission(TeamMissionConfig config);
  bool updateStatus(const TeamVehicleStatus& status, uint64_t wall_time_us);
  std::vector<TeamCommand> step(uint64_t sim_time_us, uint64_t wall_time_us);
  TeamPhase phase() const { return phase_; }
  int activeVehicle() const;
  bool paused() const { return false; }
  bool aborted() const { return aborted_; }
  bool partial() const { return !skipped_.empty(); }
  const std::string& reason() const { return reason_; }
  const std::vector<int>& completed() const { return completed_; }
  const std::vector<int>& skipped() const { return skipped_; }
  const std::vector<TeamVehicleStatus>& statuses() const { return status_; }
  double flightElapsedSeconds() const { return flight_start_?((flight_end_?flight_end_:clock_)-flight_start_)*1e-6:0; }
 private:
  bool validForBarrier(int id, uint64_t sim, uint64_t wall) const;
  bool near(int id, const TeamPoint& point, bool xy = true) const;
  bool barrier(bool condition, uint64_t now);
  void enter(TeamPhase phase, uint64_t now);
  void skipUnrecoverable(uint64_t now);
  void abort(uint64_t now, const char* reason);
  TeamMissionConfig config_;
  std::vector<TeamVehicleStatus> status_;
  std::vector<bool> seen_, skipped_now_;
  std::vector<uint64_t> received_, loss_start_;
  std::vector<TeamPoint> holds_;
  TeamPhase phase_ = TeamPhase::WAIT_READY;
  uint64_t clock_ = 0, wall_clock_ = 0, phase_start_ = 0, stable_start_ = 0;
  uint64_t sequence_ = 0, impact_baseline_ = 0;
  uint64_t flight_start_ = 0, flight_end_ = 0;
  bool stable_ = false, aborted_ = false, retreat_assembly_ = false;
  int turn_ = 0;
  int transit_index_ = 0, return_index_ = -1;
  std::vector<int> completed_;
  std::vector<int> skipped_;
  std::string reason_;
};
}  // namespace companion
