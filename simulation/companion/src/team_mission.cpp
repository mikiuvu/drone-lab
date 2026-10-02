#include "companion/team_mission.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace companion {
const char* teamPhaseName(TeamPhase phase) {
  switch (phase) {
#define TEAM_PHASE(name) case TeamPhase::name: return #name;
    TEAM_PHASE(WAIT_READY) TEAM_PHASE(TAKEOFF) TEAM_PHASE(TRANSIT) TEAM_PHASE(ASSEMBLE)
    TEAM_PHASE(APPROACH) TEAM_PHASE(CONTACT) TEAM_PHASE(RETREAT)
    TEAM_PHASE(RETURN_ALL) TEAM_PHASE(LAND_ALL) TEAM_PHASE(COMPLETE)
    TEAM_PHASE(ABORT_RETURN) TEAM_PHASE(ABORTED)
#undef TEAM_PHASE
  }
  return "WAIT_READY";
}
TeamMission::TeamMission(TeamMissionConfig config) : config_(std::move(config)) {
  std::array<bool, kTeamSize> used{};
  for (int id : config_.order) {
    if (id < 0 || id >= kTeamSize || used[id]) throw std::invalid_argument("接触顺序必须包含四个不同机号");
    used[id] = true;
  }
  auto finite = [](const TeamPoint& p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) && std::isfinite(p.yaw);
  };
  if (!std::isfinite(config_.body_clearance_m) || config_.body_clearance_m<=0 || !std::isfinite(config_.cruise_z_m) || !std::isfinite(config_.home_ground_z_m) ||
      !std::isfinite(config_.contact_speed_mps) || config_.contact_speed_mps <= 0 ||
      !std::isfinite(config_.return_tolerance_m) || config_.return_tolerance_m <= 0 ||
      config_.cruise_z_m <= config_.home_ground_z_m + config_.body_clearance_m ||
      !config_.freshness_us || !config_.stable_us || !config_.stage_timeout_us || !config_.loss_abort_us ||
      !finite(config_.approach) || !finite(config_.target)) throw std::invalid_argument("任务参数无效");
  for (int i = 0; i < kTeamSize; ++i) {
    if (!finite(config_.homes[i]) || !finite(config_.assembly[i])) throw std::invalid_argument("航点无效");
    config_.assembly[i].z = config_.cruise_z_m;
    holds_[i] = config_.homes[i];
    holds_[i].z = config_.home_ground_z_m + config_.body_clearance_m;
  }
  for (const auto& p : config_.transit_waypoints)
    if (!finite(p)) throw std::invalid_argument("航路点无效");
  if (!config_.explicit_formation_offsets) {
    double cx = 0, cy = 0;
    for (const auto& p : config_.homes) { cx += p.x / kTeamSize; cy += p.y / kTeamSize; }
    for (int i = 0; i < kTeamSize; ++i)
      config_.formation_offsets[i] = {config_.homes[i].x-cx,config_.homes[i].y-cy,0,0};
  }
  for (const auto& p : config_.formation_offsets)
    if (!finite(p)) throw std::invalid_argument("编队偏移无效");
}
bool TeamMission::updateStatus(const TeamVehicleStatus& s, uint64_t wall) {
  if (s.vehicle < 0 || s.vehicle >= kTeamSize) return false;
  const double values[] = {s.x_m,s.y_m,s.z_m,s.vx_mps,s.vy_mps,s.vz_mps,s.yaw_rad};
  for (double v : values) if (!std::isfinite(v)) return false;
  const int i = s.vehicle;
  if (seen_[i] && (s.sim_time_us <= status_[i].sim_time_us || s.impact_seq < status_[i].impact_seq)) return false;
  status_[i] = s; seen_[i] = true; received_[i] = wall;
  return true;
}
bool TeamMission::healthy(uint64_t sim, uint64_t wall) const {
  for (int i = 0; i < kTeamSize; ++i) {
    const auto& s = status_[i];
    if (!seen_[i] || !s.xy_valid || !s.z_valid || !s.ready ||
        sim < s.sim_time_us || sim - s.sim_time_us > config_.freshness_us ||
        wall < received_[i] || wall - received_[i] > config_.freshness_us) return false;
    if (phase_ != TeamPhase::WAIT_READY && phase_ != TeamPhase::TAKEOFF &&
        phase_ != TeamPhase::LAND_ALL && phase_ != TeamPhase::COMPLETE &&
        phase_ != TeamPhase::ABORTED && !s.armed) return false;
  }
  return true;
}
bool TeamMission::near(int i, const TeamPoint& p, bool xy) const {
  const auto& s = status_[i];
  return (!xy || std::hypot(s.x_m-p.x,s.y_m-p.y) <= config_.return_tolerance_m) &&
         std::abs(s.z_m-p.z) <= (phase_ == TeamPhase::LAND_ALL ? .035 : .08) &&
         (xy ? std::hypot(std::hypot(s.vx_mps,s.vy_mps),s.vz_mps) <= (phase_==TeamPhase::LAND_ALL?.12:.30) : std::abs(s.vz_mps)<=.12);
}
bool TeamMission::barrier(bool condition, uint64_t now) {
  if (!condition) { stable_ = false; return false; }
  if (!stable_) { stable_ = true; stable_start_ = now; }
  return now - stable_start_ >= config_.stable_us;
}
void TeamMission::enter(TeamPhase p, uint64_t now) {
  const auto previous = phase_;
  const int previous_return_index = return_index_;
  phase_ = p; phase_start_ = now; stable_ = false;
  if (p == TeamPhase::CONTACT) impact_baseline_ = status_[activeVehicle()].impact_seq;
  if (p == TeamPhase::RETREAT) retreat_assembly_ = false;
  if (p == TeamPhase::RETURN_ALL || p == TeamPhase::ABORT_RETURN) {
    return_index_ = config_.transit_waypoints.empty() ? -1 :
        std::min(transit_index_,static_cast<int>(config_.transit_waypoints.size())-1);
    if (p == TeamPhase::ABORT_RETURN) {
      if (previous == TeamPhase::TAKEOFF) return_index_ = -1;
      else if (previous == TeamPhase::TRANSIT) return_index_ = transit_index_-1;
      else if (previous == TeamPhase::LAND_ALL) return_index_ = -1;
      else if (previous == TeamPhase::RETURN_ALL) return_index_ = previous_return_index;
    }
  }
}
void TeamMission::abort(uint64_t now, const char* reason) {
  aborted_ = true; reason_ = reason; enter(TeamPhase::ABORT_RETURN, now);
}
int TeamMission::activeVehicle() const {
  return (phase_ == TeamPhase::APPROACH || phase_ == TeamPhase::CONTACT || phase_ == TeamPhase::RETREAT)
      ? config_.order[turn_] : -1;
}
std::array<TeamCommand, kTeamSize> TeamMission::step(uint64_t sim, uint64_t wall) {
  clock_ = std::max(clock_, sim); sim = clock_;
  const bool terminal = phase_ == TeamPhase::COMPLETE || phase_ == TeamPhase::ABORTED;
  if (terminal) paused_ = false;
  const bool valid = healthy(sim, wall);
  if (!valid && !terminal) {
    stable_ = false;
    if (!paused_) {
      paused_ = true; pause_sim_ = sim; pause_wall_ = wall;
      for (int i = 0; i < kTeamSize; ++i) if (seen_[i])
        holds_[i] = {status_[i].x_m,status_[i].y_m,status_[i].z_m,status_[i].yaw_rad};
    }
    if (phase_ != TeamPhase::WAIT_READY && !aborted_ && wall >= pause_wall_ &&
        wall - pause_wall_ >= config_.loss_abort_us) abort(sim,"定位或状态数据中断");
  } else if (valid && paused_) {
    phase_start_ += sim - pause_sim_;
    if (phase_start_ > sim) phase_start_ = sim;
    paused_ = false; stable_ = false;
    // 暂停期间的碰撞不计入接触阶段。
    if (phase_ == TeamPhase::CONTACT) impact_baseline_ = status_[activeVehicle()].impact_seq;
  }
  if (valid && !paused_ && !terminal) {
    const int active = activeVehicle();
    auto allNear = [&](bool homes, bool landing) {
      for (int i = 0; i < kTeamSize; ++i) {
        TeamPoint p = homes ? config_.homes[i] : config_.assembly[i];
        p.z = landing ? config_.home_ground_z_m + config_.body_clearance_m : config_.cruise_z_m;
        if (!near(i,p)) return false;
      }
      return true;
    };
    auto othersNear = [&]() {
      for (int i = 0; i < kTeamSize; ++i) if (i != active && !near(i,config_.assembly[i])) return false;
      return true;
    };
    auto routePoint = [&](int i, int index) {
      TeamPoint p = config_.transit_waypoints[index];
      p.x += config_.formation_offsets[i].x; p.y += config_.formation_offsets[i].y;
      return p;
    };
    auto allRouteNear = [&](int index) {
      for (int i = 0; i < kTeamSize; ++i) if (!near(i,routePoint(i,index))) return false;
      return true;
    };
    if (!aborted_ && phase_ != TeamPhase::WAIT_READY && sim - phase_start_ >= config_.stage_timeout_us) {
      abort(sim,phase_ == TeamPhase::CONTACT ? "接触超时" : "阶段超时");
      // 中止返航或降落超时保持当前阶段，等待有效位置满足条件。
    } else switch (phase_) {
      case TeamPhase::WAIT_READY: if (barrier(true,sim)) enter(TeamPhase::TAKEOFF,sim); break;
      case TeamPhase::TAKEOFF: {
        bool all = true;
        for (int i = 0; i < kTeamSize; ++i) {
          TeamPoint p = config_.homes[i]; p.z = config_.cruise_z_m;
          all = all && status_[i].armed && near(i,p,false);
        }
        if (barrier(all,sim)) enter(config_.transit_waypoints.empty() ? TeamPhase::ASSEMBLE : TeamPhase::TRANSIT,sim);
        break;
      }
      case TeamPhase::TRANSIT:
        if (barrier(allRouteNear(transit_index_),sim)) {
          if (++transit_index_ == static_cast<int>(config_.transit_waypoints.size())) enter(TeamPhase::ASSEMBLE,sim);
          else { phase_start_ = sim; stable_ = false; }
        }
        break;
      case TeamPhase::ASSEMBLE: if (barrier(allNear(false,false),sim)) enter(TeamPhase::APPROACH,sim); break;
      case TeamPhase::APPROACH:
        if (barrier(near(active,config_.approach) && othersNear() &&
            std::abs(std::remainder(status_[active].yaw_rad-config_.approach.yaw,6.283185307179586)) <= .15,sim))
          enter(TeamPhase::CONTACT,sim);
        break;
      case TeamPhase::CONTACT:
        if (status_[active].impact_seq > impact_baseline_) {
          impact_baseline_ = status_[active].impact_seq;
          const auto& s = status_[active];
          const double distance = std::hypot(std::hypot(s.x_m-config_.target.x,
              s.y_m-config_.target.y),s.z_m-config_.target.z);
          const double heading_error = std::abs(std::remainder(s.yaw_rad-config_.approach.yaw,6.283185307179586));
          if (othersNear() && distance <= .35 && heading_error <= .3) {
            completed_.push_back(active); enter(TeamPhase::RETREAT,sim);
          }
        }
        break;
      case TeamPhase::RETREAT:
        if (barrier(othersNear() && near(active,retreat_assembly_ ? config_.assembly[active] : config_.approach),sim)) {
          if (!retreat_assembly_) { retreat_assembly_ = true; stable_ = false; }
          else if (++turn_ == kTeamSize) enter(TeamPhase::RETURN_ALL,sim);
          else enter(TeamPhase::APPROACH,sim);
        }
        break;
      case TeamPhase::RETURN_ALL: case TeamPhase::ABORT_RETURN:
        if (return_index_ >= 0) {
          if (barrier(allRouteNear(return_index_),sim)) { --return_index_; stable_ = false; phase_start_ = sim; }
        } else if (barrier(allNear(true,false),sim)) enter(TeamPhase::LAND_ALL,sim);
        break;
      case TeamPhase::LAND_ALL: {
        const bool all_landed = std::all_of(status_.begin(),status_.end(),
            [](const TeamVehicleStatus& s) { return s.landed; });
        bool stable_ground=true;
        for(int i=0;i<kTeamSize;++i){auto p=config_.homes[i];p.z=config_.home_ground_z_m+config_.body_clearance_m;stable_ground=stable_ground&&near(i,p,false);}
        if (barrier(stable_ground && all_landed,sim))
          enter(aborted_ ? TeamPhase::ABORTED : TeamPhase::COMPLETE,sim);
        break;
      }
      default: break;
    }
  }
  std::array<TeamCommand,kTeamSize> commands{};
  ++sequence_;
  for (int i = 0; i < kTeamSize; ++i) {
    auto& c = commands[i]; TeamPoint p = config_.assembly[i];
    c.vehicle = i; c.sim_time_us = sim; c.sequence = sequence_; c.phase = phase_;
    c.active_vehicle = activeVehicle(); c.paused = paused_;
    if (phase_ == TeamPhase::WAIT_READY || phase_ == TeamPhase::TAKEOFF ||
        phase_ == TeamPhase::RETURN_ALL || phase_ == TeamPhase::ABORT_RETURN ||
        phase_ == TeamPhase::LAND_ALL || phase_ == TeamPhase::COMPLETE || phase_ == TeamPhase::ABORTED) {
      p = config_.homes[i]; p.z = config_.cruise_z_m;
      if (phase_ == TeamPhase::WAIT_READY || phase_ == TeamPhase::LAND_ALL ||
          phase_ == TeamPhase::COMPLETE || phase_ == TeamPhase::ABORTED) p.z = config_.home_ground_z_m + config_.body_clearance_m;
    } else if (i == c.active_vehicle) {
      p = phase_ == TeamPhase::RETREAT && retreat_assembly_ ? config_.assembly[i] : config_.approach;
      if (phase_ == TeamPhase::CONTACT) {
        const double distance = std::hypot(config_.target.x-config_.approach.x,config_.target.y-config_.approach.y);
        const double travel = std::min(distance,config_.contact_speed_mps * (sim-phase_start_) / 1e6);
        p.x += std::cos(p.yaw)*travel; p.y += std::sin(p.yaw)*travel;
        c.contact_speed_mps = config_.contact_speed_mps;
      }
    }
    const int route_index = phase_ == TeamPhase::TRANSIT ? transit_index_ :
        ((phase_ == TeamPhase::RETURN_ALL || phase_ == TeamPhase::ABORT_RETURN) ? return_index_ : -1);
    if (route_index >= 0) {
      p = config_.transit_waypoints[route_index];
      p.x += config_.formation_offsets[i].x; p.y += config_.formation_offsets[i].y;
      p.yaw = config_.homes[i].yaw;
    }
    if (paused_) { p = holds_[i]; c.contact_speed_mps = 0; }
    c.target_x_m = p.x; c.target_y_m = p.y; c.target_z_m = p.z; c.target_yaw_rad = p.yaw;
    c.disarm = phase_ == TeamPhase::COMPLETE || phase_ == TeamPhase::ABORTED;
    c.complete = phase_ == TeamPhase::COMPLETE;
  }
  return commands;
}
}  // namespace companion
