#include "companion/team_mission.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace companion {
const char* teamPhaseName(TeamPhase p) {
  switch (p) {
#define P(x) case TeamPhase::x: return #x;
    P(WAIT_READY) P(TAKEOFF) P(TRANSIT) P(ASSEMBLE) P(APPROACH) P(CONTACT)
    P(RETREAT) P(RETURN_ALL) P(LAND_ALL) P(COMPLETE) P(ABORT_RETURN) P(ABORTED)
#undef P
  }
  return "WAIT_READY";
}

TeamMission::TeamMission(TeamMissionConfig c) : config_(std::move(c)) {
  const size_t n = config_.homes.size();
  if (n == 0 || config_.assembly.size() != n || config_.order.size() != n)
    throw std::invalid_argument("队伍数量必须为正且配置长度一致");
  std::vector<bool> used(n);
  for (int id : config_.order) {
    if (id < 0 || static_cast<size_t>(id) >= n || used[id])
      throw std::invalid_argument("接触顺序必须包含每个机号一次");
    used[id] = true;
  }
  auto finite = [](const TeamPoint& p) { return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) && std::isfinite(p.yaw); };
  if (!std::isfinite(config_.cruise_z_m) || !std::isfinite(config_.home_ground_z_m) ||
      !std::isfinite(config_.body_clearance_m) || config_.body_clearance_m <= 0 ||
      config_.cruise_z_m <= config_.home_ground_z_m + config_.body_clearance_m ||
      !std::isfinite(config_.contact_speed_mps) || config_.contact_speed_mps <= 0 ||
      !std::isfinite(config_.return_tolerance_m) || config_.return_tolerance_m <= 0 ||
      !config_.freshness_us || !config_.stable_us || !config_.loss_abort_us ||
      !finite(config_.approach) || !finite(config_.target)) throw std::invalid_argument("任务参数无效");
  for (size_t i = 0; i < n; ++i) {
    if (!finite(config_.homes[i]) || !finite(config_.assembly[i])) throw std::invalid_argument("航点无效");
    config_.assembly[i].z=config_.cruise_z_m;
  }
  for (const auto& p : config_.transit_waypoints) if (!finite(p)) throw std::invalid_argument("航路点无效");
  if (!config_.explicit_formation_offsets) {
    config_.formation_offsets.resize(n); double cx = 0, cy = 0;
    for (const auto& p : config_.homes) { cx += p.x / n; cy += p.y / n; }
    for (size_t i = 0; i < n; ++i) config_.formation_offsets[i] = {config_.homes[i].x-cx, config_.homes[i].y-cy, 0, 0};
  }
  if (config_.formation_offsets.size() != n) throw std::invalid_argument("编队偏移数量无效");
  for (const auto& p : config_.formation_offsets) if (!finite(p)) throw std::invalid_argument("编队偏移无效");
  status_.resize(n); seen_.assign(n, false); skipped_now_.assign(n, false); received_.assign(n, 0); loss_start_.assign(n, 0); holds_.resize(n);
  for (size_t i = 0; i < n; ++i) { holds_[i] = config_.homes[i]; holds_[i].z = config_.home_ground_z_m + config_.body_clearance_m; }
}

bool TeamMission::updateStatus(const TeamVehicleStatus& s, uint64_t wall) {
  if (s.vehicle < 0 || static_cast<size_t>(s.vehicle) >= status_.size()) return false;
  for (double v : {s.x_m,s.y_m,s.z_m,s.vx_mps,s.vy_mps,s.vz_mps,s.yaw_rad}) if (!std::isfinite(v)) return false;
  auto& old = status_[s.vehicle];
  if (seen_[s.vehicle] && (s.sim_time_us <= old.sim_time_us || s.impact_seq < old.impact_seq)) return false;
  old = s; seen_[s.vehicle] = true; received_[s.vehicle] = wall;
  if (skipped_now_[s.vehicle] && s.xy_valid && s.z_valid && s.ready &&
      phase_!=TeamPhase::COMPLETE && phase_!=TeamPhase::ABORTED) {
    const auto slot=std::find(config_.order.begin(),config_.order.end(),s.vehicle)-config_.order.begin();
    skipped_now_[s.vehicle]=false;
    loss_start_[s.vehicle]=0;
    if (slot>=turn_) {
      skipped_.erase(std::remove(skipped_.begin(),skipped_.end(),s.vehicle),skipped_.end());
      if (skipped_.empty()) reason_.clear();
    }
  }
  return true;
}
bool TeamMission::validForBarrier(int id, uint64_t sim, uint64_t wall) const {
  if (!seen_[id]) return false;
  const auto& s = status_[id];
  return s.xy_valid && s.z_valid && s.ready && sim >= s.sim_time_us && sim-s.sim_time_us <= config_.freshness_us &&
      wall >= received_[id] && wall-received_[id] <= config_.freshness_us &&
      (phase_ == TeamPhase::WAIT_READY || phase_ == TeamPhase::TAKEOFF || phase_ == TeamPhase::LAND_ALL || phase_==TeamPhase::COMPLETE || phase_==TeamPhase::ABORTED || s.armed);
}
bool TeamMission::near(int i, const TeamPoint& p, bool xy) const {
  if (!validForBarrier(i, clock_, wall_clock_)) return false;
  const auto& s = status_[i];
  return (!xy || std::hypot(s.x_m-p.x,s.y_m-p.y) <= config_.return_tolerance_m) &&
      std::abs(s.z_m-p.z) <= (phase_ == TeamPhase::LAND_ALL ? .035 : .08) &&
      (xy ? std::hypot(std::hypot(s.vx_mps,s.vy_mps),s.vz_mps) <= .30 : std::abs(s.vz_mps) <= .12);
}
bool TeamMission::barrier(bool ok, uint64_t now) { if (!ok) { stable_=false; return false; } if (!stable_) { stable_=true; stable_start_=now; } return now-stable_start_ >= config_.stable_us; }
void TeamMission::enter(TeamPhase p, uint64_t now) {
  phase_=p; phase_start_=now; stable_=false;
  if (p==TeamPhase::CONTACT && activeVehicle()>=0) impact_baseline_=status_[activeVehicle()].impact_seq;
  if (p==TeamPhase::RETREAT) retreat_assembly_=false;
  if (p==TeamPhase::RETURN_ALL || p==TeamPhase::ABORT_RETURN)
    return_index_=config_.transit_waypoints.empty()?-1:std::min(transit_index_-1,static_cast<int>(config_.transit_waypoints.size())-1);
}
void TeamMission::skipUnrecoverable(uint64_t wall) {
  if (std::none_of(seen_.begin(),seen_.end(),[](bool seen){return seen;})) return;
  for (size_t i=0;i<status_.size();++i) if (!skipped_now_[i] && loss_start_[i] && wall-loss_start_[i] >= config_.loss_abort_us) {
    skipped_now_[i]=true; skipped_.push_back(static_cast<int>(i)); reason_="车辆失联，任务以部分结果继续";
  }
}
void TeamMission::abort(uint64_t now, const char* r) { aborted_=true; reason_=r; enter(TeamPhase::ABORT_RETURN,now); }
int TeamMission::activeVehicle() const { return (phase_==TeamPhase::APPROACH || phase_==TeamPhase::CONTACT || phase_==TeamPhase::RETREAT) && turn_ < static_cast<int>(config_.order.size()) ? config_.order[turn_] : -1; }

std::vector<TeamCommand> TeamMission::step(uint64_t sim, uint64_t wall) {
  clock_=std::max(clock_,sim); sim=clock_;wall_clock_=wall;
  for (size_t i=0;i<status_.size();++i) { if (!validForBarrier(static_cast<int>(i),sim,wall)) { if (!loss_start_[i]) loss_start_[i]=wall; } else loss_start_[i]=0; }
  skipUnrecoverable(wall);
  const bool terminal=phase_==TeamPhase::COMPLETE||phase_==TeamPhase::ABORTED;
  auto allNear=[&](bool home, bool landing=false){bool any=false;for(size_t i=0;i<status_.size();++i)if(!skipped_now_[i]){any=true;auto p=home?config_.homes[i]:config_.assembly[i];p.z=home?(landing?config_.home_ground_z_m+config_.body_clearance_m:config_.cruise_z_m):config_.cruise_z_m;if(!near(static_cast<int>(i),p,home&&landing?false:true))return false;}return any;};
  auto routeNear=[&](int k){bool any=false;for(size_t i=0;i<status_.size();++i)if(!skipped_now_[i]){any=true;auto p=config_.transit_waypoints[k];p.x+=config_.formation_offsets[i].x;p.y+=config_.formation_offsets[i].y;if(!near(static_cast<int>(i),p))return false;}return any;};
  if (!terminal && !aborted_ && config_.stage_timeout_us && phase_!=TeamPhase::WAIT_READY && sim-phase_start_>=config_.stage_timeout_us) abort(sim,phase_==TeamPhase::CONTACT?"接触阶段超时":"阶段超时");
  const int previous_active=activeVehicle();
  while (turn_ < static_cast<int>(config_.order.size()) && skipped_now_[config_.order[turn_]]) ++turn_;
  if(previous_active>=0&&previous_active!=activeVehicle()) {
    enter(turn_==static_cast<int>(config_.order.size())?TeamPhase::RETURN_ALL:TeamPhase::APPROACH,sim);
  }
  if (!terminal) switch(phase_) {
    case TeamPhase::WAIT_READY: { bool ok=true, any=false; for(size_t i=0;i<status_.size();++i) { ok&=skipped_now_[i]||validForBarrier(static_cast<int>(i),sim,wall); any|=!skipped_now_[i]&&validForBarrier(static_cast<int>(i),sim,wall); } if(barrier(ok&&any,sim))enter(TeamPhase::TAKEOFF,sim); break; }
    case TeamPhase::TAKEOFF: { bool ok=true; for(size_t i=0;i<status_.size();++i)if(!skipped_now_[i]){auto p=config_.homes[i];p.z=config_.cruise_z_m;ok&=status_[i].armed&&near(static_cast<int>(i),p,false);} if(barrier(ok,sim))enter(config_.transit_waypoints.empty()?TeamPhase::ASSEMBLE:TeamPhase::TRANSIT,sim); break; }
    case TeamPhase::TRANSIT: if(transit_index_<static_cast<int>(config_.transit_waypoints.size())&&barrier(routeNear(transit_index_),sim)){if(++transit_index_==static_cast<int>(config_.transit_waypoints.size()))enter(TeamPhase::ASSEMBLE,sim); else { phase_start_=sim; stable_=false; }} break;
    case TeamPhase::ASSEMBLE: if(barrier(allNear(false),sim))enter(TeamPhase::APPROACH,sim); break;
    case TeamPhase::APPROACH: { const int a=activeVehicle(); if(a>=0&&!skipped_now_[a]&&barrier(near(a,config_.approach)&&std::abs(std::remainder(status_[a].yaw_rad-config_.approach.yaw,6.283185307))<=.15,sim))enter(TeamPhase::CONTACT,sim); break; }
    case TeamPhase::CONTACT: { const int a=activeVehicle(); if(a>=0&&!skipped_now_[a]&&validForBarrier(a,sim,wall)&&status_[a].impact_seq>impact_baseline_){impact_baseline_=status_[a].impact_seq;const auto&s=status_[a];if(std::hypot(std::hypot(s.x_m-config_.target.x,s.y_m-config_.target.y),s.z_m-config_.target.z)<=.35&&std::abs(std::remainder(s.yaw_rad-config_.approach.yaw,6.283185307))<=.3){completed_.push_back(a);enter(TeamPhase::RETREAT,sim);}} break; }
    case TeamPhase::RETREAT: { const int a=activeVehicle();if(a<0){enter(TeamPhase::RETURN_ALL,sim);break;}if(barrier(near(a,retreat_assembly_?config_.assembly[a]:config_.approach),sim)){if(!retreat_assembly_){retreat_assembly_=true;stable_=false;}else if(++turn_==static_cast<int>(config_.order.size()))enter(TeamPhase::RETURN_ALL,sim);else enter(TeamPhase::APPROACH,sim);} break; }
    case TeamPhase::RETURN_ALL: case TeamPhase::ABORT_RETURN: if(return_index_>=0){if(barrier(routeNear(return_index_),sim)){--return_index_;stable_=false;phase_start_=sim;}}else if(barrier(allNear(true),sim))enter(TeamPhase::LAND_ALL,sim); break;
    default: break;
  }
  if (phase_==TeamPhase::LAND_ALL) {
    bool land=true; for(size_t i=0;i<status_.size();++i) if(!skipped_now_[i]) land&=status_[i].landed;
    if (barrier(allNear(true,true)&&land,sim)) enter(aborted_||!skipped_.empty()?TeamPhase::ABORTED:TeamPhase::COMPLETE,sim);
  }
  std::vector<TeamCommand> out(status_.size()); ++sequence_;
  for(size_t i=0;i<status_.size();++i) {
    auto&c=out[i];c.vehicle=static_cast<int>(i);c.sequence=sequence_;c.sim_time_us=sim;
    c.phase=phase_;c.active_vehicle=activeVehicle();c.skipped=skipped_now_[i];
    c.degraded=!validForBarrier(static_cast<int>(i),sim,wall);
    if(c.degraded) {
      if(!seen_[i]) c.degraded_reason="等待车辆上线";
      else if(sim<status_[i].sim_time_us || sim-status_[i].sim_time_us>config_.freshness_us || wall<received_[i] || wall-received_[i]>config_.freshness_us) c.degraded_reason="车辆状态超时";
      else if(!status_[i].z_valid) c.degraded_reason="高度数据不可用";
      else if(!status_[i].xy_valid) c.degraded_reason="水平定位恢复中";
      else c.degraded_reason="飞控连接恢复中";
    }
    TeamPoint p=config_.assembly[i];
    const bool returning=phase_==TeamPhase::RETURN_ALL||phase_==TeamPhase::ABORT_RETURN;
    if(phase_==TeamPhase::WAIT_READY||phase_==TeamPhase::TAKEOFF||returning||phase_==TeamPhase::LAND_ALL||phase_==TeamPhase::COMPLETE||phase_==TeamPhase::ABORTED){
      p=config_.homes[i];
      p.z=(phase_==TeamPhase::WAIT_READY||phase_==TeamPhase::LAND_ALL||phase_==TeamPhase::COMPLETE||phase_==TeamPhase::ABORTED)?config_.home_ground_z_m+config_.body_clearance_m:config_.cruise_z_m;
    }
    const int route_index=phase_==TeamPhase::TRANSIT?transit_index_:(returning?return_index_:-1);
    if(route_index>=0&&route_index<static_cast<int>(config_.transit_waypoints.size())){
      p=config_.transit_waypoints[route_index];p.x+=config_.formation_offsets[i].x;p.y+=config_.formation_offsets[i].y;p.yaw=config_.homes[i].yaw;
    } else if(static_cast<int>(i)==c.active_vehicle) {
      p=phase_==TeamPhase::RETREAT&&retreat_assembly_?config_.assembly[i]:config_.approach;
    }
    if(c.degraded&&seen_[i])p=holds_[i];
    else if(seen_[i])holds_[i]={status_[i].x_m,status_[i].y_m,status_[i].z_m,status_[i].yaw_rad};
    c.target_x_m=p.x;c.target_y_m=p.y;c.target_z_m=p.z;c.target_yaw_rad=p.yaw;
    c.contact_speed_mps=(phase_==TeamPhase::CONTACT&&static_cast<int>(i)==c.active_vehicle&&!c.degraded)?config_.contact_speed_mps:0;
    c.disarm=(phase_==TeamPhase::ABORTED||phase_==TeamPhase::COMPLETE)&&status_[i].landed;
    c.complete=phase_==TeamPhase::COMPLETE;
  }
  return out;
}
}  // namespace companion
