/**
 * @file team_mission.cpp
 * @brief 多机任务状态机实现；只计算阶段和目标，不收网络、不驱动电机。
 * 核心模型：status_ 记录每机当前状态，phase_ 记录全队阶段，turn_ 指向接触顺序。
 * step() 先检查状态和失联，再判断阶段是否完成，最后给每机产生一份 TeamCommand。
 * 读代码时把“全队同步条件”和“当前机接触轮次”分开：并不是所有阶段都等所有飞机。
 */
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

// 初始化先校验成员数量和路径，再分配状态数组；避免把配置不一致当成飞行失联。
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
      !std::isfinite(config_.route_tolerance_m) || config_.route_tolerance_m<=0 ||
      !std::isfinite(config_.route_speed_tolerance_mps) || config_.route_speed_tolerance_mps<=0 ||
      !config_.freshness_us || !config_.stable_us || !config_.loss_abort_us ||
      !finite(config_.approach) || !finite(config_.target)) throw std::invalid_argument("任务参数无效");
  for (size_t i = 0; i < n; ++i) {
    if (!finite(config_.homes[i]) || !finite(config_.assembly[i])) throw std::invalid_argument("航点无效");
    config_.assembly[i].z=config_.cruise_z_m;
  }
  if(config_.completed_holding.empty())config_.completed_holding=config_.assembly;
  if(config_.completed_holding.size()!=n||!std::isfinite(config_.contact_exit_side))throw std::invalid_argument("退出集结点配置无效");
  for(auto&p:config_.completed_holding){if(!finite(p))throw std::invalid_argument("退出集结点无效");p.z=config_.cruise_z_m;}
  if(config_.dive_attack){
    if(!finite(config_.dive_entry)||
       !std::isfinite(config_.dive_entry_tolerance_m)||config_.dive_entry_tolerance_m<=0||
       !std::isfinite(config_.dive_entry_height_m)||config_.dive_entry_height_m<=0||
       !std::isfinite(config_.dive_entry_yaw_tolerance_rad)||config_.dive_entry_yaw_tolerance_rad<=0)
      throw std::invalid_argument("俯冲入口配置无效");
  }
  for (const auto& p : config_.transit_waypoints) if (!finite(p)) throw std::invalid_argument("航路点无效");
  // 未给编队偏移时，以起飞点质心为原点，保留各机原有相对位置。
  if (!config_.explicit_formation_offsets) {
    config_.formation_offsets.resize(n); double cx = 0, cy = 0;
    for (const auto& p : config_.homes) { cx += p.x / n; cy += p.y / n; }
    for (size_t i = 0; i < n; ++i) config_.formation_offsets[i] = {config_.homes[i].x-cx, config_.homes[i].y-cy, 0, 0};
  }
  if (config_.formation_offsets.size() != n) throw std::invalid_argument("编队偏移数量无效");
  for (const auto& p : config_.formation_offsets) if (!finite(p)) throw std::invalid_argument("编队偏移无效");
  status_.resize(n); seen_.assign(n, false); skipped_now_.assign(n, false); failed_.assign(n, false); received_.assign(n, 0); loss_start_.assign(n, 0); holds_.resize(n);
  for (size_t i = 0; i < n; ++i) { holds_[i] = config_.homes[i]; holds_[i].z = config_.home_ground_z_m + config_.body_clearance_m; }
}

// 同一机号只接受更新的状态。恢复不等于补做错过的接触：历史 skipped_ 可能保留。
bool TeamMission::updateStatus(const TeamVehicleStatus& s, uint64_t wall) {
  if (s.vehicle < 0 || static_cast<size_t>(s.vehicle) >= status_.size()) return false;
  for (double v : {s.x_m,s.y_m,s.z_m,s.vx_mps,s.vy_mps,s.vz_mps,s.yaw_rad}) if (!std::isfinite(v)) return false;
  auto& old = status_[s.vehicle];
  if (seen_[s.vehicle] && (s.sim_time_us <= old.sim_time_us || s.impact_seq < old.impact_seq)) return false;
  old = s; seen_[s.vehicle] = true; received_[s.vehicle] = wall;
  if (skipped_now_[s.vehicle] && !failed_[s.vehicle] && s.xy_valid && s.z_valid && s.ready &&
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
// 双时钟新鲜度检查：仿真时间验证测量年龄，现实单调时间验证是否还在收到包。
bool TeamMission::statusFresh(int id, uint64_t sim, uint64_t wall) const {
  if (!seen_[id]) return false;
  const auto& s = status_[id];
  return sim >= s.sim_time_us && sim-s.sim_time_us <= config_.freshness_us &&
      wall >= received_[id] && wall-received_[id] <= config_.freshness_us;
}
bool TeamMission::validForBarrier(int id, uint64_t sim, uint64_t wall) const {
  const auto& s = status_[id];
  return statusFresh(id,sim,wall) && s.xy_valid && s.z_valid && s.ready &&
      (phase_ == TeamPhase::WAIT_READY || phase_ == TeamPhase::TAKEOFF || phase_ == TeamPhase::LAND_ALL || phase_==TeamPhase::COMPLETE || phase_==TeamPhase::ABORTED || s.armed);
}
// “到点”同时看位置和速度，防止高速掠过被认为停稳。降落阶段还限制垂直速度。
bool TeamMission::near(int i, const TeamPoint& p, bool xy) const {
  if (!validForBarrier(i, clock_, wall_clock_)) return false;
  const auto& s = status_[i];
  return (!xy || std::hypot(s.x_m-p.x,s.y_m-p.y) <= config_.return_tolerance_m) &&
      std::abs(s.z_m-p.z) <= (phase_ == TeamPhase::LAND_ALL ? .035 : .08) &&
      (xy ? std::hypot(std::hypot(s.vx_mps,s.vy_mps),s.vz_mps) <= .30 : std::abs(s.vz_mps) <= .12);
}
// 同步屏障：条件连续成立 stable_us 才通过；中间一次不满足便重新计时。
bool TeamMission::barrier(bool ok, uint64_t now) { if (!ok) { stable_=false; return false; } if (!stable_) { stable_=true; stable_start_=now; } return now-stable_start_ >= config_.stable_us; }
// 统一阶段入口，清除上阶段稳定计时；接触阶段保存冲击序号基线，防止旧事件重用。
void TeamMission::enter(TeamPhase p, uint64_t now) {
  phase_=p; phase_start_=now; stable_=false;
  if((p==TeamPhase::COMPLETE||p==TeamPhase::ABORTED)&&flight_start_)flight_end_=now;
  if (p==TeamPhase::CONTACT && activeVehicle()>=0) impact_baseline_=status_[activeVehicle()].impact_seq;
  if (p==TeamPhase::RETREAT) retreat_assembly_=config_.retreat_holding_direct;
  if (p==TeamPhase::RETURN_ALL || p==TeamPhase::ABORT_RETURN)
    return_index_=config_.transit_waypoints.empty()?-1:std::min(transit_index_-1,static_cast<int>(config_.transit_waypoints.size())-1);
}
// 长时间无有效状态的成员暂不参与全队屏障；保留其失联记录，其他成员继续。
// 记录按机号去重：已经错过轮次的成员恢复后再次失联时只更新 skipped_now_，
// 不重复追加历史条目，否则 skipped_ 会随失联次数线性增长并淹没界面与日志。
void TeamMission::skipUnrecoverable(uint64_t wall) {
  if (std::none_of(seen_.begin(),seen_.end(),[](bool seen){return seen;})) return;
  for (size_t i=0;i<status_.size();++i) if (!skipped_now_[i] && loss_start_[i] && wall-loss_start_[i] >= config_.loss_abort_us) {
    skipped_now_[i]=true;
    const int id=static_cast<int>(i);
    if (std::find(skipped_.begin(),skipped_.end(),id)==skipped_.end()) skipped_.push_back(id);
    reason_="车辆失联，任务以部分结果继续";
  }
}
void TeamMission::abort(uint64_t now, const char* r) { aborted_=true; reason_=r; enter(TeamPhase::ABORT_RETURN,now); }
int TeamMission::activeVehicle() const { return (phase_==TeamPhase::APPROACH || phase_==TeamPhase::CONTACT || phase_==TeamPhase::RETREAT) && turn_ < static_cast<int>(config_.order.size()) ? config_.order[turn_] : -1; }
// 轮次只越过失效机和接触超时失败机；失败机仍参与返航、降落屏障。
bool TeamMission::turnExhausted(int vehicle) const {
  return vehicle<0||static_cast<size_t>(vehicle)>=failed_.size()?true:(skipped_now_[vehicle]||failed_[vehicle]);
}

/**
 * @brief 一次任务调度：评估成员 → 推进状态 → 输出目标。
 * @note 必须先调用 updateStatus 喂入收到的数据；本函数不会读取任何真实位置。
 */
std::vector<TeamCommand> TeamMission::step(uint64_t sim, uint64_t wall) {
  // 1. 更新时间和飞行计时；只用有效、已解锁的估计高度触发离地计时。
  clock_=std::max(clock_,sim); sim=clock_;wall_clock_=wall;
  if(!flight_start_)for(size_t i=0;i<status_.size();++i)
    if(validForBarrier(static_cast<int>(i),sim,wall)&&status_[i].armed&&status_[i].z_m>config_.home_ground_z_m+config_.body_clearance_m+.03){flight_start_=sim;break;}
  for (size_t i=0;i<status_.size();++i) { if (!validForBarrier(static_cast<int>(i),sim,wall)) { if (!loss_start_[i]) loss_start_[i]=wall; } else loss_start_[i]=0; }
  skipUnrecoverable(wall);
  // 接触失败的成员仍可参与返航；无法继续飞行的成员退出路线屏障，独立恢复。
  for (size_t i = 0; i < status_.size(); ++i) {
    if (!status_[i].unrecoverable) continue;
    // 待机、起飞、降落与终态里落地是正常状态，不能当失败。
    if (phase_ == TeamPhase::WAIT_READY || phase_ == TeamPhase::TAKEOFF ||
        phase_ == TeamPhase::LAND_ALL || phase_ == TeamPhase::COMPLETE ||
        phase_ == TeamPhase::ABORTED) continue;
    skipped_now_[i] = true;
    if (failed_[i]) continue;
    failed_[i] = true;
    const int id = static_cast<int>(i);
    if (std::find(skipped_.begin(), skipped_.end(), id) == skipped_.end()) skipped_.push_back(id);
    reason_ = "成员已落地且无法复飞，任务以部分结果继续";
  }
  const bool terminal=phase_==TeamPhase::COMPLETE||phase_==TeamPhase::ABORTED;
  // 2. 全队位置屏障跳过暂时失效成员，但不能让空队伍被当成全员到达。
  auto allNear=[&](bool home, bool landing=false){bool any=false;for(size_t i=0;i<status_.size();++i)if(!skipped_now_[i]){any=true;auto p=home?config_.homes[i]:config_.assembly[i];p.z=home?(landing?config_.home_ground_z_m+config_.body_clearance_m:config_.cruise_z_m):config_.cruise_z_m;if(!near(static_cast<int>(i),p,home&&landing?false:true))return false;}return any;};
  // 去程可穿越航点；返航各段先收敛位置和速度，避免带着侧向速度切角。
  auto routeNear=[&](int k){bool any=false;for(size_t i=0;i<status_.size();++i)if(!skipped_now_[i]){any=true;auto p=config_.transit_waypoints[k];p.x+=config_.formation_offsets[i].x;p.y+=config_.formation_offsets[i].y;
    if(!config_.fly_through_route||phase_!=TeamPhase::TRANSIT){if(!near(static_cast<int>(i),p))return false;}
    else {const auto&s=status_[i];if(!validForBarrier(static_cast<int>(i),sim,wall)||std::hypot(s.x_m-p.x,s.y_m-p.y)>config_.route_tolerance_m||std::abs(s.z_m-p.z)>.12||std::hypot(s.vx_mps,s.vy_mps)>config_.route_speed_tolerance_mps)return false;}
  }return any;};
  if (!terminal && !aborted_ && config_.stage_timeout_us && phase_!=TeamPhase::WAIT_READY && sim-phase_start_>=config_.stage_timeout_us) {
    if(phase_==TeamPhase::LAND_ALL){aborted_=true;reason_="降落阶段超时，等待着地停机反馈";}
    else abort(sim,phase_==TeamPhase::CONTACT?"接触阶段超时":"阶段超时");
  }
  // 3. 接触轮次越过失效机；如果当前机改变，需要切换接近阶段，不能沿用旧机的接触状态。
  const int previous_active=activeVehicle();
  while (turn_ < static_cast<int>(config_.order.size()) && turnExhausted(config_.order[turn_])) ++turn_;
  if(previous_active>=0&&previous_active!=activeVehicle()) {
    enter(turn_==static_cast<int>(config_.order.size())?TeamPhase::RETURN_ALL:TeamPhase::APPROACH,sim);
  }
  const bool all_unrecoverable=std::all_of(seen_.begin(),seen_.end(),[](bool value){return value;})&&
      std::all_of(status_.begin(),status_.end(),[](const auto& state){return state.unrecoverable;});
  if(!terminal&&all_unrecoverable&&phase_!=TeamPhase::WAIT_READY&&phase_!=TeamPhase::TAKEOFF){
    aborted_=true;
    if(phase_!=TeamPhase::LAND_ALL){reason_="全队无法继续任务，等待降落停机";enter(TeamPhase::LAND_ALL,sim);}
  }
  // 4. 每个 case 只负责阶段转移。目标点在下方统一生成，避免转移后仍发上阶段目标。
  if (!terminal) switch(phase_) {
    case TeamPhase::WAIT_READY: { bool ok=true, any=false; for(size_t i=0;i<status_.size();++i) { ok&=skipped_now_[i]||validForBarrier(static_cast<int>(i),sim,wall); any|=!skipped_now_[i]&&validForBarrier(static_cast<int>(i),sim,wall); } if(barrier(ok&&any,sim))enter(TeamPhase::TAKEOFF,sim); break; }
    case TeamPhase::TAKEOFF: { bool ok=true; for(size_t i=0;i<status_.size();++i)if(!skipped_now_[i]){auto p=config_.homes[i];p.z=config_.cruise_z_m;ok&=status_[i].armed&&near(static_cast<int>(i),p,false);} if(barrier(ok,sim))enter(config_.transit_waypoints.empty()?TeamPhase::ASSEMBLE:TeamPhase::TRANSIT,sim); break; }
    case TeamPhase::TRANSIT: if(transit_index_<static_cast<int>(config_.transit_waypoints.size())&&barrier(routeNear(transit_index_),sim)){if(++transit_index_==static_cast<int>(config_.transit_waypoints.size()))enter(TeamPhase::ASSEMBLE,sim); else { phase_start_=sim; stable_=false; }} break;
    case TeamPhase::ASSEMBLE: if(barrier(allNear(false),sim))enter(TeamPhase::APPROACH,sim); break;
    case TeamPhase::APPROACH: {
      const int a=activeVehicle();
      if(a<0||skipped_now_[a])break;
      // 接近超时：入口是固定航点，飞不到就不该让整队一直等下去。
      if(config_.approach_timeout_us&&sim-phase_start_>=config_.approach_timeout_us){
        if(!failed_[a]){
          failed_[a]=true;
          if(std::find(skipped_.begin(),skipped_.end(),a)==skipped_.end())skipped_.push_back(a);
          reason_="接近阶段超时，未到达俯冲入口";
        }
        while(turn_<static_cast<int>(config_.order.size())&&turnExhausted(config_.order[turn_]))++turn_;
        enter(turn_==static_cast<int>(config_.order.size())?TeamPhase::RETURN_ALL:TeamPhase::APPROACH,sim);
        break;
      }
      const auto&s=status_[a];
      bool ok;
      if(config_.dive_attack){
        // 俯冲模式：入口是一个「门」而不是停点，飞过去就进接触阶段，避免在板前减速停住。
        // 视觉提供落点；入口阶段只检查位置和航向，近距灯条可能离开视野。
        ok=validForBarrier(a,sim,wall)&&
           std::hypot(s.x_m-config_.dive_entry.x,s.y_m-config_.dive_entry.y)<=config_.dive_entry_tolerance_m&&
           std::abs(s.z_m-config_.dive_entry.z)<=config_.dive_entry_height_m&&
           std::abs(std::remainder(s.yaw_rad-config_.dive_entry.yaw,6.283185307))<=config_.dive_entry_yaw_tolerance_rad;
      }else if(config_.visual_contact){
        // 视觉模式：除了图像锁定，还要求已经贴近到搜索距离内。
        // 否则从集结距离一锁定就进接触，最后几米全部交给视觉伺服爬行，
        // 而视觉伺服的横向纠偏会来回摆动，比任务位置环慢得多。
        const double range=std::hypot(s.x_m-config_.target.x,s.y_m-config_.target.y);
        ok=validForBarrier(a,sim,wall)&&s.visual_ready&&
           (config_.visual_entry_range_m<=0||range<=config_.visual_entry_range_m);
      }else{
        ok=near(a,config_.approach)&&std::abs(std::remainder(s.yaw_rad-config_.approach.yaw,6.283185307))<=.15;
      }
      if(barrier(ok,sim))enter(TeamPhase::CONTACT,sim);
      break;
    }
    // 接触要有新的冲击序号，且位置/航向符合基地条件；仅距离近或别的机撞击不算。
    case TeamPhase::CONTACT: {
      const int a=activeVehicle();
      if(a<0)break;
      // 接触超时：压力门限让力度不足的接触不再算成功，因此必须给接触阶段一个上限。
      // 超时的机记入未完成并换下一架，不再让整队停在这一轮。
      if(config_.contact_timeout_us&&sim-phase_start_>=config_.contact_timeout_us){
        if(!failed_[a]){
          failed_[a]=true;
          if(std::find(skipped_.begin(),skipped_.end(),a)==skipped_.end())skipped_.push_back(a);
          reason_="接触阶段超时，未取得达标撞击";
        }
        while(turn_<static_cast<int>(config_.order.size())&&turnExhausted(config_.order[turn_]))++turn_;
        enter(turn_==static_cast<int>(config_.order.size())?TeamPhase::RETURN_ALL:TeamPhase::APPROACH,sim);
        break;
      }
      if(a>=0&&!skipped_now_[a]&&validForBarrier(a,sim,wall)&&status_[a].impact_seq>impact_baseline_){impact_baseline_=status_[a].impact_seq;const auto&s=status_[a];if(config_.visual_contact||(std::hypot(std::hypot(s.x_m-config_.target.x,s.y_m-config_.target.y),s.z_m-config_.target.z)<=.35&&std::abs(std::remainder(s.yaw_rad-config_.approach.yaw,6.283185307))<=.3)){completed_.push_back(a);enter(TeamPhase::RETREAT,sim);}} break; }
    case TeamPhase::RETREAT: { const int a=activeVehicle();if(a<0){enter(TeamPhase::RETURN_ALL,sim);break;}
      // 流水接触：将位置投影到接触航向的前向/侧向，确认通道让出后启动下一架。
      if(config_.pipelined_contact){const auto&s=status_[a];const double dx=s.x_m-config_.approach.x,dy=s.y_m-config_.approach.y;
        const double forward=dx*std::cos(config_.approach.yaw)+dy*std::sin(config_.approach.yaw);
        const double lateral=-dx*std::sin(config_.approach.yaw)+dy*std::cos(config_.approach.yaw);
        if(turn_+1==static_cast<int>(config_.order.size())&&validForBarrier(a,sim,wall)&&forward<-.12){++turn_;enter(TeamPhase::RETURN_ALL,sim);break;}
        if(validForBarrier(a,sim,wall)&&forward<-.25)retreat_assembly_=true;
        if(validForBarrier(a,sim,wall)&&retreat_assembly_&&forward<-.25&&std::abs(lateral)>.18){if(++turn_==static_cast<int>(config_.order.size()))enter(TeamPhase::RETURN_ALL,sim);else enter(TeamPhase::APPROACH,sim);}
      }else if(barrier(near(a,retreat_assembly_?config_.completed_holding[a]:config_.approach),sim)){if(!retreat_assembly_){retreat_assembly_=true;stable_=false;}else if(++turn_==static_cast<int>(config_.order.size()))enter(TeamPhase::RETURN_ALL,sim);else enter(TeamPhase::APPROACH,sim);} break; }
    case TeamPhase::RETURN_ALL: case TeamPhase::ABORT_RETURN: if(return_index_>=0){if(barrier(routeNear(return_index_),sim)){--return_index_;stable_=false;phase_start_=sim;}}else if(barrier(allNear(true),sim))enter(TeamPhase::LAND_ALL,sim); break;
    default: break;
  }
  // 5. 路线屏障可以跳过失效成员，降落停机确认必须覆盖每个成员。
  if (phase_==TeamPhase::LAND_ALL) {
    bool stopped=true;
    for(size_t i=0;i<status_.size();++i)
      stopped=stopped&&statusFresh(static_cast<int>(i),sim,wall)&&status_[i].landed&&!status_[i].armed;
    if(barrier(stopped&&(all_unrecoverable||allNear(true,true)),sim))
      enter(aborted_||!skipped_.empty()?TeamPhase::ABORTED:TeamPhase::COMPLETE,sim);
  }
  // 6. 给每架选择目标：待机点、航路点、接触点、退出点或本机起飞点。
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
    TeamPoint p=std::find(completed_.begin(),completed_.end(),static_cast<int>(i))!=completed_.end()?config_.completed_holding[i]:config_.assembly[i];
    const bool returning=phase_==TeamPhase::RETURN_ALL||phase_==TeamPhase::ABORT_RETURN;
    if(phase_==TeamPhase::WAIT_READY||phase_==TeamPhase::TAKEOFF||returning||phase_==TeamPhase::LAND_ALL||phase_==TeamPhase::COMPLETE||phase_==TeamPhase::ABORTED){
      p=config_.homes[i];
      p.z=(phase_==TeamPhase::WAIT_READY||phase_==TeamPhase::LAND_ALL||phase_==TeamPhase::COMPLETE||phase_==TeamPhase::ABORTED)?config_.home_ground_z_m+config_.body_clearance_m:config_.cruise_z_m;
    }
    const int route_index=phase_==TeamPhase::TRANSIT?transit_index_:(returning?return_index_:-1);
    if(route_index>=0&&route_index<static_cast<int>(config_.transit_waypoints.size())){
      p=config_.transit_waypoints[route_index];p.x+=config_.formation_offsets[i].x;p.y+=config_.formation_offsets[i].y;p.yaw=config_.homes[i].yaw;
    } else if(static_cast<int>(i)==c.active_vehicle) {
      // 俯冲模式在接近阶段把目标换成高处的入口；退出阶段仍回到接触前的低点让出通道。
      // 入口航点允许位置环减速完成爬升；目标设在入口之后会压缩可用爬升时间。
      const TeamPoint&staging=(config_.dive_attack&&phase_!=TeamPhase::RETREAT)?config_.dive_entry:config_.approach;
      p=phase_==TeamPhase::RETREAT&&retreat_assembly_?config_.completed_holding[i]:staging;
      if(phase_==TeamPhase::RETREAT&&config_.pipelined_contact&&!retreat_assembly_){p.x-=.45*std::cos(p.yaw);p.y-=.45*std::sin(p.yaw);}
    }
    // 下一架在侧方预备点等待，减少接近耗时；当前接触通道仍由 active_vehicle 独占。
    if(config_.pipelined_contact&&(phase_==TeamPhase::CONTACT||phase_==TeamPhase::RETREAT)&&
       turn_+1<static_cast<int>(config_.order.size())&&config_.order[turn_+1]==static_cast<int>(i)) {
      const double heading=config_.approach.yaw;
      const double side=(config_.assembly[i].y-config_.approach.y)*std::cos(heading)-(config_.assembly[i].x-config_.approach.x)*std::sin(heading);
      const double lateral=config_.contact_exit_side!=0?-std::copysign(.25,config_.contact_exit_side):(side>=0?.25:-.25);
      p=config_.approach;p.x-=.15*std::cos(heading)+lateral*std::sin(heading);p.y+=lateral*std::cos(heading)-.15*std::sin(heading);
    }
    // 单机失效只覆盖该机目标；保持点来自上次有效估计，不能用失效状态不断刷新。
    if(c.degraded&&seen_[i])p=holds_[i];
    else if(seen_[i])holds_[i]={status_[i].x_m,status_[i].y_m,status_[i].z_m,status_[i].yaw_rad};
    c.target_x_m=p.x;c.target_y_m=p.y;c.target_z_m=p.z;c.target_yaw_rad=p.yaw;
    c.contact_speed_mps=(phase_==TeamPhase::CONTACT&&static_cast<int>(i)==c.active_vehicle&&!c.degraded)?config_.contact_speed_mps:0;
    auto home=config_.homes[i];home.z=config_.home_ground_z_m+config_.body_clearance_m;
    const bool landing_disarm=phase_==TeamPhase::LAND_ALL&&(skipped_now_[i]||near(static_cast<int>(i),home,false));
    c.disarm=(landing_disarm||phase_==TeamPhase::ABORTED||phase_==TeamPhase::COMPLETE)&&
        statusFresh(static_cast<int>(i),sim,wall)&&status_[i].landed;
    c.complete=phase_==TeamPhase::COMPLETE;
  }
  return out;
}
}  // namespace companion
