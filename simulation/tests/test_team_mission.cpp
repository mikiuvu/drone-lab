#include "companion/team_mission.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <utility>
#include <vector>
using namespace companion;
namespace {
void check(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
TeamMissionConfig config(bool transit = false, double visual_entry_range = -1.0, bool dive = false,
                         bool direct_retreat = false, uint64_t stage_timeout = 0, bool sequential_landing = false) {
  TeamMissionConfig c; c.stable_us = 200000;
  c.sequential_landing = sequential_landing;
  for (int i = 0; i < 4; ++i) {
    c.homes[i] = {double(i)-1.5,0,.5125,0};
    c.assembly[i] = {double(i)-1.5,4,1.2,0};
  }
  c.approach = {0,5,1.2,0}; c.target = {1,5,1.2,0};
  if (transit) c.transit_waypoints = {{0,1,1.2,0},{0,2,1.4,0}};
  // 负值表示不启用视觉接触；0 表示视觉接触但不设进入距离门。
  if (visual_entry_range >= 0.0) {
    c.visual_contact = true; c.visual_entry_range_m = visual_entry_range;
  }
  if (dive) {
    c.dive_attack = true;
    c.dive_entry = {0,4.5,1.7,0};
    c.contact_timeout_us = 500000;
    // 接近超时取得比接触超时用例的观察窗口长，避免两个超时互相干扰。
    c.approach_timeout_us = 5000000;
  }
  c.retreat_holding_direct = direct_retreat;
  c.stage_timeout_us = stage_timeout;
  return c;
}
struct Fixture {
  TeamMissionConfig c; TeamMission mission;
  std::vector<TeamVehicleStatus> states{4};
  uint64_t now = 1000000, wall = 1000000;
  std::vector<TeamCommand> commands{4};
  explicit Fixture(bool transit = false, double visual_entry_range = -1.0, bool dive = false,
                   bool direct_retreat = false, uint64_t stage_timeout = 0, bool sequential_landing = false)
      : Fixture(config(transit, visual_entry_range, dive, direct_retreat, stage_timeout, sequential_landing)) {}
  explicit Fixture(TeamMissionConfig custom):c(std::move(custom)),mission(c) {
    for (int i = 0; i < 4; ++i) {
      states[i].vehicle = i; states[i].xy_valid = states[i].z_valid = states[i].ready = true;
      set(i,c.homes[i]);
    }
  }
  void set(int id, TeamPoint p) {
    states[id].x_m = p.x; states[id].y_m = p.y; states[id].z_m = p.z; states[id].yaw_rad = p.yaw;
  }
  void tick(int omitted = -1, uint64_t dt = 100000) {
    now += dt; wall += dt;
    for (int i = 0; i < 4; ++i) if (i != omitted) {
      states[i].sim_time_us = now; check(mission.updateStatus(states[i],wall),"有效状态被拒绝");
    }
    commands = mission.step(now,wall);
  }
  void settle(int omitted = -1) { for (int j = 0; j < 4; ++j) tick(omitted); }
  void follow() {
    for (int i = 0; i < 4; ++i)
      set(i,{commands[i].target_x_m,commands[i].target_y_m,commands[i].target_z_m,commands[i].target_yaw_rad});
  }
  void takeoff() {
    settle(); check(mission.phase() == TeamPhase::TAKEOFF,"就绪阶段未完成");
    for (int i = 0; i < 4; ++i) { states[i].armed = true; auto p = c.homes[i]; p.z = c.cruise_z_m; set(i,p); }
    settle();
  }
  void contact() {
    takeoff(); check(mission.phase() == TeamPhase::ASSEMBLE,"起飞阶段未完成");
    for (int i = 0; i < 4; ++i) set(i,c.assembly[i]);
    settle(); check(mission.phase() == TeamPhase::APPROACH,"集合阶段未完成");
    set(0,c.approach); settle(); check(mission.phase() == TeamPhase::CONTACT,"接近阶段未完成");
  }
};
void barriers() {
  Fixture f; f.states[3].ready = false; f.settle();
  check(f.mission.phase() == TeamPhase::WAIT_READY,"缺失就绪机仍然起飞");
  f.states[3].ready = true; f.takeoff();
  for (int i = 0; i < 3; ++i) f.set(i,f.c.assembly[i]);
  f.settle(); check(f.mission.phase() == TeamPhase::ASSEMBLE,"未全队集合仍然接近");
  f.set(3,f.c.assembly[3]); f.settle();
  check(f.mission.phase() == TeamPhase::APPROACH,"集合屏障失败");
  Fixture h; h.settle();
  for (int i = 0; i < 3; ++i) { h.states[i].armed = true; h.states[i].z_m = 1.2; }
  h.settle(); check(h.mission.phase() == TeamPhase::TAKEOFF,"未全队起飞仍然集合");
  h.states[3].armed = true; h.states[3].z_m = 1.2; h.settle();
  check(h.mission.phase() == TeamPhase::ASSEMBLE,"高度屏障失败");
}
void physicalContactAndTimeout() {
  Fixture f; f.states[0].impact_seq = 4; f.contact();
  f.set(0,f.c.target); f.states[1].impact_seq = 1; f.settle();
  check(f.mission.phase() == TeamPhase::CONTACT && f.mission.completed().empty(),"距离或其他机碰撞被计为成功");
  for (int i = 0; i < 610; ++i) f.tick();
  check(f.mission.phase() == TeamPhase::CONTACT && !f.mission.aborted() &&
        f.mission.completed().empty(),"健康车辆被默认阶段期限中止或未接触却成功");
}
void contactDeadlineRemainsFixed() {
  for (const auto limits : {std::pair<uint64_t,uint64_t>{3000000,0}, {0,2000000}, {3000000,2000000}, {0,0}}) {
    Fixture f;
    f.c.contact_timeout_us=limits.first;
    f.c.stage_timeout_us=limits.second;
    f.mission=TeamMission(f.c);
    f.contact();
    const auto deadline=f.commands[0].contact_deadline_us;
    const auto limit=limits.first&&limits.second?std::min(limits.first,limits.second):std::max(limits.first,limits.second);
    check(limit?(deadline>f.now&&deadline-f.now<=limit):deadline==0,"接触截止时间未取最早期限");
    f.tick();check(f.commands[0].contact_deadline_us==deadline,"周期下发延长了接触期限");
    f.set(0,f.c.target);++f.states[0].impact_seq;f.tick();
    check(f.mission.phase()==TeamPhase::RETREAT&&f.commands[0].contact_deadline_us==0,"退出接触后保留了旧期限");
  }
}
void sequentialAndLanding() {
  Fixture f; f.contact();
  for (int active = 0; active < 4; ++active) {
    check(f.mission.activeVehicle() == active,"轮次机号错误");
    f.set(active,f.c.target);
    ++f.states[active].impact_seq; f.tick();
    check(f.mission.phase() == TeamPhase::RETREAT && f.mission.completed().size() == size_t(active+1),"未按物理碰撞退回");
    f.tick(); check(f.mission.completed().size() == size_t(active+1),"重复碰撞重复计数");
    f.set(active,f.c.approach); f.settle(); f.set(active,f.c.assembly[active]); f.settle();
    if (active < 3) {
      check(f.mission.phase() == TeamPhase::APPROACH,"提前返航");
      f.set(active+1,f.c.approach); f.settle();
    }
  }
  check(f.mission.phase() == TeamPhase::RETURN_ALL,"四次接触后未返航");
  for (int i = 0; i < 3; ++i) { auto p = f.c.homes[i]; p.z = 1.2; f.set(i,p); }
  f.settle(); check(f.mission.phase() == TeamPhase::RETURN_ALL,"未全队归位仍然降落");
  auto p = f.c.homes[3]; p.z = 1.2; f.set(3,p); f.settle();
  check(f.mission.phase() == TeamPhase::LAND_ALL,"归位后未降落");
  f.follow();
  for (auto& s : f.states) { s.z_m += .03; s.landed = false; }
  f.settle();
  check(f.mission.phase() == TeamPhase::LAND_ALL,"地面上方三厘米悬停被计为着陆");
  for (const auto& command : f.commands) check(!command.disarm && !command.complete,"未报告着陆仍然解除武装");
  f.follow();
  for (auto& s : f.states) s.landed = true;
  f.states[3].landed = false; f.settle();
  check(f.mission.phase() == TeamPhase::LAND_ALL,"未全队报告着陆仍然完成");
  f.states[3].landed = true; f.states[3].vz_mps = .3; f.settle();
  check(f.mission.phase() == TeamPhase::LAND_ALL,"高速下降被计为着陆");
  f.states[3].vz_mps = 0;
  f.states[0].x_m += .25;
  f.settle();
  check(f.mission.phase() == TeamPhase::LAND_ALL,"仍解锁的全队提前进入终态");
  for (const auto& command : f.commands) check(command.disarm && !command.complete,"降落阶段未请求停机");
  for (auto& state : f.states) state.armed = false;
  f.settle();
  check(f.mission.phase() == TeamPhase::COMPLETE,"着陆完成失败");
  for (const auto& command : f.commands) check(command.disarm && command.complete,"完成后未解除武装");
}
void staleAndAbort() {
  Fixture f; f.contact();
  for (int i=0;i<12;++i)f.tick(3);
  check(!f.mission.paused()&&f.commands[3].degraded,"失联未限定到单机");
  check(!f.commands[0].degraded&&f.commands[0].contact_speed_mps>0,"正常机被无关失联阻断");
  const double hold=f.commands[3].target_x_m;
  for(int i=0;i<10;++i)f.tick(3);
  check(f.commands[3].target_x_m==hold,"失联机恢复点漂移");
  f.tick();check(!f.commands[3].degraded,"恢复机仍然被阻断");
  for(int i=0;i<65;++i)f.tick(3);
  check(f.mission.partial()&&!f.mission.aborted()&&f.mission.skipped().size()==1,"长期失联没有记录部分任务");
  f.tick();check(!f.commands[3].degraded,"失联恢复后不能继续接收指令");
  check(f.mission.completed().empty(),"失联被计为撞击成功");
  Fixture active_loss;active_loss.contact();
  for(int i=0;i<65;++i)active_loss.tick(0);
  check(active_loss.mission.activeVehicle()==1&&active_loss.mission.partial(),"失联机未退出当前轮次");
  active_loss.tick();
  check(!active_loss.commands[0].skipped&&!active_loss.commands[0].degraded&&active_loss.mission.partial(),"错过轮次的恢复机未重新加入队伍或丢失部分结果记录");
  Fixture frozen;frozen.contact();frozen.wall+=2000000;
  frozen.commands=frozen.mission.step(frozen.now,frozen.wall);
  for(const auto&c:frozen.commands)check(c.degraded&&c.contact_speed_mps==0,"冻结时钟仍执行接触");
}
void transitAndReverse() {
  Fixture f(true); f.takeoff();
  check(f.mission.phase() == TeamPhase::TRANSIT,"未进入编队航路");
  check(std::abs(f.commands[0].target_x_m+1.5) < 1e-9 && f.commands[0].target_y_m == 1,"默认编队偏移错误");
  f.follow(); f.states[3].y_m = 0; f.settle();
  check(f.commands[0].target_y_m == 1,"航路未等待全队");
  f.follow(); f.settle(); check(f.commands[0].target_y_m == 2,"航路顺序错误");
  f.follow(); f.settle(); check(f.mission.phase() == TeamPhase::ASSEMBLE,"航路完成未集合");
  f.follow(); f.settle();
  for (int i = 0; i < 4; ++i) {
    f.set(i,f.c.approach); f.settle(); f.set(i,f.c.target); ++f.states[i].impact_seq; f.tick();
    f.set(i,f.c.approach); f.settle(); f.set(i,f.c.assembly[i]); f.settle();
  }
  check(f.mission.phase() == TeamPhase::RETURN_ALL && f.commands[0].target_y_m == 2,"未从最后航路点返航");
  f.follow(); f.settle(); check(f.commands[0].target_y_m == 1,"未逆序返航");
  f.follow(); f.settle(); check(f.commands[0].target_y_m == 0,"逆序航路后未回起点");
}
void inputGates() {
  Fixture f; auto s = f.states[0]; s.sim_time_us = 1;
  check(f.mission.updateStatus(s,1),"首包拒绝");
  check(!f.mission.updateStatus(s,2),"重复时间戳接受");
  s.sim_time_us = 2; s.vehicle = 4; check(!f.mission.updateStatus(s,2),"越界机号接受");
  s.vehicle = 0; s.x_m = std::nan(""); check(!f.mission.updateStatus(s,2),"非有限数接受");
  auto c = config(); c.order = {{0,0,2,3}}; bool rejected = false;
  try { TeamMission invalid(c); } catch (const std::invalid_argument&) { rejected = true; }
  check(rejected,"重复轮次接受");
}
void contactGates() {
  Fixture f; f.contact();
  ++f.states[0].impact_seq; f.tick();
  check(f.mission.completed().empty(),"远离目标的碰撞被接受");
  f.set(0,f.c.target); f.tick();
  check(f.mission.completed().empty(),"已拒绝碰撞在接近后被重复接受");
  f.states[0].yaw_rad = .4; ++f.states[0].impact_seq; f.tick();
  check(f.mission.completed().empty(),"偏离航向的碰撞被接受");
  f.states[0].yaw_rad = 0; ++f.states[0].impact_seq; f.tick();
  check(f.mission.phase() == TeamPhase::RETREAT,"零速度物理碰撞未被接受");
}
// 已经错过轮次的成员在恢复后再次失联时，历史记录只能保留一条。
// 若记录不去重，skipped_ 会随「失联—恢复」次数线性增长，界面与报告里会出现几百个重复机号。
void repeatedLossKeepsOneRecord() {
  Fixture f; f.contact();
  // 先让机 0 完成撞击与退出，使 turn_ 越过它；此后恢复不再清除历史记录
  // （updateStatus 只在 slot >= turn_ 时擦除，正是重复累积的入口）。
  f.set(0,f.c.target); ++f.states[0].impact_seq; f.tick();
  check(f.mission.phase() == TeamPhase::RETREAT,"撞击后未进入退出阶段");
  f.set(0,f.c.approach); f.settle();
  f.set(0,f.c.assembly[0]); f.settle();
  check(f.mission.phase() == TeamPhase::APPROACH && f.mission.activeVehicle() == 1,
        "机 0 退出后未进入下一轮次");
  for (int cycle = 0; cycle < 3; ++cycle) {
    for (int i = 0; i < 65; ++i) f.tick(0);
    check(f.mission.partial(),"长期失联没有记录部分任务");
    check(f.mission.skipped().size() == 1,"失联记录按次重复累积");
    // 恢复：重新喂入机 0 的状态，它会被重新纳入全队屏障。
    f.tick();
    check(!f.commands[0].skipped && !f.commands[0].degraded,"恢复机仍然被排除在全队屏障之外");
  }
  check(f.mission.skipped().size() == 1,"失联记录按次重复累积");
  check(f.mission.skipped()[0] == 0,"失联记录指向了错误的机号");
}
// 视觉模式的距离门：图像已锁定但仍在远处时不得进入接触阶段。
void visualApproachRangeGate() {
  Fixture f(false, 1.0);
  f.takeoff();
  check(f.mission.phase() == TeamPhase::ASSEMBLE,"视觉用例：起飞阶段未完成");
  for (int i = 0; i < 4; ++i) f.set(i,f.c.assembly[i]);
  f.settle();
  check(f.mission.phase() == TeamPhase::APPROACH,"视觉用例：集合阶段未完成");
  // 集结点距目标约 2.7 m，远超 1.0 m 的进入距离：锁定也不应进入接触。
  for (auto& s : f.states) s.visual_ready = true;
  f.settle();
  check(f.mission.phase() == TeamPhase::APPROACH,"图像在远处锁定就进入了接触阶段");
  // 推进到搜索点，距目标 1.0 m，恰好落在门限内。
  f.set(0,f.c.approach);
  f.settle();
  check(f.mission.phase() == TeamPhase::CONTACT,"进入距离内仍未进入接触阶段");
}
// 不配置进入距离时，只要求图像锁定即可进入接触阶段。
void visualRangeGateDisabledWithoutSetting() {
  Fixture f(false, 0.0);
  f.takeoff();
  for (int i = 0; i < 4; ++i) f.set(i,f.c.assembly[i]);
  f.settle();
  check(f.mission.phase() == TeamPhase::APPROACH,"无距离门用例：集合阶段未完成");
  for (auto& s : f.states) s.visual_ready = true;
  f.settle();
  check(f.mission.phase() == TeamPhase::CONTACT,"未配置进入距离时锁定期望直接进入接触");
}
// 视觉模式不再要求到达搜索点：位置条件已由距离门替代，停在原地也应能推进。
void visualGateIgnoresApproachYaw() {
  Fixture f(false, 1.0);
  f.takeoff();
  for (int i = 0; i < 4; ++i) f.set(i,f.c.assembly[i]);
  f.settle();
  for (auto& s : f.states) s.visual_ready = true;
  f.set(0,f.c.approach);
  f.states[0].yaw_rad = 1.4;  // 远离搜索点航向
  f.settle();
  check(f.mission.phase() == TeamPhase::CONTACT,"视觉模式下仍被搜索点航向卡住");
}
// 俯冲模式：接近阶段的目标换成高处的俯冲入口，飞过入口就进入接触阶段。
// 入口只按位置和航向判定，不要求图像锁定：近距离视觉失锁是必然的，
// 把它当门限会让任务卡在接近阶段，而不是去撞板。
void diveAttackFliesThroughEntryGate() {
  Fixture f(false, -1.0, true);
  f.takeoff();
  for (int i = 0; i < 4; ++i) f.set(i,f.c.assembly[i]);
  f.settle();
  check(f.mission.phase() == TeamPhase::APPROACH,"俯冲用例：集合阶段未完成");
  check(std::abs(f.commands[0].target_z_m - f.c.dive_entry.z) < 1e-9,
        "接近阶段没有瞄准俯冲入口高度");
  check(std::abs(f.commands[0].target_y_m - f.c.dive_entry.y) < 1e-9,
        "接近阶段没有瞄准俯冲入口位置");
  f.settle();
  check(f.mission.phase() == TeamPhase::APPROACH,"没到入口就进入接触阶段");
  f.set(0,f.c.dive_entry);
  f.settle();
  check(f.mission.phase() == TeamPhase::CONTACT,"到达俯冲入口后未进入接触阶段");
}
// 入口航向偏差过大时不得进入：机体系前向与俯冲方向不一致，速度下限会作用在错误方向。
void diveEntryRejectsMisalignedHeading() {
  Fixture f(false, -1.0, true);
  f.takeoff();
  for (int i = 0; i < 4; ++i) f.set(i,f.c.assembly[i]);
  f.settle();
  f.set(0,f.c.dive_entry); f.states[0].yaw_rad = 1.0; f.settle();
  check(f.mission.phase() == TeamPhase::APPROACH,"入口航向偏差过大仍进入接触阶段");
  f.states[0].yaw_rad = 0; f.settle();
  check(f.mission.phase() == TeamPhase::CONTACT,"航向对齐后未进入接触阶段");
}
// 接触超时机退出接触轮次，仍参与返航和降落屏障。
void contactTimeoutSkipsFailedVehicle() {
  Fixture f(false, -1.0, true);
  f.takeoff();
  for (int i = 0; i < 4; ++i) f.set(i,f.c.assembly[i]);
  f.settle();
  f.set(0,f.c.dive_entry); f.settle();
  check(f.mission.phase() == TeamPhase::CONTACT && f.mission.activeVehicle() == 0,
        "俯冲用例：未进入接触阶段");
  for (int i = 0; i < 8; ++i) f.tick();
  check(f.mission.activeVehicle() == 1,"接触超时后未换下一架");
  check(f.mission.partial(),"接触超时未记录部分结果");
  check(f.mission.completed().empty(),"接触超时被计为撞击成功");
  check(f.mission.skipped().size() == 1 && f.mission.skipped()[0] == 0,
        "接触超时没有记录正确的机号");
  // 失败的机 0 仍然接到返航目标，没有被排除在全队屏障之外。
  check(std::abs(f.commands[0].target_y_m - f.c.assembly[0].y) < 1e-9,
        "接触超时机被排除在返航之外");
  // 持续上报健康状态不会让它重新加入接触轮次。
  for (int i = 0; i < 10; ++i) f.tick();
  check(f.mission.activeVehicle() == 1,"接触超时机重新加入了接触轮次");
  // 后续若被判定无法继续飞行，则退出路线屏障并独立恢复。
  check(!f.commands[0].skipped,"接触超时机本应仍参与屏障");
  f.states[0].unrecoverable = true;
  f.settle();
  check(f.commands[0].skipped,"趴窝的失败机没有退出全队屏障");
}
// 接近超时：入口是固定航点，飞不到就不该让整队一直等下去。
void approachTimeoutSkipsVehicleThatCannotReachEntry() {
  Fixture f(false, -1.0, true);
  f.takeoff();
  for (int i = 0; i < 4; ++i) f.set(i,f.c.assembly[i]);
  f.settle();
  check(f.mission.phase() == TeamPhase::APPROACH,"接近超时用例：集合阶段未完成");
  // 机 0 一直停在集结点，到不了入口。
  for (int i = 0; i < 55; ++i) f.tick();
  check(f.mission.activeVehicle() == 1,"接近超时后未换下一架");
  check(f.mission.partial(),"接近超时未记录部分结果");
  check(f.mission.completed().empty(),"接近超时被计为撞击成功");
  check(f.mission.skipped().size() == 1 && f.mission.skipped()[0] == 0,
        "接近超时没有记录正确的机号");
  check(std::abs(f.commands[0].target_y_m - f.c.assembly[0].y) < 1e-9,
        "接近超时机被排除在返航之外");
}
// 直接退避配置将撞击后的目标设为停留点，省去接触前低点的中间停留。
void retreatHoldingDirect() {
  Fixture slow(false, -1.0, false, false);
  slow.contact();
  slow.set(0,slow.c.target); ++slow.states[0].impact_seq; slow.tick();
  check(slow.mission.phase() == TeamPhase::RETREAT,"撞击后未进入退出阶段");
  check(std::abs(slow.commands[0].target_y_m - slow.c.approach.y) < 1e-9,
        "默认退避没有先回到接触前低点");

  Fixture f(false, -1.0, false, true);
  f.contact();
  f.set(0,f.c.target); ++f.states[0].impact_seq; f.tick();
  check(f.mission.phase() == TeamPhase::RETREAT,"撞击后未进入退出阶段");
  check(std::abs(f.commands[0].target_y_m - f.c.assembly[0].y) < 1e-9,
        "开启直接退避后仍然先回到接触前低点");
  check(std::abs(f.commands[0].target_y_m - f.c.approach.y) > 1e-6,
        "直接退避的目标与接触前低点重合，用例失去意义");
  // 到达停留点后直接换下一架，不再多停一次。
  f.set(0,f.c.assembly[0]); f.settle();
  check(f.mission.phase() == TeamPhase::APPROACH && f.mission.activeVehicle() == 1,
        "到达停留点后未进入下一架");
}
void retreatUsesSeparateSideExitBeforeHolding() {
  auto c=config(false,-1,false,true);
  c.retreat_exit_enabled=true;c.retreat_exit={2,6,c.cruise_z_m,0};
  Fixture f(c);f.contact();
  f.set(0,f.c.target);++f.states[0].impact_seq;f.tick();
  check(f.commands[0].target_x_m==c.retreat_exit.x&&f.commands[0].target_y_m==c.retreat_exit.y,
        "侧方退出未先经过独立航点");
  f.set(0,f.c.assembly[0]);f.settle();
  check(f.mission.phase()==TeamPhase::RETREAT,"未经过侧方航点便完成退出");
  f.set(0,c.retreat_exit);f.settle();
  check(f.commands[0].target_y_m==f.c.assembly[0].y,"到达侧方航点后未转向本机待命位置");
  f.set(0,f.c.assembly[0]);f.settle();
  check(f.mission.phase()==TeamPhase::APPROACH&&f.mission.activeVehicle()==1,"待命点确认后未轮换");
}
}
// 趴窝的成员要立刻记失败并越过，不能等接触超时干等。
void landedVehicleFailsImmediately() {
  Fixture f(false, -1.0, true);
  f.takeoff();
  for (int i = 0; i < 4; ++i) f.set(i,f.c.assembly[i]);
  f.settle();
  f.set(0,f.c.dive_entry);
  // 只推进到刚进入接触阶段，确保下面的单拍远在接触超时（0.5 s）之内。
  for (int i = 0; i < 4 && f.mission.phase() != TeamPhase::CONTACT; ++i) f.tick();
  check(f.mission.phase() == TeamPhase::CONTACT && f.mission.activeVehicle() == 0,
        "趴窝用例：未进入接触阶段");
  f.states[0].unrecoverable = true;
  f.tick();
  check(f.mission.activeVehicle() == 1,"趴窝成员未被立刻越过");
  check(f.mission.partial(),"趴窝成员未记入部分结果");
  check(f.mission.skipped().size() == 1 && f.mission.skipped()[0] == 0,
        "趴窝成员没有记录正确的机号");
}
// 起飞与降落阶段落地是正常状态，不能把上报的落地当成趴窝失败。
void landedDuringTakeoffIsNotFailure() {
  Fixture f(false, -1.0, true);
  f.states[0].unrecoverable = true;
  f.settle();
  check(f.mission.phase() == TeamPhase::TAKEOFF,"落地误判：就绪阶段未完成");
  check(f.mission.skipped().empty(),"起飞阶段的落地被误判为趴窝失败");
}
void allUnrecoverableWaitsForFreshLandingAndDisarm() {
  Fixture f(true);
  f.takeoff();
  check(f.mission.phase()==TeamPhase::TRANSIT,"全队恢复用例未进入航路阶段");
  for(auto& state:f.states)state.unrecoverable=true;
  f.tick();
  check(f.mission.phase()==TeamPhase::LAND_ALL,"全队失效后仍等待航路屏障");
  check(f.mission.partial()&&f.mission.skipped().size()==4,"全队失效没有保留失败记录");
  for(auto& state:f.states){
    state.xy_valid=false;state.ready=false;state.landed=true;state.armed=false;
    state.x_m=20;state.y_m=-8;state.z_m=.25;
  }
  f.states[0].armed=true;
  f.settle();
  check(f.mission.phase()==TeamPhase::LAND_ALL,"仍解锁的成员被当成恢复完成");
  f.states[0].armed=false;f.states[0].landed=false;
  f.settle();
  check(f.mission.phase()==TeamPhase::LAND_ALL,"未确认着地的成员被当成恢复完成");
  f.states[0].landed=true;
  f.tick(0,f.c.freshness_us+1);
  check(f.mission.phase()==TeamPhase::LAND_ALL,"陈旧的停机反馈被当成恢复完成");
  f.settle();
  check(f.mission.phase()==TeamPhase::ABORTED,"全队降落停机后未进入失败终态");
  for(const auto& command:f.commands)check(command.phase==TeamPhase::ABORTED&&!command.complete&&command.disarm,"失败终态的停机命令不正确");
}
void partialLandingWaitsForSkippedMember() {
  Fixture f;f.contact();
  f.states[0].unrecoverable=true;f.tick();
  for(int active=1;active<4;++active){
    f.set(active,f.c.approach);f.settle();
    f.set(active,f.c.target);++f.states[active].impact_seq;f.tick();
    f.set(active,f.c.approach);f.settle();
    f.set(active,f.c.assembly[active]);f.settle();
  }
  check(f.mission.phase()==TeamPhase::RETURN_ALL,"部分降落用例未返航");
  for(int i=1;i<4;++i){auto point=f.c.homes[i];point.z=f.c.cruise_z_m;f.set(i,point);}
  f.settle();check(f.mission.phase()==TeamPhase::LAND_ALL,"部分降落用例未进入降落阶段");
  for(int i=1;i<4;++i){auto point=f.c.homes[i];point.z=f.c.home_ground_z_m+f.c.body_clearance_m;f.set(i,point);f.states[i].landed=true;f.states[i].armed=false;}
  f.states[0].xy_valid=false;f.states[0].ready=false;
  f.settle();check(f.mission.phase()==TeamPhase::LAND_ALL,"跳过的未着地成员被排除在终态检查之外");
  check(!f.commands[0].disarm,"未着地的跳过成员收到停机请求");
  f.states[0].landed=true;f.settle();
  check(f.mission.phase()==TeamPhase::LAND_ALL&&f.commands[0].disarm,"跳过成员仍解锁时提前终止或未请求停机");
  f.states[0].armed=false;
  f.tick(0,f.c.freshness_us+1);
  check(f.mission.phase()==TeamPhase::LAND_ALL&&!f.commands[0].disarm,"陈旧反馈推进终态或触发停机");
  f.settle();check(f.mission.phase()==TeamPhase::ABORTED,"跳过成员停机后未进入失败终态");
  for(const auto& command:f.commands)check(!command.complete&&command.disarm,"部分终态命令错误");
}
void landingTimeoutKeepsLandingCommands() {
  Fixture f(false,-1,false,false,2000000);f.contact();
  for(int active=0;active<4;++active){
    if(active){f.set(active,f.c.approach);f.settle();}
    f.set(active,f.c.target);++f.states[active].impact_seq;f.tick();
    f.set(active,f.c.approach);f.settle();
    f.set(active,f.c.assembly[active]);f.settle();
  }
  check(f.mission.phase()==TeamPhase::RETURN_ALL,"降落超时用例未返航");
  for(int i=0;i<4;++i){auto point=f.c.homes[i];point.z=f.c.cruise_z_m;f.set(i,point);}
  f.settle();check(f.mission.phase()==TeamPhase::LAND_ALL,"降落超时用例未进入降落阶段");
  f.follow();f.tick(-1,3000000);
  check(f.mission.phase()==TeamPhase::LAND_ALL&&f.mission.aborted(),"降落超时回到返航或提前结束");
  for(const auto& command:f.commands)check(command.phase==TeamPhase::LAND_ALL&&!command.complete&&!command.disarm,"未着地时降落命令错误");
  for(auto& state:f.states)state.landed=true;
  f.settle();
  for(const auto& command:f.commands)check(command.disarm&&!command.complete,"降落超时后未请求停机");
  for(auto& state:f.states)state.armed=false;
  f.settle();check(f.mission.phase()==TeamPhase::ABORTED,"超时降落停机后没有确认终态");
}
void narrowHangarLandsOneVehicleAtATime() {
  auto c=config(false,-1,false,false,0,true);
  c.landing_alignment_height_m=.2;c.landing_require_structural=true;c.landing_alignment_stable_us=300000;
  c.landing_order={3,2,1,0};
  c.stage_timeout_us=4000000;
  Fixture f(c);f.contact();
  for(int active=0;active<4;++active){
    if(active){f.set(active,f.c.approach);f.settle();}
    f.set(active,f.c.target);++f.states[active].impact_seq;f.tick();
    f.set(active,f.c.approach);f.settle();
    f.set(active,f.c.assembly[active]);f.settle();
  }
  f.tick();check(f.mission.phase()==TeamPhase::LAND_ALL,"逐架降落未在集结位置开始");
  for(int i=0;i<4;++i)if(i!=f.c.landing_order.front())
    check(f.commands[i].phase==TeamPhase::RETURN_ALL&&f.commands[i].target_y_m==f.c.assembly[i].y&&
          f.commands[i].target_z_m==f.c.cruise_z_m,"等候机提前进入机库或下降");
  for(size_t slot=0;slot<f.c.landing_order.size();++slot){
    const int active=f.c.landing_order[slot];
    auto home=f.c.homes[active];home.z=f.c.cruise_z_m;f.set(active,home);
    f.settle();check(f.commands[active].target_z_m==f.c.home_ground_z_m+.2,"未在托盘上方停稳对准");
    home.z=f.c.home_ground_z_m+.2;f.set(active,home);
    f.states[active].x_m+=f.c.landing_alignment_tolerance_m+.001;f.settle();
    check(f.commands[active].phase==TeamPhase::RETURN_ALL,"对准误差超限仍下降");
    f.set(active,home);f.states[active].vx_mps=.11;f.settle();
    check(f.commands[active].phase==TeamPhase::RETURN_ALL,"水平速度过高仍下降");
    f.states[active].vx_mps=0;f.states[active].xy_reason="PEER_REFERENCE";f.settle();
    check(f.commands[active].phase==TeamPhase::RETURN_ALL,"仅有队友参考时仍下降");
    f.states[active].xy_reason="STRUCTURAL";f.settle();
    check(f.commands[active].phase==TeamPhase::LAND_ALL,"已对准停稳仍未下降");
    home.z=f.c.home_ground_z_m+f.c.body_clearance_m;f.set(active,home);f.states[active].landed=true;f.tick();
    check(f.commands[active].disarm,"着地后未请求停机");
    if(slot+1<f.c.landing_order.size()) {
      const int next=f.c.landing_order[slot+1];
      check(f.commands[next].target_y_m==f.c.assembly[next].y,"当前机尚未停机便轮换");
    }
    f.states[active].armed=false;f.tick(-1,1100000);
    check(!f.mission.aborted(),"逐架降落错误复用前机阶段期限");
    check(f.mission.phase()!=TeamPhase::COMPLETE,"未确认全部成员停机便完成");
  }
  f.settle();check(f.mission.phase()==TeamPhase::COMPLETE,"全员逐架着地停机后未完成");
}
void narrowHangarTimeoutDescendsAtSeparatePositions() {
  Fixture f(false,-1,false,false,2000000,true);f.contact();
  for(int active=0;active<4;++active){
    if(active){f.set(active,f.c.approach);f.settle();}
    f.set(active,f.c.target);++f.states[active].impact_seq;f.tick();
    f.set(active,f.c.approach);f.settle();
    f.set(active,f.c.assembly[active]);f.settle();
  }
  f.tick();f.tick(-1,2100000);
  check(f.mission.phase()==TeamPhase::LAND_ALL&&f.mission.aborted(),"逐架降落超时没有中止");
  for(int i=0;i<4;++i){
    check(f.commands[i].phase==TeamPhase::LAND_ALL&&!f.commands[i].disarm,"未着地中止机停机或仍在等待");
    check(f.commands[i].target_x_m==f.states[i].x_m&&f.commands[i].target_y_m==f.states[i].y_m,
          "降落中止后仍挤向同一机库");
    f.states[i].landed=true;
  }
  f.tick();for(const auto& command:f.commands)check(command.disarm,"中止着地后未请求停机");
  for(auto& state:f.states)state.armed=false;
  f.settle();check(f.mission.phase()==TeamPhase::ABORTED,"中止后未等待全员原生停机");
}
void landingAlignmentHeightStaysAboveTheBodyAndBelowCruise() {
  for(double height:{-.1,.005,1.0}) {
    auto c=config();c.landing_alignment_height_m=height;
    bool rejected=false;
    try { TeamMission mission(c); } catch(const std::invalid_argument&) { rejected=true; }
    check(rejected,"不安全的降落对准高度被接受");
  }
  for(const auto& order:std::vector<std::vector<int>>{{0,1},{0,1,1,3},{0,1,2,4}}) {
    auto c=config();c.landing_order=order;
    bool rejected=false;
    try { TeamMission mission(c); } catch(const std::invalid_argument&) { rejected=true; }
    check(rejected,"无效的降落顺序被接受");
  }
}
int main() {
  try {
    barriers(); physicalContactAndTimeout(); contactDeadlineRemainsFixed(); sequentialAndLanding(); staleAndAbort(); transitAndReverse(); inputGates(); contactGates(); repeatedLossKeepsOneRecord();
    visualApproachRangeGate(); visualRangeGateDisabledWithoutSetting(); visualGateIgnoresApproachYaw();
    diveAttackFliesThroughEntryGate(); diveEntryRejectsMisalignedHeading(); contactTimeoutSkipsFailedVehicle();
    approachTimeoutSkipsVehicleThatCannotReachEntry(); retreatHoldingDirect();
    retreatUsesSeparateSideExitBeforeHolding();
    landedVehicleFailsImmediately(); landedDuringTakeoffIsNotFailure();
    allUnrecoverableWaitsForFreshLandingAndDisarm();
    partialLandingWaitsForSkippedMember();
    landingTimeoutKeepsLandingCommands();
    narrowHangarLandsOneVehicleAtATime();
    narrowHangarTimeoutDescendsAtSeparatePositions();
    landingAlignmentHeightStaysAboveTheBodyAndBelowCruise();
    std::cout << "队伍任务测试通过\n"; return 0;
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}



