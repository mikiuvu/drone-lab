#include "companion/team_mission.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace companion;
namespace {
void check(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
TeamMissionConfig config(bool transit = false) {
  TeamMissionConfig c; c.stable_us = 200000;
  for (int i = 0; i < kTeamSize; ++i) {
    c.homes[i] = {double(i)-1.5,0,.5125,0};
    c.assembly[i] = {double(i)-1.5,4,1.2,0};
  }
  c.approach = {0,5,1.2,0}; c.target = {1,5,1.2,0};
  if (transit) c.transit_waypoints = {{0,1,1.2,0},{0,2,1.4,0}};
  return c;
}
struct Fixture {
  TeamMissionConfig c; TeamMission mission;
  std::array<TeamVehicleStatus,kTeamSize> states{};
  uint64_t now = 1000000, wall = 1000000;
  std::array<TeamCommand,kTeamSize> commands{};
  explicit Fixture(bool transit = false) : c(config(transit)), mission(c) {
    for (int i = 0; i < kTeamSize; ++i) {
      states[i].vehicle = i; states[i].xy_valid = states[i].z_valid = states[i].ready = true;
      set(i,c.homes[i]);
    }
  }
  void set(int id, TeamPoint p) {
    states[id].x_m = p.x; states[id].y_m = p.y; states[id].z_m = p.z; states[id].yaw_rad = p.yaw;
  }
  void tick(int omitted = -1, uint64_t dt = 100000) {
    now += dt; wall += dt;
    for (int i = 0; i < kTeamSize; ++i) if (i != omitted) {
      states[i].sim_time_us = now; check(mission.updateStatus(states[i],wall),"有效状态被拒绝");
    }
    commands = mission.step(now,wall);
  }
  void settle(int omitted = -1) { for (int j = 0; j < 4; ++j) tick(omitted); }
  void follow() {
    for (int i = 0; i < kTeamSize; ++i)
      set(i,{commands[i].target_x_m,commands[i].target_y_m,commands[i].target_z_m,commands[i].target_yaw_rad});
  }
  void takeoff() {
    settle(); check(mission.phase() == TeamPhase::TAKEOFF,"就绪阶段未完成");
    for (int i = 0; i < kTeamSize; ++i) { states[i].armed = true; auto p = c.homes[i]; p.z = c.cruise_z_m; set(i,p); }
    settle();
  }
  void contact() {
    takeoff(); check(mission.phase() == TeamPhase::ASSEMBLE,"起飞阶段未完成");
    for (int i = 0; i < kTeamSize; ++i) set(i,c.assembly[i]);
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
  check(f.mission.phase() == TeamPhase::ABORT_RETURN && f.mission.aborted() &&
        f.mission.completed().empty(),"接触超时被计为成功");
}
void sequentialAndLanding() {
  Fixture f; f.contact();
  for (int active = 0; active < kTeamSize; ++active) {
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
  check(f.mission.phase() == TeamPhase::COMPLETE,"着陆完成失败");
  for (const auto& command : f.commands) check(command.disarm && command.complete,"完成后未解除武装");
}
void staleAndAbort() {
  Fixture f; f.contact();
  for (int i = 0; i < 12; ++i) f.tick(3);
  check(f.mission.paused() && f.commands[0].contact_speed_mps == 0,"状态过期未暂停接触");
  const double hold = f.commands[0].target_x_m;
  for (int i = 0; i < 10; ++i) f.tick(3);
  check(f.commands[0].target_x_m == hold && f.mission.completed().empty(),"暂停目标漂移");
  ++f.states[0].impact_seq; f.tick();
  check(f.mission.phase() == TeamPhase::CONTACT && f.mission.completed().empty(),"暂停期间碰撞被计为成功");
  for (int i = 0; i < 65; ++i) f.tick(3);
  check(f.mission.aborted() && f.mission.paused(),"长期数据中断未中止");
  f.tick(); check(f.mission.phase() == TeamPhase::ABORT_RETURN && !f.mission.paused(),"恢复后未中止返航");
  f.follow(); f.settle(); check(f.mission.phase() == TeamPhase::LAND_ALL,"中止归位后未降落");
  f.follow();
  for (auto& s : f.states) s.landed = true;
  for (int i = 0; i < 12; ++i) f.tick(3);
  check(f.mission.phase() == TeamPhase::LAND_ALL,"缺失状态仍然完成着陆");
  f.tick(); f.settle();
  check(f.mission.phase() == TeamPhase::ABORTED,"中止着陆错误");
  for (const auto& c : f.commands) check(c.disarm && !c.complete,"中止被计为任务成功");
  Fixture frozen; frozen.contact();
  frozen.wall += 2000000; frozen.commands = frozen.mission.step(frozen.now,frozen.wall);
  check(frozen.mission.paused(),"仿真时钟冻结时未按接收时间检查过期");
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
  for (int i = 0; i < kTeamSize; ++i) {
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
}
int main() {
  try {
    barriers(); physicalContactAndTimeout(); sequentialAndLanding(); staleAndAbort(); transitAndReverse(); inputGates(); contactGates();
    std::cout << "队伍任务测试通过\n"; return 0;
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
