#include "companion/team_mission.hpp"
#include <cassert>
#include <vector>

using namespace companion;

TeamMissionConfig config(int count) {
  TeamMissionConfig c;
  c.homes.clear();c.assembly.clear();c.order.clear();
  for(int i=0;i<count;++i){c.homes.push_back({i*.5,0,.5125,0});c.assembly.push_back({i*.5,3,1.2,0});c.order.push_back(i);}
  c.transit_waypoints={{1,1,1.2,0},{2,2,1.2,0}};
  c.approach={2,3,1.2,0};c.target={2.2,3,1.2,0};
  return c;
}

void fullMission(int count, bool fast=false) {
  auto c=config(count);
  c.pipelined_contact=c.fly_through_route=fast;
  c.route_speed_tolerance_mps=3.5;
  if(fast){
    c.contact_exit_side=-1;
    for(int i=0;i<count;++i){c.assembly[i]={1,3.5+i*.3,1.2,0};c.completed_holding.push_back({1,2.5-i*.3,1.2,0});}
  }
  TeamMission mission(c);
  std::vector<TeamVehicleStatus> states(count);
  for(int i=0;i<count;++i){auto&s=states[i];s.vehicle=i;s.x_m=c.homes[i].x;s.z_m=.5125;s.xy_valid=s.z_valid=s.ready=s.flow_valid=true;}
  std::vector<TeamCommand> commands;
  bool reached_return=false;int expected=0;
  for(uint64_t tick=1;tick<1000;++tick){
    for(int i=0;i<count;++i){
      auto&s=states[i];
      if(!commands.empty()){
        const auto&cmd=commands[i];s.x_m=cmd.target_x_m;s.y_m=cmd.target_y_m;s.z_m=cmd.target_z_m;s.yaw_rad=cmd.target_yaw_rad;
        if(cmd.phase!=TeamPhase::WAIT_READY&&!cmd.disarm)s.armed=true;
        if(cmd.disarm)s.armed=false;
        if(cmd.phase==TeamPhase::CONTACT&&cmd.active_vehicle==i){
          s.x_m=c.target.x;s.y_m=c.target.y;s.z_m=c.target.z;++s.impact_seq;
        }
        if(cmd.phase==TeamPhase::LAND_ALL)s.landed=true;
        s.vx_mps=fast&&cmd.phase==TeamPhase::TRANSIT?2.0:0.0;
      }
      s.sim_time_us=tick*100000;assert(mission.updateStatus(s,tick*100000));
    }
    commands=mission.step(tick*100000,tick*100000);
    assert(commands.size()==static_cast<size_t>(count));
    for(int i:mission.completed()){assert(i==expected++);}expected=0;
    if(mission.phase()==TeamPhase::RETURN_ALL){assert(mission.completed().size()==static_cast<size_t>(count));reached_return=true;}
    if(mission.phase()==TeamPhase::COMPLETE){
      assert(reached_return&&!mission.partial());
      assert(mission.flightElapsedSeconds()>0);
      const auto elapsed=mission.flightElapsedSeconds();
      mission.step(tick*100000+1000000,tick*100000+1000000);
      assert(mission.flightElapsedSeconds()==elapsed);
      for(const auto&cmd:commands)assert(cmd.complete&&cmd.disarm);
      return;
    }
  }
  assert(false&&"command-following mission did not complete");
}

void individualDegradation() {
  auto c=config(6);TeamMission m(c);std::vector<TeamVehicleStatus> states(6);
  for(int i=0;i<6;++i){states[i].vehicle=i;states[i].xy_valid=states[i].z_valid=states[i].ready=true;states[i].z_m=.5125;}
  for(int t=1;t<9;++t){for(auto&s:states){s.sim_time_us=t*100000;m.updateStatus(s,t*100000);}m.step(t*100000,t*100000);}
  states[5].xy_valid=false;
  for(auto&s:states){s.sim_time_us=900000;m.updateStatus(s,900000);}
  auto commands=m.step(900000,900000);
  assert(!m.paused());
  assert(commands[5].degraded);
  for(int i=0;i<5;++i){assert(!commands[i].degraded);assert(commands[i].target_z_m==c.cruise_z_m);}
}

void returnRouteRequiresPositionAndSpeedSettling() {
  auto c=config(1);c.fly_through_route=true;c.route_tolerance_m=.5;
  c.route_speed_tolerance_mps=5.5;c.return_tolerance_m=.14;
  TeamMission mission(c);TeamVehicleStatus state;state.vehicle=0;
  state.ready=state.xy_valid=state.z_valid=true;state.z_m=c.homes[0].z;
  std::vector<TeamCommand> commands;uint64_t now=0;
  auto tick=[&](){now+=100000;state.sim_time_us=now;assert(mission.updateStatus(state,now));commands=mission.step(now,now);};
  for(int count=0;count<500&&mission.phase()!=TeamPhase::RETURN_ALL;++count){
    if(!commands.empty()){
      const auto& command=commands[0];
      state.x_m=command.target_x_m;state.y_m=command.target_y_m;state.z_m=command.target_z_m;
      state.armed=command.phase!=TeamPhase::WAIT_READY;
      if(command.phase==TeamPhase::CONTACT){state.x_m=c.target.x;state.y_m=c.target.y;++state.impact_seq;}
      state.vx_mps=command.phase==TeamPhase::TRANSIT?2.0:0.0;
    }
    tick();
  }
  assert(mission.phase()==TeamPhase::RETURN_ALL);
  const auto waypoint=c.transit_waypoints.back();
  state.x_m=waypoint.x;state.y_m=waypoint.y;state.z_m=waypoint.z;
  state.vx_mps=2.0;
  for(int count=0;count<10;++count){tick();assert(commands[0].target_x_m==waypoint.x&&commands[0].target_y_m==waypoint.y);}
  state.vx_mps=0.0;state.x_m=waypoint.x+.3;
  for(int count=0;count<10;++count){tick();assert(commands[0].target_x_m==waypoint.x&&commands[0].target_y_m==waypoint.y);}
  state.x_m=waypoint.x;
  for(int count=0;count<10;++count)tick();
  assert(commands[0].target_x_m==c.transit_waypoints.front().x&&commands[0].target_y_m==c.transit_waypoints.front().y);
}

void missingVehicleAndRecovery() {
  auto c=config(2); c.loss_abort_us=1000000;
  TeamMission mission(c);
  TeamVehicleStatus first; first.vehicle=0;first.xy_valid=first.z_valid=first.ready=true;
  first.z_m=c.homes[0].z;
  for(uint64_t tick=1;tick<=18;++tick) {
    first.sim_time_us=tick*100000;mission.updateStatus(first,first.sim_time_us);
    mission.step(first.sim_time_us,first.sim_time_us);
  }
  assert(mission.phase()==TeamPhase::TAKEOFF && mission.partial());
  TeamVehicleStatus second=first;second.vehicle=1;
  second.sim_time_us=1900000;
  assert(mission.updateStatus(second,1900000));
  assert(!mission.partial());
  auto commands=mission.step(1900000,1900000);
  assert(!commands[1].skipped && !commands[1].degraded);

  TeamMission empty(c);
  for(uint64_t tick=1;tick<=30;++tick)empty.step(tick*100000,tick*100000);
  assert(empty.phase()==TeamPhase::WAIT_READY && empty.completed().empty());
}

void visionRequired() {
  auto c=config(1);c.visual_contact=true;c.transit_waypoints.clear();
  TeamMission mission(c);TeamVehicleStatus state;state.vehicle=0;
  state.ready=state.xy_valid=state.z_valid=state.armed=true;state.z_m=c.cruise_z_m;
  std::vector<TeamCommand> commands;
  for(uint64_t i=1;i<80;++i){
    if(!commands.empty()){state.x_m=commands[0].target_x_m;state.y_m=commands[0].target_y_m;state.z_m=commands[0].target_z_m;}
    state.sim_time_us=i*100000;mission.updateStatus(state,state.sim_time_us);commands=mission.step(state.sim_time_us,state.sim_time_us);
  }
  assert(mission.phase()==TeamPhase::APPROACH);
  state.visual_ready=true;
  for(uint64_t i=80;i<90;++i){state.sim_time_us=i*100000;mission.updateStatus(state,state.sim_time_us);commands=mission.step(state.sim_time_us,state.sim_time_us);}
  assert(mission.phase()==TeamPhase::CONTACT&&mission.completed().empty());
  // 移动装甲不在地图静态目标点；视觉门控的冲击序号由 runner 提供。
  state.y_m+=1;state.sim_time_us=9000000;++state.impact_seq;
  mission.updateStatus(state,state.sim_time_us);mission.step(state.sim_time_us,state.sim_time_us);
  assert(mission.phase()==TeamPhase::RETREAT&&mission.completed().size()==1);
}

void abortKeepsGroundedMembersStopped() {
  auto c=config(2);c.transit_waypoints.clear();c.stable_us=100000;c.stage_timeout_us=500000;
  TeamMission mission(c);std::vector<TeamVehicleStatus> states(2);
  for(int i=0;i<2;++i){auto&s=states[i];s.vehicle=i;s.ready=s.xy_valid=s.z_valid=true;
    s.x_m=c.homes[i].x;s.y_m=c.homes[i].y;s.z_m=i==0?c.home_ground_z_m+c.body_clearance_m:c.cruise_z_m;
    s.armed=i==1;s.landed=i==0;}
  uint64_t now=0;std::vector<TeamCommand> commands;
  auto tick=[&](){now+=50000;for(auto&s:states){s.sim_time_us=now;assert(mission.updateStatus(s,now));}commands=mission.step(now,now);};
  for(int n=0;n<40&&mission.phase()!=TeamPhase::LAND_ALL;++n)tick();
  assert(mission.phase()==TeamPhase::LAND_ALL&&"grounded member blocks abort return");
  assert(commands[0].skipped&&mission.partial());
  // 停机反馈必须同时包含着地；只有解除武装不能完成终态。
  states[1].armed=false;states[1].landed=false;
  for(int n=0;n<5;++n){tick();assert(mission.phase()==TeamPhase::LAND_ALL);}
  states[1].landed=true;
  for(int n=0;n<5;++n)tick();
  assert(mission.phase()==TeamPhase::ABORTED);

  TeamMission emptyFlight(c);now=0;
  for(auto&s:states){s.armed=false;s.landed=true;s.z_m=c.home_ground_z_m+c.body_clearance_m;}
  for(int n=0;n<40&&emptyFlight.phase()!=TeamPhase::ABORTED;++n){now+=50000;
    for(auto&s:states){s.sim_time_us=now;assert(emptyFlight.updateStatus(s,now));}emptyFlight.step(now,now);}
  assert(emptyFlight.phase()==TeamPhase::ABORTED);
}

int main(){for(int n:{1,4,6,12}){fullMission(n);fullMission(n,true);}individualDegradation();returnRouteRequiresPositionAndSpeedSettling();missingVehicleAndRecovery();visionRequired();abortKeepsGroundedMembersStopped();}
