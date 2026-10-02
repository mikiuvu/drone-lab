#include "companion/team_mission.hpp"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <direct.h>
#include <errno.h>
#else
#include <arpa/inet.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
#include <sys/stat.h>
#include <errno.h>
#endif

namespace {
using json = nlohmann::json;
using namespace companion;
volatile std::sig_atomic_t running = 1;
void stop(int) { running = 0; }
uint64_t wallTime() {
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
}
double number(const json& j, const char* key) {
  const auto& value = j.at(key);
  if (!value.is_number()) throw std::invalid_argument(std::string("数值字段无效：")+key);
  const double n = value.get<double>();
  if (!std::isfinite(n)) throw std::invalid_argument(std::string("数值字段无效：")+key);
  return n;
}
uint64_t integer(const json& j, const char* key) {
  const auto& v = j.at(key);
  if (!v.is_number_integer() || (!v.is_number_unsigned() && v.get<int64_t>() < 0))
    throw std::invalid_argument(std::string("整数字段无效：")+key);
  return v.get<uint64_t>();
}
int vehicleId(const json& j, const char* key, size_t count) {
  const auto id = integer(j,key);
  if (id >= count) throw std::invalid_argument("机号超出范围");
  return static_cast<int>(id);
}
bool boolean(const json& j, const char* key) {
  if (!j.at(key).is_boolean()) throw std::invalid_argument(std::string("布尔字段无效：")+key);
  return j.at(key).get<bool>();
}
TeamPoint point(const json& j, double defaultZ) {
  TeamPoint p; p.x = number(j,"x"); p.y = number(j,"y");
  p.z = j.contains("z") ? number(j,"z") : defaultZ;
  p.yaw = j.contains("yaw") ? number(j,"yaw") : 0;
  return p;
}
void pointsById(const json& entries, std::vector<TeamPoint>& result, double z) {
  if (!entries.is_array() || entries.empty()) throw std::invalid_argument("必须配置至少一个机号");
  result.assign(entries.size(), {}); std::vector<bool> seen(entries.size());
  for (const auto& entry : entries) {
    const int id = vehicleId(entry,"id",entries.size());
    if (seen[id]) throw std::invalid_argument("机号重复");
    seen[id] = true; result[id] = point(entry,z);
  }
}
std::string joinPath(const std::string& directory, const char* file) {
  if (directory.empty()) return file;
  const char last = directory.back();
  return directory + ((last == '/' || last == '\\') ? "" : "/") + file;
}
void ensureDirectory(const std::string& directory) {
#ifdef _WIN32
  if (_mkdir(directory.c_str()) != 0 && errno != EEXIST) throw std::runtime_error("无法创建运行目录");
#else
  if (mkdir(directory.c_str(),0777) != 0 && errno != EEXIST) throw std::runtime_error("无法创建运行目录");
#endif
}
TeamMissionConfig readConfig(const std::string& path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("无法读取地图配置");
  json map; input >> map; const auto& m = map.at("team_mission");
  TeamMissionConfig c;
  if (m.contains("cruise_z_m")) c.cruise_z_m = number(m,"cruise_z_m");
  if (m.contains("contact_speed_mps")) c.contact_speed_mps = number(m,"contact_speed_mps");
  if (m.contains("return_tolerance_m")) c.return_tolerance_m = number(m,"return_tolerance_m");
  c.route_tolerance_m=m.value("route_tolerance_m",c.return_tolerance_m);
  c.route_speed_tolerance_mps=m.value("route_speed_tolerance_mps",.30);
  c.fly_through_route=m.value("fly_through_route",false);
  c.pipelined_contact=m.value("pipelined_contact",false);
  if(m.contains("stable_s")) {
    const double seconds=number(m,"stable_s");
    if(seconds<=0||seconds>10)throw std::invalid_argument("稳定确认时间无效");
    c.stable_us=static_cast<uint64_t>(seconds*1e6);
  }
  if (m.contains("home_ground_z_m")) c.home_ground_z_m = number(m,"home_ground_z_m");
  if (m.contains("body_clearance_m")) c.body_clearance_m=number(m,"body_clearance_m");
  if (m.contains("stage_timeout_s")) {
    const double seconds=number(m,"stage_timeout_s");
    if(seconds<0 || seconds>86400) throw std::invalid_argument("阶段超时时间超出范围");
    c.stage_timeout_us=static_cast<uint64_t>(seconds*1e6);
  }
  if (m.contains("loss_abort_s")) { const double seconds=number(m,"loss_abort_s");if(seconds<1||seconds>30)throw std::invalid_argument("失联中止时间超出范围");c.loss_abort_us=static_cast<uint64_t>(seconds*1e6); }
  pointsById(map.at("spawns"),c.homes,c.home_ground_z_m+.0125);
  pointsById(m.at("assembly"),c.assembly,c.cruise_z_m);
  if(m.contains("completed_holding"))pointsById(m.at("completed_holding"),c.completed_holding,c.cruise_z_m);
  c.contact_exit_side=m.value("contact_exit_side",0.0);
  c.approach = point(m.at("approach"),c.cruise_z_m);
  c.target = point(m.at("target"),c.approach.z);
  const auto& order = m.at("order");
  if (!order.is_array() || order.size() != c.homes.size()) throw std::invalid_argument("接触顺序无效");
  c.order.clear();
  for (const auto& value : order) {
    json id = {{"id",value}}; c.order.push_back(vehicleId(id,"id",c.homes.size()));
  }
  if (m.contains("transit_waypoints")) {
    if (!m.at("transit_waypoints").is_array()) throw std::invalid_argument("航路配置无效");
    for (const auto& p : m.at("transit_waypoints")) c.transit_waypoints.push_back(point(p,c.cruise_z_m));
  }
  if (m.contains("formation_offsets")) {
    pointsById(m.at("formation_offsets"),c.formation_offsets,0); c.explicit_formation_offsets = true;
  }
  return c;
}
TeamVehicleStatus readStatus(const char* data, int length) {
  const json j = json::parse(data,data+length);
  if (!j.is_object() || integer(j,"schema") != 1 || j.at("kind") != "team_status")
    throw std::invalid_argument("状态报文格式无效");
  TeamVehicleStatus s; s.vehicle = vehicleId(j,"vehicle",static_cast<size_t>(std::numeric_limits<int>::max()));
  s.sim_time_us = integer(j,"sim_time_us"); s.impact_seq = integer(j,"impact_seq");
  s.x_m = number(j,"x_m"); s.y_m = number(j,"y_m"); s.z_m = number(j,"z_m");
  s.vx_mps = number(j,"vx_mps"); s.vy_mps = number(j,"vy_mps"); s.vz_mps = number(j,"vz_mps");
  s.yaw_rad = number(j,"yaw_rad");
  s.xy_valid = boolean(j,"xy_valid");
  s.flow_valid = j.contains("flow_valid") ? boolean(j,"flow_valid") : false;
  s.xy_valid = s.xy_valid || s.flow_valid;
  s.z_valid = boolean(j,"z_valid"); s.ready = boolean(j,"ready"); s.armed = boolean(j,"armed");
  s.landed = j.contains("landed") ? boolean(j,"landed") : false;
  s.xy_reason = j.contains("localization_mode") ? j.at("localization_mode").get<std::string>() :
      (j.contains("xy_reason") ? j.at("xy_reason").get<std::string>() : "");
  return s;
}
json commandJson(const TeamCommand& c) {
  return {{"schema",1},{"kind","team_command"},{"vehicle",c.vehicle},
      {"sim_time_us",c.sim_time_us},{"sequence",c.sequence},
      {"phase",c.degraded && c.phase != TeamPhase::WAIT_READY ? "HOLD" : teamPhaseName(c.phase)},
      {"mission_phase",teamPhaseName(c.phase)},
      {"active_vehicle",c.active_vehicle},{"target_x_m",c.target_x_m},{"target_y_m",c.target_y_m},
      {"target_z_m",c.target_z_m},{"target_yaw_rad",c.target_yaw_rad},
      {"contact_speed_mps",c.contact_speed_mps},{"paused",c.degraded},
      {"degraded",c.degraded},{"degraded_reason",c.degraded_reason},{"skipped",c.skipped},
      {"disarm",c.disarm},{"complete",c.complete}};
}
void atomicStatus(const std::string& path, const json& status) {
  const auto temporary = path + ".tmp";
  {
    std::ofstream output(temporary,std::ios::binary | std::ios::trunc);
    if (!output || !(output << status.dump(2) << '\n')) throw std::runtime_error("无法写入任务状态");
    output.close();
    if (!output) throw std::runtime_error("无法保存任务状态");
  }
#ifdef _WIN32
  if (!MoveFileExA(temporary.c_str(),path.c_str(),MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH))
    throw std::runtime_error("无法替换任务状态文件");
#else
  if (std::rename(temporary.c_str(),path.c_str()) != 0) throw std::runtime_error("无法替换任务状态文件");
#endif
}
#ifdef _WIN32
using Socket = SOCKET;
constexpr Socket invalidSocket = INVALID_SOCKET;
#else
using Socket = int;
constexpr Socket invalidSocket = -1;
#endif
struct UdpSocket {
  Socket fd = invalidSocket;
  UdpSocket() {
#ifdef _WIN32
    WSADATA data{};
    if (WSAStartup(MAKEWORD(2,2),&data) != 0) throw std::runtime_error("无法初始化网络");
#endif
    fd = socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);
    if (fd == invalidSocket) throw std::runtime_error("无法创建 UDP 套接字");
  }
  ~UdpSocket() {
#ifdef _WIN32
    if (fd != invalidSocket) closesocket(fd);
    WSACleanup();
#else
    if (fd != invalidSocket) close(fd);
#endif
  }
};
}  // namespace

int main(int argc, char** argv) {
  try {
    std::string map, runtime;
    for (int i = 1; i < argc; ++i) {
      const std::string option = argv[i];
      if ((option == "--map" || option == "--runtime") && i+1 < argc) {
        (option == "--map" ? map : runtime) = argv[++i];
      } else throw std::invalid_argument("用法：team_coordinator --map 地图.json --runtime 输出目录");
    }
    if (map.empty() || runtime.empty()) throw std::invalid_argument("必须指定地图和输出目录");
    TeamMission mission(readConfig(map));
    ensureDirectory(runtime);
    std::ofstream log(joinPath(runtime,"team_mission.jsonl"),std::ios::binary | std::ios::app);
    if (!log) throw std::runtime_error("无法打开任务日志");
    UdpSocket udp; sockaddr_in local{}; local.sin_family = AF_INET;
    local.sin_port = htons(18100); local.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (bind(udp.fd,reinterpret_cast<sockaddr*>(&local),sizeof(local)) != 0)
      throw std::runtime_error("无法绑定 UDP 18100");
    std::signal(SIGINT,stop); std::signal(SIGTERM,stop);
    uint64_t sim = 0, lastEmit = 0, rejected = 0, telemetry_errors = 0, send_errors = 0;
    const size_t team_count = mission.statuses().size();
    std::vector<bool> seen(team_count);
    std::vector<uint64_t> received(team_count), stamps(team_count);
    while (running) {
      fd_set reads; FD_ZERO(&reads); FD_SET(udp.fd,&reads); timeval wait{0,20000};
      const int selected = select(static_cast<int>(udp.fd)+1,&reads,nullptr,nullptr,&wait);
      if (selected < 0) { if (!running) break; throw std::runtime_error("UDP 等待失败"); }
      if (selected > 0) {
        char data[8192]; sockaddr_in sender{};
#ifdef _WIN32
        int senderLength = sizeof(sender);
#else
        socklen_t senderLength = sizeof(sender);
#endif
        const int count = static_cast<int>(recvfrom(udp.fd,data,sizeof(data),0,
            reinterpret_cast<sockaddr*>(&sender),&senderLength));
        if (count > 0 && sender.sin_addr.s_addr == htonl(INADDR_LOOPBACK)) {
          try {
            const auto s = readStatus(data,count); const auto wall = wallTime();
            if (!mission.updateStatus(s,wall)) ++rejected;
            else {
              sim = std::max(sim,s.sim_time_us); seen[s.vehicle] = true;
              received[s.vehicle] = wall; stamps[s.vehicle] = s.sim_time_us;
            }
          } catch (const std::exception&) { ++rejected; }
        } else ++rejected;
      }
      const auto wall = wallTime();
      if (wall-lastEmit < 50000) continue;
      lastEmit = wall;
      const auto commands = mission.step(sim,wall); json targets = json::array(), vehicles = json::array();
      for (const auto& c : commands) {
        const json value = commandJson(c); const auto payload = value.dump(); targets.push_back(value);
        sockaddr_in peer{}; peer.sin_family = AF_INET; peer.sin_port = htons(static_cast<uint16_t>(18000+c.vehicle));
        peer.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        if (sendto(udp.fd,payload.data(),static_cast<int>(payload.size()),0,
            reinterpret_cast<sockaddr*>(&peer),sizeof(peer)) < 0) ++send_errors;
        const int id = c.vehicle;
        const auto& status = mission.statuses()[id];
        vehicles.push_back({{"vehicle",id},{"seen",static_cast<bool>(seen[id])},
            {"fresh",seen[id] && wall-received[id] <= 1000000 && sim-stamps[id] <= 1000000},
            {"ready",status.ready},{"xy_valid",status.xy_valid},{"z_valid",status.z_valid},
            {"degraded",c.degraded},{"degraded_reason",c.degraded_reason},{"skipped",c.skipped}});
      }
      const json snapshot = {{"schema",1},{"sim_time_us",sim},{"sequence",commands.empty()?0:commands[0].sequence},
          {"phase",mission.paused() && mission.phase() != TeamPhase::WAIT_READY ? "PAUSED" : teamPhaseName(mission.phase())},
          {"mission_phase",teamPhaseName(mission.phase())},{"active_vehicle",mission.activeVehicle()},
          {"completed",mission.completed()},{"paused",mission.paused()},{"aborted",mission.aborted()},
          {"flight_elapsed_s",mission.flightElapsedSeconds()},
          {"complete",mission.phase() == TeamPhase::COMPLETE},{"partial",mission.partial()},{"skipped",mission.skipped()},{"reason",mission.reason()},
          {"rejected_datagrams",rejected},{"telemetry_errors",telemetry_errors},{"send_errors",send_errors},{"vehicles",vehicles},{"commands",targets}};
      log << snapshot.dump() << '\n'; log.flush();
      if (!log) {
        ++telemetry_errors;
        log.close(); log.clear();
        log.open(joinPath(runtime,"team_mission.jsonl"),std::ios::binary | std::ios::app);
      }
      try { atomicStatus(joinPath(runtime,"team_status.json"),snapshot); }
      catch (const std::exception& e) {
        ++telemetry_errors;
        if (telemetry_errors==1 || telemetry_errors%100==0)
          std::cerr << "任务状态写入错误：" << e.what() << '\n';
      }
    }
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "队伍任务错误：" << e.what() << '\n'; return 1;
  }
}
