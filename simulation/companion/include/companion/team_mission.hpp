/**
 * @file team_mission.hpp
 * @brief 多机任务的数据契约：把每机估计状态转换成下一步目标点。
 *
 * 阅读顺序：TeamMissionConfig（要去哪里）→ TeamVehicleStatus（现在在哪里）
 * → TeamPhase（任务做到哪一步）→ TeamCommand（每架下一步做什么）。
 * 本模块不操作电机，也不收 UDP；网络由 team_coordinator.cpp 处理，
 * 速度指令和 MAVLink 由 runner.cpp 处理，飞控负责姿态和电机闭环。
 */
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace companion {
/** @brief 世界坐标中的目标点；x/y/z 单位 m，yaw 单位 rad，零航向指向 +X。 */
struct TeamPoint { double x = 0, y = 0, z = 0, yaw = 0; };
/**
 * @brief 全队任务配置，由地图 JSON 的 spawns 和 team_mission 转换而来。
 * @note 向量下标就是机号。homes 的长度决定机群数量，默认四个值只是默认配置。
 *       assembly、order 和显式编队偏移必须与机群长度一致。
 */
struct TeamMissionConfig {
  // 世界高度，不是离地高度；例如平台 z=0.5 m、巡航 z=0.75 m，相差 0.25 m。
  double cruise_z_m = 1.2;
  std::vector<TeamPoint> homes{{-1.5,0,.5125,0},{-.5,0,.5125,0},{.5,0,.5125,0},{1.5,0,.5125,0}}, assembly{{-1.5,4,1.2,0},{-.5,4,1.2,0},{.5,4,1.2,0},{1.5,4,1.2,0}};
  TeamPoint approach{}, target{}; // approach 是接触前稳定点；target 是基地表面参考点。
  // 俯冲接触：APPROACH 使用板面前上方的入口；按位置和航向判定通过入口。
  bool dive_attack = false;
  TeamPoint dive_entry{};               // 俯冲入口：高于装甲板中心、位于其正前方。
  double dive_entry_tolerance_m = .20;  // 判定已飞过入口的水平容差，m。
  double dive_entry_height_m = .12;     // 判定已飞过入口的垂直容差，m。
  double dive_entry_yaw_tolerance_rad = .35; // 入口处允许的航向偏差，rad。
  // 接触阶段最长时间，0 表示不限制；超时成员退出接触轮次并参与返航。
  uint64_t contact_timeout_us = 0;
  // 俯冲接近阶段的最长时间，0 表示不限制。
  uint64_t approach_timeout_us = 0;
  // 航路点描述编队中心；每机再叠加 formation_offsets，保持队形。
  std::vector<TeamPoint> transit_waypoints;
  std::vector<TeamPoint> formation_offsets;
  bool explicit_formation_offsets = false; // false 时由起飞点相对质心的位置计算。
  bool sequential_landing = false; // 返航末段在编队位置等待，逐架对准起点并降落。
  std::vector<int> landing_order; // 为空时沿用接触顺序。
  double landing_alignment_tolerance_m = .025; // 下降前的水平位置容差，m。
  double landing_alignment_height_m = 0; // 对准平台上方高度，m；0 使用巡航高度。
  uint64_t landing_alignment_stable_us = 0; // 0 沿用 stable_us。
  bool landing_require_structural = false; // 下降前要求本地静态结构定位有效。
  std::vector<int> order{0, 1, 2, 3};
  double contact_speed_mps = .2, return_tolerance_m = .10; // 接触速度 m/s；到点 XY 容差 m。
  double route_tolerance_m = .10, route_speed_tolerance_mps = .30;
  bool fly_through_route = false; // true 允许去程穿越航点；返航路线仍要求速度降至 0.30 m/s。
  bool pipelined_contact = false; // true 在前机让出通道后切换轮次，不必等其完全停稳。
  bool visual_contact = false; // 相机闭环接触；地图点只负责到达搜索区。
  // 视觉模式下进入接触阶段前，本机到目标点必须小于该距离，单位 m。
  // 0 表示不限制，此时只要图像锁定就会立刻进入接触，末段行程全部由视觉伺服完成。
  double visual_entry_range_m = 0;
  double contact_exit_side = 0; // 接触航向左侧为正；0 按集结点决定待命侧。
  // true 时接触后直接飞往 completed_holding；false 时先经过 approach。
  bool retreat_holding_direct = false;
  // 已接触飞机的停留点；不配置则沿用 assembly。
  std::vector<TeamPoint> completed_holding;
  double home_ground_z_m = .5; // 起飞平台表面世界高度，m。
  double body_clearance_m = .0125; // 机体落地时中心到平台的距离，m。
  uint64_t freshness_us = 1000000, stable_us = 500000; // 状态有效期、条件连续成立时间，μs。
  // 阶段期限为 0 表示不设超时；loss_abort_us 是失效机退出屏障的等待时间，μs。
  uint64_t stage_timeout_us = 0, loss_abort_us = 5000000;
};
/** @brief runner 上报的单机状态；位置和速度来自定位器，armed/landed 来自飞控。 */
struct TeamVehicleStatus {
  int vehicle = -1;
  uint64_t sim_time_us = 0, impact_seq = 0; // 测量时刻 μs；冲击事件递增序号，避免重复计数。
  double x_m = 0, y_m = 0, z_m = 0;
  double vx_mps = 0, vy_mps = 0, vz_mps = 0, yaw_rad = 0;
  bool xy_valid = false, z_valid = false, ready = false, armed = false, landed = false;
  bool flow_valid = false; // 协调器解码时可用有效光流补充 xy_valid。
  bool visual_ready = false; // 近期图像锁定目标，且满足接触推进的对准条件。
  /** 本机无法继续任务，由 runner 对持续落地或结构卡住作时间过滤后置位。
   * @note 碰撞瞬间的短暂着地标志不触发失效；置位后退出路线屏障并独立降落。
   */
  bool unrecoverable = false;
  std::string xy_reason;
};
/**
 * @brief 任务状态机，只有满足当前阶段条件才跳转。
 * WAIT_READY→TAKEOFF→TRANSIT→ASSEMBLE 为全队准备与集结；
 * APPROACH→CONTACT→RETREAT 按 order 逐架循环；
 * RETURN_ALL→LAND_ALL→COMPLETE 为共同返航与落地。
 * ABORT_RETURN 是中止后的返航；ABORTED 也用于有跳过记录的部分任务终态。
 */
enum class TeamPhase { WAIT_READY, TAKEOFF, TRANSIT, ASSEMBLE, APPROACH, CONTACT,
                       RETREAT, RETURN_ALL, LAND_ALL, COMPLETE, ABORT_RETURN, ABORTED };
const char* teamPhaseName(TeamPhase phase);
/** @brief 给一架飞机的任务目标；runner 将目标位置误差转换为速度参考。 */
struct TeamCommand {
  int vehicle = -1, active_vehicle = -1;
  uint64_t sim_time_us = 0, sequence = 0;
  uint64_t contact_deadline_us = 0; // 当前接触轮次的仿真截止时刻，0 表示不限时。
  TeamPhase phase = TeamPhase::WAIT_READY;
  double target_x_m = 0, target_y_m = 0, target_z_m = 0, target_yaw_rad = 0;
  double contact_speed_mps = 0;
  // degraded 表示当前单机不能执行正常目标；skipped 表示当前不参与全队屏障。
  // disarm 在降落或终态收到新鲜着地反馈时置位；终态等待全队停机确认。
  bool paused = false, degraded = false, skipped = false, disarm = false, complete = false;
  std::string degraded_reason;
};
/**
 * @brief 单线程多机状态机，调用者先 updateStatus，再周期调用 step。
 * @note impact_seq 是 runner 根据 IMU 和接触位置识别的事件，物理真值由评估器另行核验。
 *       sim_time_us 推进任务，wall_time_us 检查现实经过时间；两者都用 μs，不能互减。
 */
class TeamMission {
 public:
  /** @brief 按配置分配机群状态；非法长度、机号或数值抛出 invalid_argument。 */
  explicit TeamMission(TeamMissionConfig config);
  /**
   * @brief 接收一个单机状态，并处理失联后的重新加入。
   * @param status 新状态，时间戳必须严格晚于该机上一包，冲击计数不可倒退。
   * @param wall_time_us 接收瞬间的单调时钟，μs，用于发现时钟冻结或链路停更。
   * @return true 表示已保存；false 表示越界、非有限数或旧包，保留原状态。
   */
  bool updateStatus(const TeamVehicleStatus& status, uint64_t wall_time_us);
  /**
   * @brief 推进阶段并生成 N 架的目标，不执行网络发送。
   * @param sim_time_us 当前共同仿真时刻，μs；内部不允许任务时钟倒退。
   * @param wall_time_us 当前单调时钟，μs。
   * @return 按机号排列的命令向量，元素数量等于 homes 的数量。
   */
  std::vector<TeamCommand> step(uint64_t sim_time_us, uint64_t wall_time_us);
  TeamPhase phase() const { return phase_; }
  int activeVehicle() const; // 接近/接触/后撤阶段返回轮次机号，其他阶段返回 -1。
  bool turnExhausted(int vehicle) const; // 该机是否已从接触轮次中剔除（失效或接触超时）。
  bool paused() const { return false; }
  bool aborted() const { return aborted_; }
  bool partial() const { return !skipped_.empty(); }
  const std::string& reason() const { return reason_; }
  const std::vector<int>& completed() const { return completed_; }
  const std::vector<int>& skipped() const { return skipped_; }
  const std::vector<TeamVehicleStatus>& statuses() const { return status_; }
  // 从有效估计首次离平台静置中心高度超过 3 cm 开始，到终态冻结；返回 s。
  double flightElapsedSeconds() const { return flight_start_?((flight_end_?flight_end_:clock_)-flight_start_)*1e-6:0; }
 private:
  bool statusFresh(int id, uint64_t sim, uint64_t wall) const;
  bool validForBarrier(int id, uint64_t sim, uint64_t wall) const;
  bool near(int id, const TeamPoint& point, bool xy = true) const;
  bool barrier(bool condition, uint64_t now);
  void enter(TeamPhase phase, uint64_t now);
  void skipUnrecoverable(uint64_t now);
  void abort(uint64_t now, const char* reason);
  TeamMissionConfig config_;
  std::vector<TeamVehicleStatus> status_;
  std::vector<bool> seen_, skipped_now_; // 是否收过首包、是否暂时排除在同步屏障之外。
  // failed_ 保留轮次失败；skipped_now_ 控制路线屏障。所有成员仍须确认着地停机。
  std::vector<bool> failed_;
  std::vector<uint64_t> received_, loss_start_;
  std::vector<TeamPoint> holds_; // 上次有效状态所在位置，单机失效时作为保持目标。
  TeamPhase phase_ = TeamPhase::WAIT_READY;
  uint64_t clock_ = 0, wall_clock_ = 0, phase_start_ = 0, stable_start_ = 0;
  uint64_t sequence_ = 0, impact_baseline_ = 0;
  uint64_t flight_start_ = 0, flight_end_ = 0;
  bool stable_ = false, aborted_ = false, retreat_assembly_ = false;
  int turn_ = 0; // order 中的下标，不是机号；机号由 order[turn_] 获取。
  int transit_index_ = 0, return_index_ = -1;
  size_t landing_index_ = 0;
  bool landing_descending_ = false;
  bool landing_approached_ = false;
  uint64_t landing_alignment_since_ = 0;
  std::vector<int> completed_;
  std::vector<int> skipped_; // 历史未完成轮次记录；恢复机仍可加入返航，记录不一定清除。
  std::string reason_;
};
}  // namespace companion
