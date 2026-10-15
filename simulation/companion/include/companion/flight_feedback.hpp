#pragma once
#include <chrono>

namespace companion {

struct FlightFeedbackState {
  bool heartbeat_fresh{false};
  bool landed_fresh{false};
  bool armed{true};
  bool landed{false};
  bool offboard{false};
  bool auto_land{false};
  bool failsafe_active{false};
  bool failsafe_latched{false};
  bool heartbeat_gap_latched{false};
};

/** @brief 飞控反馈按本机单调时钟过期；未知状态不能确认停机或着地。 */
class FlightFeedback {
 public:
  using Clock = std::chrono::steady_clock;
  using Time = Clock::time_point;

  void heartbeat(Time now, bool armed, bool offboard, bool auto_land, bool failsafe = false) {
    heartbeat_gap_latched_ = heartbeat_gap_latched_ || armedHeartbeatGap(now);
    // 重新连接或解锁状态变化后，等待新的着地消息，避免复用上一次飞行的反馈。
    if (!heartbeatFresh(now) || armed != armed_) landed_seen_ = false;
    heartbeat_rx_ = now;
    heartbeat_seen_ = true;
    armed_ = armed;
    offboard_ = offboard;
    auto_land_ = auto_land;
    failsafe_active_ = failsafe;
    failsafe_latched_ = failsafe_latched_ || (armed && failsafe);
  }

  void landed(Time now, bool on_ground) {
    if (!heartbeatFresh(now)) return;
    landed_rx_ = now;
    landed_seen_ = true;
    on_ground_ = on_ground;
  }

  FlightFeedbackState state(Time now) const {
    FlightFeedbackState result;
    result.heartbeat_fresh = heartbeatFresh(now);
    result.landed_fresh = result.heartbeat_fresh && landed_seen_ &&
        fresh(now, landed_rx_, std::chrono::milliseconds(500));
    result.armed = !result.heartbeat_fresh || armed_;
    result.landed = result.landed_fresh && on_ground_;
    result.offboard = result.heartbeat_fresh && offboard_;
    result.auto_land = result.heartbeat_fresh && auto_land_;
    result.failsafe_active = result.heartbeat_fresh && failsafe_active_;
    result.failsafe_latched = failsafe_latched_;
    result.heartbeat_gap_latched = heartbeat_gap_latched_ || armedHeartbeatGap(now);
    return result;
  }

 private:
  static bool fresh(Time now, Time received, Clock::duration limit) {
    return now >= received && now - received < limit;
  }
  bool heartbeatFresh(Time now) const {
    return heartbeat_seen_ && fresh(now, heartbeat_rx_, std::chrono::seconds(3));
  }
  bool armedHeartbeatGap(Time now) const {
    return heartbeat_seen_ && armed_ && now >= heartbeat_rx_ &&
        now - heartbeat_rx_ >= std::chrono::seconds(3);
  }
  Time heartbeat_rx_{}, landed_rx_{};
  bool heartbeat_seen_{false}, landed_seen_{false};
  bool armed_{true}, on_ground_{false}, offboard_{false}, auto_land_{false};
  bool failsafe_active_{false}, failsafe_latched_{false};
  bool heartbeat_gap_latched_{false};
};

}  // namespace companion
