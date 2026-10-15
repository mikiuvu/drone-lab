#pragma once
#include <chrono>
#include <cmath>
#include <cstdint>

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
  bool setpoint_gap_latched{false};
  bool attitude_inverted_latched{false};
};

/** @brief 飞控反馈按本机单调时钟过期；未知状态不能确认停机或着地。 */
class FlightFeedback {
 public:
  using Clock = std::chrono::steady_clock;
  using Time = Clock::time_point;

  void heartbeat(Time now, bool armed, bool offboard, bool auto_land, bool failsafe = false) {
    heartbeat_gap_latched_ = heartbeat_gap_latched_ || armedHeartbeatGap(now);
    setpoint_gap_latched_ = setpoint_gap_latched_ || armedSetpointGap(now);
    // 重新连接或解锁状态变化后，等待新的着地消息，避免复用上一次飞行的反馈。
    if (!heartbeatFresh(now) || armed != armed_) {
      landed_seen_ = false;
      attitude_seen_ = inversion_pending_ = false;
    }
    heartbeat_rx_ = now;
    heartbeat_seen_ = true;
    armed_ = armed;
    offboard_ = offboard;
    auto_land_ = auto_land;
    failsafe_active_ = failsafe;
    failsafe_latched_ = failsafe_latched_ || (armed && failsafe);
    if (!armed) setpoint_monitored_ = false;
  }

  /** @brief 成功提交 OFFBOARD 参考后记录本机时刻；已解锁控制流中断 500 ms 后锁存。 */
  void setpointSent(Time now) {
    setpoint_gap_latched_ = setpoint_gap_latched_ || armedSetpointGap(now);
    if (heartbeat_seen_ && armed_ && offboard_) setpoint_monitored_ = true;
    setpoint_tx_ = now;
  }

  void landed(Time now, bool on_ground) {
    if (!heartbeatFresh(now)) return;
    landed_rx_ = now;
    landed_seen_ = true;
    on_ground_ = on_ground;
  }

  /** @brief 已解锁时，机体上轴朝下连续 500 ms 后锁存；不作为着地确认。
   * @param source_us 姿态源时间 us，必须递增；body_up_cosine 为机体上轴与世界上轴的点积。
   * @note 源时间与收包墙钟须同时覆盖 500 ms，源或收包间隔超过 250 ms 时重新计时。
   *       排队历史、重复样本、未解锁姿态不能代替连续的新鲜输入。
   */
  void attitude(Time now, uint64_t source_us, double body_up_cosine) {
    if (!heartbeatFresh(now) || !armed_) {
      attitude_seen_ = inversion_pending_ = false;
      return;
    }
    if (!source_us || !std::isfinite(body_up_cosine) ||
        body_up_cosine < -1 || body_up_cosine > 1) {
      inversion_pending_ = false;
      return;
    }
    if (attitude_seen_ && source_us <= attitude_source_us_) return;
    const bool gap = !attitude_seen_ || now < attitude_rx_ ||
        now - attitude_rx_ > std::chrono::milliseconds(250) ||
        source_us - attitude_source_us_ > 250000;
    attitude_seen_ = true;
    attitude_source_us_ = source_us;
    attitude_rx_ = now;
    if (body_up_cosine >= 0) {
      inversion_pending_ = false;
      return;
    }
    if (!inversion_pending_ || gap) {
      inversion_pending_ = true;
      inversion_source_begin_us_ = source_us;
      inversion_begin_ = now;
    }
    attitude_inverted_latched_ = attitude_inverted_latched_ ||
        (source_us - inversion_source_begin_us_ >= 500000 &&
         now - inversion_begin_ >= std::chrono::milliseconds(500));
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
    result.setpoint_gap_latched = setpoint_gap_latched_ || armedSetpointGap(now);
    result.attitude_inverted_latched = attitude_inverted_latched_;
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
  bool armedSetpointGap(Time now) const {
    return setpoint_monitored_ && now >= setpoint_tx_ &&
        now - setpoint_tx_ >= std::chrono::milliseconds(500);
  }
  Time heartbeat_rx_{}, landed_rx_{}, setpoint_tx_{};
  bool heartbeat_seen_{false}, landed_seen_{false};
  bool armed_{true}, on_ground_{false}, offboard_{false}, auto_land_{false};
  bool failsafe_active_{false}, failsafe_latched_{false};
  bool heartbeat_gap_latched_{false};
  bool setpoint_monitored_{false}, setpoint_gap_latched_{false};
  Time attitude_rx_{}, inversion_begin_{};
  uint64_t attitude_source_us_{0}, inversion_source_begin_us_{0};
  bool attitude_seen_{false}, inversion_pending_{false}, attitude_inverted_latched_{false};
};

}  // namespace companion
