/** @file latest_tof.hpp
 * @brief 每个方向只保留最新深度帧，每次取出最早的一帧以分散融合计算。
 */
#pragma once
#include "companion/types.hpp"
#include <optional>
namespace companion {
class LatestTof {
 public:
  bool submit(const TofFrame& frame) {
    const int sensor = frame.sensor_index;
    if (sensor < 0 || sensor >= 3 || !frame.sim_time_us || frame.sim_time_us <= seen_[sensor]) return false;
    seen_[sensor] = frame.sim_time_us; ++submitted;
    if (pending_[sensor]) ++replaced;
    pending_[sensor] = frame; return true;
  }
  std::optional<TofFrame> take() {
    int selected = -1;
    for (int i = 0; i < 3; ++i)
      if (pending_[i] && (selected < 0 || pending_[i]->sim_time_us < pending_[selected]->sim_time_us)) selected = i;
    if (selected < 0) return std::nullopt;
    auto frame = std::move(pending_[selected]); pending_[selected].reset(); ++taken; return frame;
  }
  uint64_t submitted{0}, replaced{0}, taken{0};
 private:
  std::array<std::optional<TofFrame>, 3> pending_;
  std::array<uint64_t, 3> seen_{};
};
}  // namespace companion
