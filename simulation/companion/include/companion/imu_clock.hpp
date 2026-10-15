#pragma once
#include <cstdint>
#include <limits>
#include <optional>

namespace companion {

enum class ImuClockDomain { FirstArrival, Gazebo };

/** @brief 将 IMU 源时刻映射到观测时钟，保留 Gazebo 锁步采样时间。
 * @note FirstArrival 用首包建立相对纪元；Gazebo 要求飞控使用世界锁步时钟。
 *       与最新观测相差超过 500 ms 的样本不进入估计器。
 */
class ImuClockMapper {
 public:
  explicit ImuClockMapper(ImuClockDomain domain) : domain_(domain) {}
  std::optional<uint64_t> map(uint64_t source_us, uint64_t observation_us) {
    if (!source_us || !observation_us) return std::nullopt;
    uint64_t mapped = source_us;
    if (domain_ == ImuClockDomain::FirstArrival) {
      if (!source_origin_) {
        source_origin_ = source_us;
        observation_origin_ = observation_us;
      }
      if (source_us < source_origin_) return std::nullopt;
      const uint64_t delta = source_us - source_origin_;
      if (delta > std::numeric_limits<uint64_t>::max() - observation_origin_) return std::nullopt;
      mapped = observation_origin_ + delta;
    }
    const uint64_t gap = mapped >= observation_us ? mapped - observation_us : observation_us - mapped;
    return gap <= 500000 ? std::optional<uint64_t>(mapped) : std::nullopt;
  }

 private:
  ImuClockDomain domain_;
  uint64_t source_origin_{0}, observation_origin_{0};
};

}  // namespace companion
