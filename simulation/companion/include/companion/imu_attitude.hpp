#pragma once
#include <Eigen/Geometry>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>

namespace companion {

/** @brief 将回绕的毫秒启动时钟展开到参考微秒时钟最近的周期。 */
inline std::optional<uint64_t> attitudeBootTimeUs(uint32_t boot_ms, uint64_t reference_us) {
  if (!reference_us) return static_cast<uint64_t>(boot_ms) * 1000;
  const uint32_t delta = boot_ms - static_cast<uint32_t>(reference_us / 1000);
  const int64_t delta_ms = delta <= 0x7fffffffU ? static_cast<int64_t>(delta) :
      static_cast<int64_t>(delta) - (int64_t{1} << 32);
  const uint64_t base = reference_us - reference_us % 1000;
  const uint64_t distance = static_cast<uint64_t>(delta_ms < 0 ? -delta_ms : delta_ms) * 1000;
  if (delta_ms < 0) return base >= distance ? std::optional<uint64_t>(base - distance) : std::nullopt;
  return distance <= std::numeric_limits<uint64_t>::max() - base ?
      std::optional<uint64_t>(base + distance) : std::nullopt;
}

enum class ImuAttitudeMode { Exact, Interpolated, Extrapolated };
struct ImuAttitude {
  Eigen::Quaterniond body_to_world{Eigen::Quaterniond::Identity()};
  uint64_t begin_us{0}, end_us{0};
  ImuAttitudeMode mode{ImuAttitudeMode::Exact};
};

/** @brief 按姿态源时间保存四元数，在 IMU 采样时刻插值或用机体系角速度短时外推。
 * @note 角速度与四元数使用相同机体系；外推假设该短时间段内角速度恒定。
 *       插值间隔及外推时长均不超过 100 ms。重复、乱序及非有限姿态不进入历史。
 */
class ImuAttitudeHistory {
 public:
  bool observe(uint64_t source_us, const Eigen::Quaterniond& rotation) {
    const double norm = rotation.norm();
    if (!source_us || (size_ && source_us <= newest().stamp) ||
        !rotation.coeffs().allFinite() || !std::isfinite(norm) || norm < 1e-12) return false;
    samples_[(begin_ + size_) % samples_.size()] = {source_us, rotation.normalized()};
    if (size_ < samples_.size()) ++size_;
    else begin_ = (begin_ + 1) % samples_.size();
    return true;
  }

  std::optional<ImuAttitude> at(uint64_t imu_us,
      const std::optional<Eigen::Vector3d>& body_rate_radps = std::nullopt) const {
    if (!imu_us || !size_ || imu_us < samples_[begin_].stamp) return std::nullopt;
    const auto& latest = newest();
    if (imu_us >= latest.stamp) {
      ImuAttitude result{latest.rotation, latest.stamp, latest.stamp, ImuAttitudeMode::Exact};
      const uint64_t gap = imu_us - latest.stamp;
      if (!gap) return result;
      if (gap > kMaxGapUs || !body_rate_radps || !body_rate_radps->allFinite()) return std::nullopt;
      const double rate = body_rate_radps->norm(), angle = rate * (gap * 1e-6);
      if (!std::isfinite(rate) || !std::isfinite(angle)) return std::nullopt;
      if (rate > 0) result.body_to_world = (latest.rotation *
          Eigen::Quaterniond(Eigen::AngleAxisd(angle, *body_rate_radps / rate))).normalized();
      result.mode = ImuAttitudeMode::Extrapolated;
      return result;
    }
    for (std::size_t i = 0; i < size_; ++i) {
      const auto& right = samples_[(begin_ + i) % samples_.size()];
      if (imu_us == right.stamp) return ImuAttitude{right.rotation, right.stamp, right.stamp, ImuAttitudeMode::Exact};
      if (imu_us < right.stamp) {
        const auto& left = samples_[(begin_ + i - 1) % samples_.size()];
        if (right.stamp - left.stamp > kMaxGapUs) return std::nullopt;
        const double fraction = static_cast<double>(imu_us - left.stamp) / (right.stamp - left.stamp);
        return ImuAttitude{left.rotation.slerp(fraction, right.rotation).normalized(),
            left.stamp, right.stamp, ImuAttitudeMode::Interpolated};
      }
    }
    return std::nullopt;
  }

  uint64_t latestTimeUs() const { return size_ ? newest().stamp : 0; }

 private:
  struct Sample { uint64_t stamp{0}; Eigen::Quaterniond rotation{Eigen::Quaterniond::Identity()}; };
  const Sample& newest() const { return samples_[(begin_ + size_ - 1) % samples_.size()]; }
  static constexpr uint64_t kMaxGapUs = 100000;
  std::array<Sample, 16> samples_{};
  std::size_t begin_{0}, size_{0};
};
}  // namespace companion
