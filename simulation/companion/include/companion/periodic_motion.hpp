/** @file periodic_motion.hpp
 * @brief 从已接受的视觉世界位置学习 Y 轴往复周期，并用独立周期验证外推。
 */
#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <vector>

namespace companion {
struct PeriodicPrediction {
  bool valid{false};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  double confidence{0};
  double horizon{0};  ///< 从最新观测起允许外推的总时长，单位 s。
};

/** 时间戳单位为微秒，位置单位为 m。调用方只传入关联已接受、位姿已补偿的视觉位置。
 * 四个完整周期包含至少九次转向；前三个周期训练，最近一个周期只验证。
 * 置信度是保守的质量分数，不是统计概率。运动改变后必须重新积累完整周期。
 */
class PeriodicMotion {
 public:
  bool observe(uint64_t stamp, const Eigen::Vector3d& point) {
    if (!stamp || stamp <= last_stamp_ || !point.allFinite()) return false;
    const Sample sample{stamp, point};
    if (last_stamp_ && seconds(stamp, last_stamp_) > kMaxGap) clear();
    if (ready_) {
      const auto expected = replay(stamp);
      bool changed = (point - expected.position).norm() > tolerance_;
      // 短窗口比较位移，提前识别停转和速度变化；单帧位置误差可能仍很小。
      auto previous = samples_.rbegin();
      while (previous != samples_.rend() && seconds(stamp, previous->stamp) < .12)
        ++previous;
      if (previous != samples_.rend()) {
        const double observed = point.y() - previous->point.y();
        const double expected_delta = expected.position.y() - replay(previous->stamp).position.y();
        changed = changed || std::abs(observed - expected_delta) > tolerance_
                  || (std::abs(observed) < .003 && std::abs(expected_delta) > .012);
      }
      if (changed) clear();
    }
    last_stamp_ = stamp;
    samples_.push_back(sample);
    if (!direction_) {
      if (!have_extreme_) { extreme_ = sample; have_extreme_ = true; }
      const double delta = point.y() - extreme_.point.y();
      if (std::abs(delta) >= kTurnExcursion) {
        direction_ = delta > 0 ? 1 : -1;
        extreme_ = sample;
      }
    } else {
      const double delta = direction_ * (point.y() - extreme_.point.y());
      if (delta >= 0) extreme_ = sample;
      else if (-delta >= kTurnExcursion) {
        turns_.push_back(extreme_);
        if (turns_.size() > 9) turns_.pop_front();
        direction_ = -direction_;
        extreme_ = sample;
        if (!ready_) learn();
      }
    }
    // 限定内存及可学习历史；不足以保存四周期时保持无效。
    while (samples_.size() > 8192 || seconds(stamp, samples_.front().stamp) > 120)
      samples_.pop_front();
    while (!turns_.empty() && turns_.front().stamp < samples_.front().stamp)
      turns_.pop_front();
    return true;
  }

  /** 有效时间窗内重放周期，X/Z 保持训练均值，速度仅含 Y 分量。
   * 不允许查询早于最新曝光的时刻；缺测超过窗口时立即返回默认无效结果。
   */
  PeriodicPrediction predict(uint64_t now) const {
    if (!ready_ || now < last_stamp_) return {};
    const double age = seconds(now, last_stamp_);
    if (age > horizon_) return {};
    auto result = replay(now);
    result.valid = true;
    result.confidence = confidence_ * (1 - .15 * age / horizon_);
    result.horizon = horizon_;
    return result;
  }

 private:
  struct Sample { uint64_t stamp; Eigen::Vector3d point; };
  static constexpr double kMaxGap = .25;
  static constexpr double kTurnExcursion = .006;
  static constexpr std::size_t kBins = 128;
  static double seconds(uint64_t later, uint64_t earlier) {
    return static_cast<double>(later - earlier) * 1e-6;
  }
  void clear() {
    samples_.clear(); turns_.clear(); ready_ = false;
    direction_ = 0; have_extreme_ = false;
  }
  Eigen::Vector3d interpolate(uint64_t stamp) const {
    auto right = std::lower_bound(samples_.begin(), samples_.end(), stamp,
        [](const Sample& sample, uint64_t time) { return sample.stamp < time; });
    if (right == samples_.begin()) return right->point;
    if (right == samples_.end()) return samples_.back().point;
    const auto left = std::prev(right);
    const double ratio = seconds(stamp, left->stamp) / seconds(right->stamp, left->stamp);
    return left->point + ratio * (right->point - left->point);
  }
  PeriodicPrediction replay(uint64_t stamp) const {
    // 有符号相对时间支持监测窗口位于模板锚点之前，避免 uint64_t 下溢。
    const double elapsed = stamp >= anchor_ ? seconds(stamp, anchor_) : -seconds(anchor_, stamp);
    double phase = std::fmod(elapsed / period_, 1.0);
    if (phase < 0) phase += 1;
    const double bin = phase * kBins;
    const auto index = std::min(kBins - 1, static_cast<std::size_t>(bin));
    const double ratio = bin - index;
    PeriodicPrediction result;
    result.position = fixed_;
    result.position.y() = shape_[index] + ratio * (shape_[index + 1] - shape_[index]);
    result.velocity.y() = (shape_[index + 1] - shape_[index]) * kBins / period_;
    return result;
  }
  void learn() {
    if (turns_.size() < 9 || samples_.front().stamp > turns_.front().stamp) return;
    std::array<double, 4> durations{};
    for (std::size_t i = 0; i < 4; ++i)
      durations[i] = seconds(turns_[2 * i + 2].stamp, turns_[2 * i].stamp);
    period_ = (durations[0] + durations[1] + durations[2]) / 3;
    if (period_ < .4 || period_ > 20) return;
    double instability = 0;
    for (double duration : durations)
      instability = std::max(instability, std::abs(duration / period_ - 1));
    if (instability > .025) return;

    // 相位、周期及形状全部只用前三周期；验证周期不能反向参与拟合。
    anchor_ = turns_[6].stamp;
    fixed_.setZero();
    for (std::size_t bin = 0; bin <= kBins; ++bin) {
      Eigen::Vector3d mean = Eigen::Vector3d::Zero();
      for (std::size_t cycle = 0; cycle < 3; ++cycle) {
        const uint64_t start = turns_[2 * cycle].stamp;
        const uint64_t length = turns_[2 * cycle + 2].stamp - start;
        mean += interpolate(start + length * bin / kBins) / 3;
      }
      shape_[bin] = mean.y();
      fixed_ += mean / static_cast<double>(kBins + 1);
    }
    shape_[kBins] = shape_[0];
    const auto bounds = std::minmax_element(shape_.begin(), shape_.end());
    const double span = *bounds.second - *bounds.first;
    if (span < .05) return;
    tolerance_ = .008 + .025 * span;
    double squared_error = 0, validation_error = 0, max_error = 0, max_gap = 0;
    std::size_t training_count = 0, validation_count = 0;
    uint64_t previous = turns_[0].stamp;
    for (const auto& sample : samples_) {
      if (sample.stamp < turns_[0].stamp || sample.stamp > turns_[8].stamp) continue;
      max_gap = std::max(max_gap, seconds(sample.stamp, previous));
      previous = sample.stamp;
      const double error = (sample.point - replay(sample.stamp).position).norm();
      max_error = std::max(max_error, error);
      if (sample.stamp < anchor_) { squared_error += error * error; ++training_count; }
      else { validation_error += error * error; ++validation_count; }
    }
    if (!training_count || validation_count < 32 || max_gap > std::min(kMaxGap, period_ / 32)) return;
    const double residual = std::sqrt(squared_error / training_count);
    const double held_out = std::sqrt(validation_error / validation_count);
    if (residual > tolerance_ * .5 || held_out > tolerance_ * .5 || max_error > tolerance_) return;
    confidence_ = 1 - .1 * instability / .025
                    - .1 * residual / (tolerance_ * .5) - .1 * held_out / (tolerance_ * .5);
    if (confidence_ < .85) return;
    // 预测窗由周期稳定性控制；采样间隔只用于检验训练数据是否连续。
    horizon_ = std::min(.2, period_ * .12);
    // 转向确认需要少量后续观测，它们也必须符合已冻结的模板。
    for (const auto& sample : samples_)
      if (sample.stamp > turns_[8].stamp &&
          (sample.point - replay(sample.stamp).position).norm() > tolerance_) return;
    ready_ = true;
  }
  std::deque<Sample> samples_, turns_;
  Sample extreme_{0, Eigen::Vector3d::Zero()};
  uint64_t last_stamp_{0}, anchor_{0};
  int direction_{0};
  bool have_extreme_{false}, ready_{false};
  std::array<double, kBins + 1> shape_{};
  Eigen::Vector3d fixed_{Eigen::Vector3d::Zero()};
  double period_{0}, tolerance_{0}, confidence_{0}, horizon_{0};
};
}  // namespace companion
