#include "companion/periodic_motion.hpp"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <vector>

namespace {
constexpr double kPi = 3.14159265358979323846;

Eigen::Vector3d sinusoid(double t, double amplitude = .2, double period = 1.0) {
  return Eigen::Vector3d(11.0, amplitude * std::sin(2.0 * kPi * t / period), 1.4);
}

/** 滑轨的真实形状：匀速三角波。周期内前一半上行、后一半下行。 */
Eigen::Vector3d triangle(double t, double amplitude = .28, double period = 5.6) {
  const double phase = std::fmod(t / period, 1.0);
  const double y = amplitude * (phase < .5 ? 4.0 * phase - 1.0 : 3.0 - 4.0 * phase);
  return Eigen::Vector3d(11.0, y, 1.4);
}

using Shape = Eigen::Vector3d (*)(double, double, double);

void feed(companion::PeriodicMotion& model, double begin, double end, Shape shape,
          double amplitude, double period, double step = .01, double noise = 0) {
  for (double t = begin + step; t <= end + 1e-9; t += step) {
    auto point = shape(t, amplitude, period);
    if (noise > 0) point.y() += noise * (std::sin(97.0 * t) + .6 * std::cos(151.0 * t));
    assert(model.observe(static_cast<uint64_t>(std::llround(t * 1e6)), point));
  }
}
}  // namespace

int main() {
  // 正弦波：学成后外推 0.1 s 的落点误差应在毫米级。
  // 1 s 周期按 5 ms 喂入，单周期 200 个相位，与真机（50 Hz 喂 5.6 s 周期，约 280 个）同量级。
  companion::PeriodicMotion model;
  model.setMinSpan(4.0);
  feed(model, 0, 5.2, sinusoid, .2, 1.0, .005);
  assert(model.ready());
  assert(std::abs(model.period_estimate() - 1.0) < .02);
  assert(model.explained() > .95);
  const auto prediction = model.predict(5'300'000);
  assert(prediction.valid);
  assert(prediction.confidence > .8);
  assert(prediction.horizon > .05);
  assert(std::abs(prediction.position.y() - sinusoid(5.30).y()) < .03);
  assert(std::abs(prediction.velocity.y() - .2 * 2.0 * kPi * std::cos(2.0 * kPi * 5.30)) < .35);

  // 滑轨的真实形状（三角波）加实测量级的噪声，周期仍必须是 5.6 s：
  // 真周期和它的整数倍都能把波形折到重合，判据必须取最小的那一个。
  companion::PeriodicMotion rail;
  rail.setMinSpan(11.0);
  feed(rail, 0, 20.0, triangle, .28, 5.6, .02, .02);
  assert(rail.ready());
  assert(std::abs(rail.period_estimate() - 5.6) < .1);
  assert(rail.explained() > .8);
  const auto rail_prediction = rail.predict(20'050'000);
  assert(rail_prediction.valid);
  assert(std::abs(rail_prediction.position.y() - triangle(20.05).y()) < .05);

  // 白噪声不是周期信号，不得学成。
  companion::PeriodicMotion irregular;
  irregular.setMinSpan(4.0);
  uint32_t state = 12345;
  for (uint64_t i = 1; i <= 700; ++i) {
    state = state * 1664525u + 1013904223u;
    const double y = .2 * (static_cast<double>(state) / 4294967296.0 * 2 - 1);
    assert(irregular.observe(i * 10'000, Eigen::Vector3d(11.0, y, 1.4)));
  }
  assert(!irregular.ready());
  assert(!irregular.predict(7'100'000).valid);

  // 板面停住后必须停用模型：重学连续失败超过两个周期，说明模板已不符实际。
  companion::PeriodicMotion stopped;
  stopped.setMinSpan(4.0);
  feed(stopped, 0, 5.2, sinusoid, .2, 1.0, .005);
  assert(stopped.ready());
  for (uint64_t i = 521; i <= 900; ++i)
    assert(stopped.observe(i * 10'000, Eigen::Vector3d(11.0, .2, 1.4)));
  assert(!stopped.ready());
  assert(!stopped.predict(9'100'000).valid);

  // 乱序样本被拒绝，且不影响已有模型。
  companion::PeriodicMotion old_packet;
  old_packet.setMinSpan(4.0);
  feed(old_packet, 0, 5.2, sinusoid, .2, 1.0, .005);
  assert(old_packet.ready());
  assert(!old_packet.observe(5'190'000, sinusoid(5.19)));
  assert(old_packet.ready());
  assert(old_packet.predict(5'250'000).valid);

  // 观测窗不足两个完整周期时不得学成，失败原因要能说清是窗口太短。
  companion::PeriodicMotion short_window;
  short_window.setMinSpan(11.0);
  feed(short_window, 0, 8.0, triangle, .28, 5.6, .02);
  assert(!short_window.ready());
  assert(short_window.learn_reject() == "观测窗太短");

  // 跨机共享：滑轨全周期 5.6 s，学成一个模型要 11 s 以上的连续观测，而单机一次
  // 回合的视觉窗只有 9–10 s。两台各看一段，把队友的样本合进来才够——共享的是
  // 原始观测，因为谁都学不成，也就没有「学成的模型」可以共享。
  companion::PeriodicMotion second;
  second.setMinSpan(11.0);
  feed(second, 8.0, 16.0, triangle, .28, 5.6, .02);
  assert(!second.ready());
  const auto exported = second.exportSamples(400);
  assert(!exported.empty());
  assert(exported.front().stamp < exported.back().stamp);
  assert(exported.back().stamp > 15'000'000 && exported.back().stamp <= 16'000'000);
  // 只导出本机样本，避免转发形成放大回路。
  assert(second.merged_count() == 0);
  assert(short_window.merge(exported) > 0);
  assert(short_window.window_span() > 15.0);
  assert(short_window.ready());
  assert(std::abs(short_window.period_estimate() - 5.6) < .1);
  assert(std::abs(short_window.predict(16'050'000).position.y() - triangle(16.05).y()) < .05);

  // 同一时刻的样本只保留本机的，不重复插入。
  const auto again = short_window.merge(exported);
  assert(again == 0);

  // 本机观测不足以导出时不广播空报文。
  companion::PeriodicMotion empty;
  assert(empty.exportSamples(192).empty());
  assert(empty.merge({}) == 0);
  assert(!empty.ready());
  assert(!empty.predict(1'000'000).valid);
}
