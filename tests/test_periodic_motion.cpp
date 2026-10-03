#include "companion/periodic_motion.hpp"

#include <cassert>
#include <cmath>

namespace {
constexpr double kPi = 3.14159265358979323846;

Eigen::Vector3d sinusoid(double t, double amplitude = .2, double period = 1.0) {
  return Eigen::Vector3d(11.0, amplitude * std::sin(2.0 * kPi * t / period), 1.4);
}

Eigen::Vector3d triangle(double t, double amplitude = .2, double period = 1.0) {
  const double phase = std::fmod(t / period, 1.0);
  const double y = amplitude * (phase < .5 ? 4.0 * phase - 1.0 : 3.0 - 4.0 * phase);
  return Eigen::Vector3d(11.0, y, 1.4);
}

void feed(companion::PeriodicMotion& model, double end, double step = .01,
          double amplitude = .2, double period = 1.0) {
  for (double t = step; t <= end + 1e-9; t += step) {
    assert(model.observe(static_cast<uint64_t>(std::llround(t * 1e6)),
                         sinusoid(t, amplitude, period)));
  }
}

void feed_triangle(companion::PeriodicMotion& model, double end, double step = .01) {
  for (double t = step; t <= end + 1e-9; t += step) {
    const double noise = .0015 * std::sin(17.0 * t) + .0008 * std::cos(31.0 * t);
    auto point = triangle(t);
    point.y() += noise;
    assert(model.observe(static_cast<uint64_t>(std::llround(t * 1e6)), point));
  }
}
}  // namespace

int main() {
  companion::PeriodicMotion model;
  feed(model, 5.2);
  const auto prediction = model.predict(5'300'000);
  assert(prediction.valid);
  assert(prediction.confidence > .8);
  assert(prediction.horizon > .05);
  const auto expected = sinusoid(5.30);
  assert(std::abs(prediction.position.y() - expected.y()) < .045);
  assert(std::abs(prediction.velocity.y() -
                  .2 * 2.0 * kPi * std::cos(2.0 * kPi * 5.30)) < .35);

  companion::PeriodicMotion noisy_triangle;
  feed_triangle(noisy_triangle, 5.2);
  const auto triangle_prediction = noisy_triangle.predict(5'300'000);
  assert(triangle_prediction.valid);
  assert(triangle_prediction.confidence > .8);
  assert(std::abs(triangle_prediction.position.y() - triangle(5.30).y()) < .035);
  assert(std::abs(triangle_prediction.velocity.y()) > .65);

  companion::PeriodicMotion irregular;
  for (uint64_t i = 0; i < 700; ++i) {
    const double t = (i + 1) * .01;
    const double y = .15 * std::sin(2.0 * kPi * t * (.7 + .002 * i));
    assert(irregular.observe((i + 1) * 10'000,
                             Eigen::Vector3d(11.0, y, 1.4)));
  }
  assert(!irregular.predict(7'100'000).valid);

  companion::PeriodicMotion changed;
  feed(changed, 5.2);
  assert(changed.predict(5'250'000).valid);
  for (uint64_t i = 521; i <= 580; ++i) {
    const double t = i * .01;
    assert(changed.observe(i * 10'000,
                           sinusoid(t, .30, 1.0)));
  }
  assert(!changed.predict(5'800'000).valid);

  companion::PeriodicMotion stopped;
  feed(stopped, 5.2);
  assert(stopped.predict(5'250'000).valid);
  for (uint64_t i = 521; i <= 560; ++i) {
    assert(stopped.observe(i * 10'000, Eigen::Vector3d(11.0, .2, 1.4)));
  }
  assert(!stopped.predict(5'600'000).valid);

  companion::PeriodicMotion gap;
  feed(gap, 5.2);
  assert(gap.predict(5'250'000).valid);
  assert(gap.observe(5'600'000, sinusoid(5.6)));
  assert(!gap.predict(5'600'000).valid);

  companion::PeriodicMotion old_packet;
  feed(old_packet, 5.2);
  assert(old_packet.predict(5'250'000).valid);
  assert(!old_packet.observe(5'190'000, sinusoid(5.19)));
  assert(old_packet.predict(5'250'000).valid);
}
