#include "companion/imu_attitude.hpp"
#include <cassert>
#include <cmath>

using namespace companion;
namespace {
const double pi = std::acos(-1.0);
Eigen::Quaterniond orientation(double roll, double pitch, double yaw) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
}
void close(const Eigen::Quaterniond& a, const Eigen::Quaterniond& b) {
  assert((a.toRotationMatrix() - b.toRotationMatrix()).norm() < 1e-12);
}
void asynchronousRotation(bool attitude_first) {
  ImuAttitudeHistory history;
  const auto origin = orientation(.4, -.3, 3.1);
  const Eigen::Vector3d world_rate(.8, .6, 1.2), body_rate = origin.conjugate() * world_rate;
  const auto truth = [&](uint64_t stamp) {
    return Eigen::Quaterniond(Eigen::AngleAxisd(world_rate.norm() *
        (static_cast<double>(stamp) - 1000000) * 1e-6, world_rate.normalized())) * origin;
  };
  Eigen::Quaterniond held = origin;
  double held_error_squared = 0, aligned_error_squared = 0;
  uint64_t samples = 0;
  for (uint64_t tick = 0; tick < 1000; ++tick) {
    const uint64_t publish = 1000000 + tick * 10000, imu = publish - 3000;
    const auto update = [&] {
      if (tick % 2 == 0) { held = truth(publish); assert(history.observe(publish, held)); }
    };
    if (attitude_first) update();
    if (const auto aligned = history.at(imu, body_rate)) {
      const auto actual = truth(imu);
      const Eigen::Vector3d gravity(0, 0, 9.80665), specific_body = actual.conjugate() * gravity;
      held_error_squared += (held * specific_body - gravity).squaredNorm();
      aligned_error_squared += (aligned->body_to_world * specific_body - gravity).squaredNorm();
      assert(aligned->body_to_world.angularDistance(actual) < 1e-12);
      ++samples;
    }
    if (!attitude_first) update();
  }
  assert(samples >= 998);
  assert(std::sqrt(held_error_squared / samples) > .03);
  assert(std::sqrt(aligned_error_squared / samples) < 1e-12);
}
}
int main() {
  assert(attitudeBootTimeUs(1234, 1234999) == 1234000);
  const uint64_t wrap = (uint64_t{1} << 32) * 1000;
  assert(attitudeBootTimeUs(3, wrap + 3999) == wrap + 3000);
  assert(attitudeBootTimeUs(0xfffffffeU, wrap + 3000) == wrap - 2000);
  assert(attitudeBootTimeUs(3, wrap - 2000) == wrap + 3000);
  assert(!attitudeBootTimeUs(0xfffffffeU, 3000));
  assert(attitudeBootTimeUs(0xffffffffU, 0) == wrap - 1000);
  assert(!attitudeBootTimeUs(static_cast<uint32_t>(std::numeric_limits<uint64_t>::max()/1000) + 1,
      std::numeric_limits<uint64_t>::max()));

  ImuAttitudeHistory history;
  const auto initial = orientation(.4, -.2, 3.1);
  assert(!history.at(1000000));
  assert(!history.observe(0, initial));
  auto invalid = initial; invalid.x() = std::numeric_limits<double>::quiet_NaN();
  assert(!history.observe(1000000, invalid));
  assert(!history.observe(1000000, Eigen::Quaterniond(0, 0, 0, 0)));
  assert(history.observe(1000000, initial));
  assert(!history.observe(1000000, orientation(0, 0, 0)));
  assert(!history.observe(999999, initial));
  assert(!history.at(999999));
  const auto exact = history.at(1000000);
  assert(exact && exact->mode == ImuAttitudeMode::Exact);
  close(exact->body_to_world, initial);
  assert(!history.at(1000001));
  const auto still = history.at(1100000, Eigen::Vector3d::Zero());
  assert(still && still->mode == ImuAttitudeMode::Extrapolated);
  close(still->body_to_world, initial);
  assert(!history.at(1100001, Eigen::Vector3d::Zero()));
  assert(!history.at(1000001, Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity())));

  // 横滚 90° 后绕机体 Y 轴转动，对应世界 Z 轴；不能把机体角速度当欧拉角导数。
  ImuAttitudeHistory tilted;
  assert(tilted.observe(1000000, orientation(pi/2, 0, 0)));
  const auto propagated = tilted.at(1100000, Eigen::Vector3d(0, pi/2, 0));
  assert(propagated);
  close(propagated->body_to_world, orientation(pi/2, 0, pi/20));

  // 三轴转动跨越航向包角，且后一个姿态在 IMU 采样之后。
  const auto following = orientation(-.1, .3, -3.0);
  assert(history.observe(1020000, following));
  const auto middle = history.at(1010000);
  assert(middle && middle->mode == ImuAttitudeMode::Interpolated);
  assert(middle->begin_us == 1000000 && middle->end_us == 1020000);
  assert(std::abs(initial.angularDistance(middle->body_to_world) -
      following.angularDistance(middle->body_to_world)) < 1e-12);
  close(history.at(1020000)->body_to_world, following);
  auto negative = following; negative.coeffs() *= -1;
  assert(history.observe(1040000, negative));
  close(history.at(1030000)->body_to_world, following);
  assert(history.observe(1140001, initial));
  assert(!history.at(1100000, Eigen::Vector3d::Zero()));
  assert(history.at(1140001));

  ImuAttitudeHistory ring;
  for (uint64_t i = 1; i <= 20; ++i) assert(ring.observe(i * 20000, initial));
  assert(ring.latestTimeUs() == 400000);
  assert(!ring.at(99999));
  close(ring.at(100000)->body_to_world, initial);
  close(ring.at(390000)->body_to_world, initial);
  ring = ImuAttitudeHistory{};
  assert(!ring.latestTimeUs() && !ring.at(400000));
  asynchronousRotation(true);
  asynchronousRotation(false);
}
