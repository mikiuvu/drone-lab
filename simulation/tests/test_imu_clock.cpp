#include "companion/imu_clock.hpp"
#include <cassert>
#include <initializer_list>

using namespace companion;

int main() {
  for (uint64_t delay : {uint64_t{1000}, uint64_t{15000}, uint64_t{20000}}) {
    ImuClockMapper clock(ImuClockDomain::Gazebo);
    assert(clock.map(8000000, 8000000 + delay) == 8000000);
    assert(clock.map(8004000, 8010000) == 8004000);
    assert(clock.map(8008000, 8001000) == 8008000);
    assert(!clock.map(9000000, 8001000));
    assert(!clock.map(8000000, 9000000));
    assert(clock.map(8500000, 8000000) == 8500000);
    assert(!clock.map(8500001, 8000000));
    assert(!clock.map(0, 8000000));
    assert(!clock.map(8000000, 0));
  }
  ImuClockMapper relative(ImuClockDomain::FirstArrival);
  assert(!relative.map(90000000, 0));
  assert(relative.map(90000000, 8000000) == 8000000);
  assert(relative.map(90004000, 8010000) == 8004000);
  assert(!relative.map(89999999, 8000000));
  assert(!relative.map(91000000, 8000000));
  assert(relative.map(90008000, 8010000) == 8008000);
  const auto max = std::numeric_limits<uint64_t>::max();
  ImuClockMapper overflow(ImuClockDomain::FirstArrival);
  assert(overflow.map(1, max - 100) == max - 100);
  assert(!overflow.map(102, max));
  assert(overflow.map(101, max) == max);
  ImuClockMapper wide(ImuClockDomain::Gazebo);
  assert(wide.map(max, max - 100) == max);
  assert(!wide.map(max, 1));
}
