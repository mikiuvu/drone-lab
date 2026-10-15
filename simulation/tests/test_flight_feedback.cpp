#include "companion/flight_feedback.hpp"
#include <cassert>
#include <chrono>

int main() {
  using namespace std::chrono_literals;
  companion::FlightFeedback feedback;
  const auto t = companion::FlightFeedback::Time{};
  auto state = feedback.state(t);
  assert(state.armed && !state.landed && !state.heartbeat_fresh && !state.landed_fresh);
  feedback.landed(t, true);
  assert(!feedback.state(t).landed);

  feedback.heartbeat(t, false, true, false);
  feedback.landed(t, true);
  state = feedback.state(t + 499ms);
  assert(!state.armed && state.landed && state.offboard && !state.auto_land);
  feedback.heartbeat(t + 500ms, false, true, false);
  state = feedback.state(t + 500ms);
  assert(!state.landed && !state.landed_fresh && state.heartbeat_fresh && !state.armed);
  feedback.landed(t + 500ms, false);
  assert(feedback.state(t + 501ms).landed_fresh && !feedback.state(t + 501ms).landed);

  feedback.landed(t + 600ms, true);
  feedback.heartbeat(t + 601ms, true, false, true);
  state = feedback.state(t + 601ms);
  assert(state.armed && state.auto_land && !state.landed && !state.landed_fresh);
  feedback.landed(t + 602ms, true);
  assert(feedback.state(t + 602ms).landed);
  // 丢失心跳后，新的着地包也不能单独确认已停机；重连必须重新取得着地反馈。
  feedback.landed(t + 3500ms, true);
  state = feedback.state(t + 3601ms);
  assert(!state.heartbeat_fresh && !state.landed && state.armed && !state.auto_land);
  feedback.landed(t + 3601ms, true);
  feedback.heartbeat(t + 3602ms, false, false, false);
  assert(!feedback.state(t + 3602ms).landed_fresh);
  feedback.landed(t + 3603ms, true);
  assert(feedback.state(t + 3603ms).landed && !feedback.state(t + 3603ms).armed);
  assert(!feedback.state(t).heartbeat_fresh);
  assert(!feedback.state(t).landed_fresh);

  companion::FlightFeedback failure;
  failure.heartbeat(t, false, false, false, true);
  assert(failure.state(t).failsafe_active && !failure.state(t).failsafe_latched);
  failure.heartbeat(t + 1ms, true, false, false, true);
  assert(failure.state(t + 1ms).failsafe_active && failure.state(t + 1ms).failsafe_latched);
  failure.heartbeat(t + 2ms, true, false, true);
  assert(!failure.state(t + 2ms).failsafe_active && failure.state(t + 2ms).failsafe_latched);
  failure.heartbeat(t + 3ms, false, false, false);
  failure.landed(t + 4ms, true);
  assert(failure.state(t + 4ms).landed && failure.state(t + 4ms).failsafe_latched);
  assert(failure.state(t + 4s).failsafe_latched && !failure.state(t + 4s).failsafe_active);

  companion::FlightFeedback gap;
  gap.heartbeat(t, true, true, false);
  assert(!gap.state(t + 2999ms).heartbeat_gap_latched);
  assert(gap.state(t + 3s).heartbeat_gap_latched);
  gap.heartbeat(t + 3100ms, false, false, true);
  assert(gap.state(t + 3100ms).heartbeat_gap_latched);
  assert(!gap.state(t + 3100ms).failsafe_latched);
  companion::FlightFeedback preflight;
  preflight.heartbeat(t, false, false, false);
  preflight.heartbeat(t + 4s, false, false, false);
  assert(!preflight.state(t + 4s).heartbeat_gap_latched);
}
