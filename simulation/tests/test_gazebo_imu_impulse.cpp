#include <gz/sim/TestFixture.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Link.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/imu.pb.h>
#include <nlohmann/json.hpp>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <thread>

int main(int argc, char **argv) {
  if (argc != 3) return 2;
  using json = nlohmann::json;
  json physics = json::array(), slow = json::array(), fast = json::array();
  std::mutex mutex;
  gz::transport::Node node;
  auto record = [&](json &rows, const gz::msgs::IMU &msg) {
    const auto &stamp = msg.header().stamp();
    std::lock_guard<std::mutex> lock(mutex);
    const auto &a = msg.linear_acceleration();
    rows.push_back({{"time_us", stamp.sec()*1000000 + stamp.nsec()/1000},
                    {"specific_force", {a.x(), a.y(), a.z()}}});
  };
  std::function<void(const gz::msgs::IMU &)> slow_callback = [&](const auto &msg) { record(slow, msg); };
  std::function<void(const gz::msgs::IMU &)> fast_callback = [&](const auto &msg) { record(fast, msg); };
  if (!node.Subscribe("/impulse/imu250", slow_callback) ||
      !node.Subscribe("/impulse/imu1000", fast_callback)) return 3;
  gz::sim::Entity body = gz::sim::kNullEntity;
  gz::sim::TestFixture fixture(argv[1]);
  fixture.OnConfigure([&](const auto &, const auto &, auto &ecm, auto &) {
    body = ecm.EntityByComponents(gz::sim::components::Link(), gz::sim::components::Name("body"));
    if (body == gz::sim::kNullEntity) throw std::runtime_error("missing falling body");
    gz::sim::Link link(body);
    link.EnableVelocityChecks(ecm);
    link.EnableAccelerationChecks(ecm);
  });
  fixture.OnPostUpdate([&](const auto &info, const auto &ecm) {
    const gz::sim::Link link(body);
    const auto v = link.WorldLinearVelocity(ecm);
    const auto a = link.WorldLinearAcceleration(ecm);
    if (!v || !a) return;
    physics.push_back({{"time_us", std::chrono::duration_cast<std::chrono::microseconds>(info.simTime).count()},
                       {"velocity", {v->X(), v->Y(), v->Z()}},
                       {"acceleration", {a->X(), a->Y(), a->Z()}}});
  }).Finalize();
  // Allow subscription discovery before advancing the deterministic physics steps.
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  if (!fixture.Server()->Run(true, 700, false)) return 4;
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  std::lock_guard<std::mutex> lock(mutex);
  const json result = {{"physics", physics}, {"imu250", slow}, {"imu1000", fast}};
  std::ofstream output(argv[2]);
  output << result.dump(2) << '\n';
  if (!output || physics.size() != 700 || fast.size() < 600 || slow.size() < 150) return 5;
  auto velocity_at = [&](int64_t stamp) {
    for (const auto &row : physics) {
      if (row.at("time_us").get<int64_t>() == stamp)
        return row.at("velocity").at(2).get<double>();
    }
    throw std::runtime_error("missing physics velocity endpoint");
  };
  auto integrate = [&](const json &rows, int64_t begin, int64_t end, int64_t period) {
    double delta = 0;
    int64_t coverage = 0;
    for (size_t i = 1; i < rows.size(); ++i) {
      const auto previous = rows[i - 1].at("time_us").get<int64_t>();
      const auto current = rows[i].at("time_us").get<int64_t>();
      if (current <= begin || previous >= end) continue;
      if (current - previous != period)
        throw std::runtime_error("missing or unordered IMU sample");
      const auto duration = std::min(end, current) - std::max(begin, previous);
      delta += (rows[i].at("specific_force").at(2).get<double>() - 9.80665) * duration * 1e-6;
      coverage += duration;
    }
    if (coverage != end - begin) throw std::runtime_error("incomplete IMU window");
    return delta;
  };
  const double actual = velocity_at(530000) - velocity_at(380000);
  const double fast_error = std::abs(integrate(fast, 380000, 530000, 1000) - actual);
  const double slow_error = std::abs(integrate(slow, 380000, 530000, 4000) - actual);
  const double freefall = velocity_at(250000) - velocity_at(100000);
  const double fast_freefall_error = std::abs(integrate(fast, 100000, 250000, 1000) - freefall);
  const double slow_freefall_error = std::abs(integrate(slow, 100000, 250000, 4000) - freefall);
  std::cout << json({{"fast_error_mps", fast_error}, {"slow_error_mps", slow_error},
                    {"fast_freefall_error_mps", fast_freefall_error},
                    {"slow_freefall_error_mps", slow_freefall_error}}).dump() << '\n';
  if (fast_error > 1e-6 || fast_freefall_error > 1e-6 || slow_freefall_error > 1e-6) return 6;
}
