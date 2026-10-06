#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <iostream>
#include <mutex>
#include <thread>
#include <unistd.h>

#include <gz/sim/components/Pose.hh>
#include <gz/msgs/empty.pb.h>
#include <sdf/sdf.hh>
#include <sdf/parser.hh>

#include "../gazebo/src/tof_ray_visualizer.cc"

namespace
{
gz::msgs::Image DepthImage(bool _millimeters = false)
{
  gz::msgs::Image image;
  image.set_width(8);
  image.set_height(8);
  image.set_pixel_format_type(_millimeters ? gz::msgs::L_INT16 : gz::msgs::R_FLOAT32);
  const size_t pixelBytes = _millimeters ? sizeof(uint16_t) : sizeof(float);
  const size_t stride = 8 * pixelBytes + 8;
  image.set_step(stride);
  std::string bytes(stride * 8, static_cast<char>(0xee));
  for (int cell = 0; cell < 64; ++cell)
  {
    const size_t offset = (cell / 8) * stride + (cell % 8) * pixelBytes;
    if (_millimeters)
    {
      const uint16_t value = 2000 + cell * 10;
      std::memcpy(bytes.data() + offset, &value, sizeof(value));
    }
    else
    {
      const float value = 1.0F + cell * 0.01F;
      std::memcpy(bytes.data() + offset, &value, sizeof(value));
    }
  }
  image.set_data(bytes);
  return image;
}

void Near(double _actual, double _expected)
{
  assert(std::abs(_actual - _expected) < 1e-6);
}
}  // namespace

int main()
{
  // 独立 transport 分区，使用真实服务与深度话题验证插件，不启动仿真。
  const std::string partition = "tof_visualizer_test_" + std::to_string(::getpid());
  assert(::setenv("GZ_PARTITION", partition.c_str(), 1) == 0);
  std::mutex mutex;
  gz::msgs::Marker received;
  uint64_t sequence = 0;
  gz::transport::Node receiver;
  std::function<bool(const gz::msgs::Marker &, gz::msgs::Empty &)> callback =
      [&](const gz::msgs::Marker &_marker, gz::msgs::Empty &) {
        std::lock_guard<std::mutex> lock(mutex);
        received = _marker;
        ++sequence;
        return true;
      };
  assert(receiver.Advertise("/marker", callback));

  auto sdf = std::make_shared<sdf::SDF>();
  assert(sdf::init(sdf));
  assert(sdf::readString(R"(
    <sdf version="1.9"><world name="test"><plugin name="rays" filename="rays">
      <vehicle_count>1</vehicle_count><publish_hz>10</publish_hz>
      <hfov_rad>0.7853981633974483</hfov_rad>
      <vfov_rad>0.7853981633974483</vfov_rad>
      <sensor_0_origin>0.055 0 0</sensor_0_origin>
      <sensor_0_rpy>0 0.3490658503988659 0</sensor_0_rpy>
      <sensor_1_origin>-0.0275 0.04763139720814413 0</sensor_1_origin>
      <sensor_1_rpy>0 0.3490658503988659 2.0943951023931953</sensor_1_rpy>
      <sensor_2_origin>-0.0275 -0.04763139720814413 0</sensor_2_origin>
      <sensor_2_rpy>0 0.3490658503988659 -2.0943951023931953</sensor_2_rpy>
    </plugin></world></sdf>)", sdf));
  gz::sim::EntityComponentManager ecm;
  gz::sim::EventManager events;
  const auto model = ecm.CreateEntity();
  ecm.CreateComponent(model, gz::sim::components::Model());
  ecm.CreateComponent(model, gz::sim::components::Name("micro_0"));
  ecm.CreateComponent(model, gz::sim::components::Pose(
      gz::math::Pose3d(2, 3, 4, 0, 0, M_PI / 2)));
  drone_sim::TofRayVisualizer plugin;
  plugin.Configure(model, sdf->Root()->GetElement("world")->GetElement("plugin"),
                   ecm, events);
  std::array<gz::transport::Node::Publisher, 3> depthPublishers;
  for (int sensor = 0; sensor < 3; ++sensor)
    depthPublishers[sensor] = receiver.Advertise<gz::msgs::Image>(
        "/simulation/vehicle_0/tof_" + std::to_string(sensor) + "/depth_image");
  gz::sim::UpdateInfo info;
  info.paused = false;
  auto waitFor = [&](const std::function<bool(const gz::msgs::Marker &)> &_ready,
                     const gz::msgs::Image *_image = nullptr) {
    uint64_t previous;
    {
      std::lock_guard<std::mutex> lock(mutex);
      previous = sequence;
    }
    for (int attempt = 0; attempt < 500; ++attempt)
    {
      if (_image)
        for (auto &publisher : depthPublishers)
          publisher.Publish(*_image);
      info.simTime += std::chrono::milliseconds(110);
      plugin.PostUpdate(info, ecm);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      std::lock_guard<std::mutex> lock(mutex);
      if (sequence > previous && _ready(received))
        return received;
    }
    assert(false && "ToF marker service did not receive expected geometry");
    return gz::msgs::Marker();
  };

  const auto cleared = waitFor([](const auto &_m) {
    return _m.action() == gz::msgs::Marker::DELETE_MARKER;
  });
  assert(cleared.ns() == "drone_tof_rays" && cleared.id() == 1);
  auto image = DepthImage();
  auto marker = waitFor([](const auto &_m) { return _m.point_size() == 384; }, &image);
  assert(marker.action() == gz::msgs::Marker::ADD_MODIFY);
  assert(marker.type() == gz::msgs::Marker::LINE_LIST);
  assert(marker.visibility() == gz::msgs::Marker::GUI);

  // 独立标量旋转校验三个安装角、像素轴向深度及模型世界位姿。
  const double pitch = 20 * M_PI / 180;
  for (int sensor = 0; sensor < 3; ++sensor)
  {
    const double yaw = (sensor == 0 ? 0 : sensor == 1 ? 120 : -120) * M_PI / 180;
    const double worldYaw = yaw + M_PI / 2;
    const double ox = 2 + 0.055 * std::cos(worldYaw);
    const double oy = 3 + 0.055 * std::sin(worldYaw);
    for (int cell = 0; cell < 64; ++cell)
    {
      const auto &origin = marker.point(sensor * 128 + cell * 2);
      const auto &end = marker.point(sensor * 128 + cell * 2 + 1);
      const double depth = 1.0F + cell * 0.01F;
      const double lateral = -(cell % 8 - 3.5) / 4 * std::tan(M_PI / 8) * depth;
      const double vertical = -(cell / 8 - 3.5) / 4 * std::tan(M_PI / 8) * depth;
      const double forward = std::cos(pitch) * depth + std::sin(pitch) * vertical;
      Near(origin.x(), ox); Near(origin.y(), oy); Near(origin.z(), 4);
      Near(end.x(), ox + std::cos(worldYaw) * forward - std::sin(worldYaw) * lateral);
      Near(end.y(), oy + std::sin(worldYaw) * forward + std::cos(worldYaw) * lateral);
      Near(end.z(), 4 - std::sin(pitch) * depth + std::cos(pitch) * vertical);
    }
  }

  image = DepthImage(true);
  marker = waitFor([&](const auto &_m) {
    const double expectedZ = 4 - 2 * std::sin(pitch) +
        std::cos(pitch) * 2 * 3.5 / 4 * std::tan(M_PI / 8);
    return _m.point_size() == 384 && std::abs(_m.point(1).z() - expectedZ) < 1e-6;
  }, &image);
  Near(marker.point(127).z(), 4 - 2.63 * std::sin(pitch) -
      std::cos(pitch) * 2.63 * 3.5 / 4 * std::tan(M_PI / 8));

  // 截断及全部无效图像必须删除旧标记，不能沿用旧点集。
  image.set_data("");
  marker = waitFor([](const auto &_m) {
    return _m.action() == gz::msgs::Marker::DELETE_MARKER;
  }, &image);
  assert(marker.point_size() == 0);
  image = DepthImage();
  image.set_data(std::string(image.data().size(), '\0'));
  waitFor([](const auto &_m) { return _m.action() == gz::msgs::Marker::DELETE_MARKER; }, &image);
  std::cout << "ToF service, 192 rays, mounts, padded depth and clearing passed\n";
}
