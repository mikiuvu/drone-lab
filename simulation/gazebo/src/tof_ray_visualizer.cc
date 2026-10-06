/**
 * @file tof_ray_visualizer.cc
 * @brief 把每架无人机三向 8×8 ToF 的测距射线画成 Gazebo 可视化标记。
 * @note 标记以 GUI 可见性发布，不会被机载相机或深度相机渲染，因此不会污染
 *       仿真传感器数据；本插件只读深度图话题，不参与物理或估计。
 */
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <iostream>
#include <mutex>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/marker.pb.h>
#include <gz/msgs/vector3d.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace drone_sim
{
/**
 * @brief 订阅三向深度图并向 GUI 标记服务发送 LINE_LIST，逐架无人机一种颜色。
 * @note 射线几何与 companion 的 Estimator::tofRayBody 完全一致：像素列/行给出
 *       光轴坐标下的横向/纵向偏移，深度是沿光轴的轴向距离而非欧氏距离。
 */
class TofRayVisualizer final : public gz::sim::System,
                               public gz::sim::ISystemConfigure,
                               public gz::sim::ISystemPostUpdate
{
 public:
  /**
   * @brief 读取机数、发布频率、视场与三个传感器的安装外参。
   * @param _entity 世界实体，本插件不修改任何实体或组件。
   * @param _sdf 插件参数；缺少视场或安装外参时插件停用并打印错误。
   */
  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &,
                 gz::sim::EventManager &) override
  {
    (void)_entity;
    if (_sdf->HasElement("vehicle_count"))
      vehicleCount_ = _sdf->Get<int>("vehicle_count");
    if (_sdf->HasElement("publish_hz"))
      publishHz_ = _sdf->Get<double>("publish_hz");
    if (_sdf->HasElement("model_prefix"))
      modelPrefix_ = _sdf->Get<std::string>("model_prefix");
    if (_sdf->HasElement("topic_template"))
      topicTemplate_ = _sdf->Get<std::string>("topic_template");
    if (_sdf->HasElement("hfov_rad"))
      hfovRad_ = _sdf->Get<double>("hfov_rad");
    if (_sdf->HasElement("vfov_rad"))
      vfovRad_ = _sdf->Get<double>("vfov_rad");
    if (_sdf->HasElement("min_range_m"))
      minRangeM_ = _sdf->Get<double>("min_range_m");
    if (_sdf->HasElement("max_range_m"))
      maxRangeM_ = _sdf->Get<double>("max_range_m");
    for (int sensor = 0; sensor < kSensorCount; ++sensor)
    {
      const std::string originKey = "sensor_" + std::to_string(sensor) + "_origin";
      const std::string rpyKey = "sensor_" + std::to_string(sensor) + "_rpy";
      if (_sdf->HasElement(originKey) && _sdf->HasElement(rpyKey))
      {
        const gz::math::Vector3d origin =
            _sdf->Get<gz::math::Vector3d>(originKey);
        const gz::math::Vector3d rpy = _sdf->Get<gz::math::Vector3d>(rpyKey);
        mounts_[sensor] = gz::math::Pose3d(
            origin, gz::math::Quaterniond(rpy.X(), rpy.Y(), rpy.Z()));
        mountValid_[sensor] = true;
      }
    }
    const bool mountsComplete =
        std::all_of(mountValid_.begin(), mountValid_.end(), [](bool ok) { return ok; });
    if (vehicleCount_ < 1 || vehicleCount_ > kMaxVehicles || !mountsComplete ||
        !std::isfinite(publishHz_) || publishHz_ <= 0 ||
        !std::isfinite(hfovRad_) || hfovRad_ <= 0 || hfovRad_ >= M_PI / 2 ||
        !std::isfinite(vfovRad_) || vfovRad_ <= 0 || vfovRad_ >= M_PI / 2 ||
        !std::isfinite(minRangeM_) || !std::isfinite(maxRangeM_) ||
        minRangeM_ <= 0 || maxRangeM_ <= minRangeM_)
    {
      gzerr << "TofRayVisualizer: invalid configuration; rays disabled\n";
      valid_ = false;
      return;
    }
    period_ = 1.0 / publishHz_;
    depths_.assign(static_cast<size_t>(vehicleCount_),
                   std::array<std::array<double, kCellCount>, kSensorCount>{});
    fresh_.assign(static_cast<size_t>(vehicleCount_),
                  std::array<bool, kSensorCount>{});
    for (int vehicle = 0; vehicle < vehicleCount_; ++vehicle)
      for (int sensor = 0; sensor < kSensorCount; ++sensor)
        node_.Subscribe(
            TopicFor(vehicle, sensor),
            std::function<void(const gz::msgs::Image &,
                               const gz::transport::MessageInfo &)>(
                [this, vehicle, sensor](const gz::msgs::Image &_msg,
                                        const gz::transport::MessageInfo &) {
                  OnDepth(_msg, vehicle, sensor);
                }));
    valid_ = true;
    gzmsg << "TofRayVisualizer: ToF rays for " << vehicleCount_
          << " vehicles via " << kMarkerService << " service\n";
  }

  /**
   * @brief 按发布周期把缓存深度转成世界系射线并发布标记。
   * @param _info 提供仿真时间与暂停标志；暂停时不更新，标记保持上一帧。
   * @param _ecm 用于查询各机模型的世界位姿。
   */
  void PostUpdate(const gz::sim::UpdateInfo &_info,
                  const gz::sim::EntityComponentManager &_ecm) override
  {
    if (!valid_ || _info.paused)
      return;
    const double now = std::chrono::duration<double>(_info.simTime).count();
    if (lastPublish_ >= 0.0 && now - lastPublish_ < period_)
      return;
    lastPublish_ = now;

    std::lock_guard<std::mutex> guard(mutex_);
    for (int vehicle = 0; vehicle < vehicleCount_; ++vehicle)
    {
      if (modelEntities_[vehicle] == gz::sim::kNullEntity)
      {
        modelEntities_[vehicle] = _ecm.EntityByComponents(
            gz::sim::components::Model(),
            gz::sim::components::Name(modelPrefix_ + std::to_string(vehicle)));
        if (modelEntities_[vehicle] == gz::sim::kNullEntity)
          continue;
      }
      const gz::math::Pose3d modelPose =
          gz::sim::worldPose(modelEntities_[vehicle], _ecm);
      gz::msgs::Marker marker;
      marker.set_ns(kMarkerNamespace);
      // id 从 1 起编：proto3 不序列化等于默认值的字段，id=0 在报文里与「未设置」无法区分。
      marker.set_id(static_cast<uint64_t>(vehicle) + 1U);
      marker.set_action(gz::msgs::Marker::ADD_MODIFY);
      marker.set_type(gz::msgs::Marker::LINE_LIST);
      // 只让 GUI 相机渲染：机载相机与深度相机必须看不到这些辅助线。
      marker.set_visibility(gz::msgs::Marker::GUI);
      SetColor(marker.mutable_material(), vehicle);
      for (int sensor = 0; sensor < kSensorCount; ++sensor)
      {
        if (!fresh_[vehicle][sensor])
          continue;
        const gz::math::Pose3d sensorPose = modelPose * mounts_[sensor];
        const gz::math::Vector3d origin = sensorPose.Pos();
        for (int cell = 0; cell < kCellCount; ++cell)
        {
          const double depth = depths_[vehicle][sensor][cell];
          if (!std::isfinite(depth) || depth <= minRangeM_ || depth >= maxRangeM_)
            continue;
          const int row = cell / kSide;
          const int col = cell % kSide;
          const double u = (col - kCenter) / kHalf * std::tan(hfovRad_ / 2.0);
          const double v = (row - kCenter) / kHalf * std::tan(vfovRad_ / 2.0);
          // 轴向深度：光轴方向分量恒为 depth，横向/纵向按像素角偏移缩放。
          const gz::math::Vector3d offset = sensorPose.Rot() *
              gz::math::Vector3d(depth, -u * depth, -v * depth);
          const gz::math::Vector3d end = origin + offset;
          auto *start = marker.add_point();
          start->set_x(origin.X());
          start->set_y(origin.Y());
          start->set_z(origin.Z());
          auto *stop = marker.add_point();
          stop->set_x(end.X());
          stop->set_y(end.Y());
          stop->set_z(end.Z());
        }
      }
      // 空点集的 ADD_MODIFY 不清除旧点；显式删除本机上一帧标记。
      if (marker.point_size() == 0)
        marker.set_action(gz::msgs::Marker::DELETE_MARKER);
      // MarkerManager 的 /marker 服务接收标记，同名话题仅输出更新。
      // 单向请求不等待 GUI 响应，GUI 稍后连接时下一周期会重发当前测距。
      node_.Request(kMarkerService, marker);
    }
  }

 private:
  static constexpr int kSensorCount = 3;
  static constexpr int kSide = 8;
  static constexpr int kCellCount = kSide * kSide;
  static constexpr int kMaxVehicles = 16;
  static constexpr double kCenter = 3.5; // 8×8 的像素中心偏移
  static constexpr double kHalf = 4.0;
  static constexpr const char *kMarkerService = "/marker";
  static constexpr const char *kMarkerNamespace = "drone_tof_rays";

  /** @return 指定机与传感器的深度图话题名。 */
  std::string TopicFor(int _vehicle, int _sensor) const
  {
    std::string topic = topicTemplate_;
    for (const auto &pair : {std::make_pair(std::string("{i}"), std::to_string(_vehicle)),
                             std::make_pair(std::string("{s}"), std::to_string(_sensor))})
    {
      for (size_t at = topic.find(pair.first); at != std::string::npos;
           at = topic.find(pair.first, at))
        topic.replace(at, pair.first.size(), pair.second);
    }
    return topic;
  }

  /** @brief 把 8×8 深度图缓存到本机本传感器槽位，不修改消息内容。 */
  void OnDepth(const gz::msgs::Image &_msg, int _vehicle, int _sensor)
  {
    if (_vehicle < 0 || _vehicle >= vehicleCount_ ||
        _sensor < 0 || _sensor >= kSensorCount)
      return;
    if (_msg.width() != kSide || _msg.height() != kSide)
      return;
    std::array<double, kCellCount> depth{};
    bool any = false;
    for (int cell = 0; cell < kCellCount; ++cell)
    {
      double value = 0.0;
      if (!ReadDepth(_msg, cell, value))
        continue;
      depth[cell] = value;
      any = true;
    }
    std::lock_guard<std::mutex> guard(mutex_);
    depths_[_vehicle][_sensor] = depth;
    fresh_[_vehicle][_sensor] = any;
  }

  /** @return 成功读取一个轴向深度像素（米）时为 true。 */
  bool ReadDepth(const gz::msgs::Image &_msg, int _cell, double &_depthM) const
  {
    if (_cell < 0 || _cell >= kCellCount)
      return false;
    size_t bytesPerPixel = 0;
    switch (_msg.pixel_format_type())
    {
      case gz::msgs::PixelFormatType::R_FLOAT32:
        bytesPerPixel = sizeof(float);
        break;
      case gz::msgs::PixelFormatType::L_INT16:
        bytesPerPixel = sizeof(uint16_t);
        break;
      default:
        return false;
    }
    const size_t rowBytes = static_cast<size_t>(_msg.width()) * bytesPerPixel;
    const size_t stride = _msg.step() != 0 ? _msg.step() : rowBytes;
    if (stride < rowBytes)
      return false;
    const size_t offset = static_cast<size_t>(_cell / kSide) * stride +
                          static_cast<size_t>(_cell % kSide) * bytesPerPixel;
    if (offset + bytesPerPixel > _msg.data().size())
      return false;
    if (bytesPerPixel == sizeof(float))
    {
      float value = 0.0F;
      std::memcpy(&value, _msg.data().data() + offset, sizeof(value));
      _depthM = static_cast<double>(value);
    }
    else
    {
      uint16_t value = 0;
      std::memcpy(&value, _msg.data().data() + offset, sizeof(value));
      _depthM = static_cast<double>(value) * 0.001;
    }
    return std::isfinite(_depthM);
  }

  /** @brief 按机号写入与桌面界面一致的分类型配色。 */
  static void SetColor(gz::msgs::Material *_material, int _vehicle)
  {
    // 与 app/theme.py 的 CATEGORICAL_DARK 一致，便于在 Gazebo 里对上号。
    static const std::array<std::array<float, 3>, 8> kPalette{{
        {{0.353F, 0.635F, 0.941F}},
        {{0.941F, 0.478F, 0.306F}},
        {{0.298F, 0.765F, 0.541F}},
        {{0.694F, 0.514F, 0.910F}},
        {{0.878F, 0.690F, 0.235F}},
        {{0.247F, 0.722F, 0.769F}},
        {{0.898F, 0.482F, 0.690F}},
        {{0.576F, 0.651F, 0.769F}},
    }};
    const auto &rgb = kPalette[static_cast<size_t>(_vehicle) % kPalette.size()];
    for (auto *color : {_material->mutable_ambient(), _material->mutable_diffuse()})
    {
      color->set_r(rgb[0]);
      color->set_g(rgb[1]);
      color->set_b(rgb[2]);
      color->set_a(1.0F);
    }
  }

  int vehicleCount_{1};
  double publishHz_{10.0};
  double period_{0.1};
  double lastPublish_{-1.0};
  double hfovRad_{0.0};
  double vfovRad_{0.0};
  double minRangeM_{0.02};
  double maxRangeM_{4.0};
  bool valid_{false};
  std::string modelPrefix_{"micro_"};
  std::string topicTemplate_{"/simulation/vehicle_{i}/tof_{s}/depth_image"};
  std::array<gz::math::Pose3d, kSensorCount> mounts_{};
  std::array<bool, kSensorCount> mountValid_{};
  std::array<gz::sim::Entity, kMaxVehicles> modelEntities_{};
  std::vector<std::array<std::array<double, kCellCount>, kSensorCount>> depths_;
  std::vector<std::array<bool, kSensorCount>> fresh_;
  mutable std::mutex mutex_;
  gz::transport::Node node_;
};
}  // namespace drone_sim

GZ_ADD_PLUGIN(drone_sim::TofRayVisualizer, gz::sim::System,
              drone_sim::TofRayVisualizer::ISystemConfigure,
              drone_sim::TofRayVisualizer::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(drone_sim::TofRayVisualizer, "drone_sim::TofRayVisualizer")
