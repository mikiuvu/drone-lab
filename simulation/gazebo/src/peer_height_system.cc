/**
 * @file peer_height_system.cc
 * @brief 根据 Gazebo 物理状态生成高度、光流速度、基站测距和机间 UWB 观测。
 */
#include "drone_sim/peer_height_system.hh"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <filesystem>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>

#include "companion/terrain.hpp"
#include "drone_sim/uwb_occlusion.hh"

namespace
{
constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kTwoPi = 2.0 * kPi;

/**
 * @brief 将有限方位角归一到 [-π,π]，便于视场检查和角噪声叠加。
 * @param _angle 角度，单位 rad。
 * @return 与输入等价的主值角，单位 rad。
 */
double WrapBearing(double _angle)
{
  while (_angle > kPi)
    _angle -= kTwoPi;
  while (_angle < -kPi)
    _angle += kTwoPi;
  return _angle;
}

// 可选 SDF 元素缺失时保留默认值；存在时由 SDF 完成类型转换。
template<typename T>
T GetOr(const std::shared_ptr<const sdf::Element> &_sdf,
        const std::string &_name, const T &_fallback)
{
  if (!_sdf || !_sdf->HasElement(_name))
    return _fallback;
  return _sdf->Get<T>(_name);
}

}  // namespace

namespace drone_sim
{

PeerHeightSystem::PeerHeightSystem()
  : random_(seed_)
{
}

PeerHeightSystem::~PeerHeightSystem()
{
  if (socketFd_ >= 0)
    ::close(socketFd_);
}

/**
 * @brief 读取世界插件参数、加载遮挡地形并创建观测发送套接字。
 * @param _sdf 插件配置；缺失字段使用默认值。
 * @note 位置/距离为 m，速度为 m/s，角度为 rad，发布周期为 us；
 *       光流和基站非法参数可抛异常，其他已检查的配置失败记录错误后返回。
 */
void PeerHeightSystem::Configure(
    const gz::sim::Entity &,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &,
    gz::sim::EventManager &)
{
  destinationHost_ = GetOr<std::string>(_sdf, "destination_host", destinationHost_);
  portBase_ = GetOr<int>(_sdf, "port_base", portBase_);
  const auto publishHz = GetOr<double>(_sdf, "publish_hz", 50.0);
  const int vehicleCount=GetOr<int>(_sdf,"vehicle_count",4);
  if (!std::isfinite(publishHz) || publishHz <= 0.0 || publishHz > 1000.0 ||
      vehicleCount<1||portBase_ < 1 || portBase_+vehicleCount > 65536)
  {
    std::cerr << "peer_height_system: invalid publish_hz or port_base\n";
    return;
  }
  vehicles_.resize(vehicleCount);
  flowEnabled_=GetOr<bool>(_sdf,"flow_enabled",true);
  flowNoiseStdMps_=GetOr<double>(_sdf,"flow_noise_std_mps",.015);
  flowScaleError_=GetOr<double>(_sdf,"flow_scale_error",.005);
  flowDropProbability_=GetOr<double>(_sdf,"flow_drop_probability",.01);
  if(!std::isfinite(flowNoiseStdMps_)||flowNoiseStdMps_<0||!std::isfinite(flowScaleError_)||!std::isfinite(flowDropProbability_)||flowDropProbability_<0||flowDropProbability_>1)throw std::invalid_argument("invalid optical flow parameters");
  const int anchorCount=GetOr<int>(_sdf,"anchor_count",0);
  if(anchorCount<0)throw std::invalid_argument("invalid anchor count");
  for(int i=0;i<anchorCount;++i){
    const auto prefix="anchor_"+std::to_string(i);
    Anchor anchor{GetOr<int>(_sdf,prefix+"_id",i),GetOr<gz::math::Vector3d>(_sdf,prefix+"_position",gz::math::Vector3d::Zero)};
    if(anchor.id<0||!anchor.position.IsFinite())throw std::invalid_argument("invalid anchor");
    anchors_.push_back(anchor);
  }
  publishPeriodUs_ = publishHz > 0.0
      ? static_cast<int64_t>(std::llround(1.0e6 / publishHz)) : 20000;
  rangeNoiseStdM_ = std::max(0.0, GetOr<double>(_sdf, "range_noise_std_m", rangeNoiseStdM_));
  bearingNoiseStdRad_ = std::max(0.0, GetOr<double>(
      _sdf, "bearing_noise_std_rad", bearingNoiseStdRad_));
  rangeMinM_ = std::max(0.0, GetOr<double>(_sdf, "range_min_m", rangeMinM_));
  rangeMaxM_ = std::max(rangeMinM_, GetOr<double>(_sdf, "range_max_m", rangeMaxM_));
  bearingFovRad_ = std::clamp(GetOr<double>(_sdf, "bearing_fov_rad", bearingFovRad_),
                              0.0, kTwoPi);
  dropProbability_ = std::clamp(
      GetOr<double>(_sdf, "drop_probability", dropProbability_), 0.0, 1.0);
  bearingPiAmbiguityProbability_ = std::clamp(GetOr<double>(
      _sdf, "bearing_pi_ambiguity_probability", bearingPiAmbiguityProbability_), 0.0, 1.0);
  const auto terrainTrianglesFile = GetOr<std::string>(
      _sdf, "terrain_triangles_file", "");
  nlosDropProbability_ = GetOr<double>(
      _sdf, "nlos_drop_probability", nlosDropProbability_);
  nlosRangeBiasM_ = GetOr<double>(
      _sdf, "nlos_range_bias_m", nlosRangeBiasM_);
  nlosRangeNoiseStdM_ = GetOr<double>(
      _sdf, "nlos_range_noise_std_m", nlosRangeNoiseStdM_);
  if (!std::isfinite(nlosDropProbability_) || nlosDropProbability_ < 0.0 ||
      nlosDropProbability_ > 1.0 || !std::isfinite(nlosRangeBiasM_) ||
      nlosRangeBiasM_ < 0.0 || !std::isfinite(nlosRangeNoiseStdM_) ||
      nlosRangeNoiseStdM_ < 0.0)
  {
    std::cerr << "peer_height_system: invalid NLOS stress parameters\n";
    return;
  }
  terrainMap_.reset();
  terrainLoadFailed_ = false;
  if (!terrainTrianglesFile.empty())
  {
    try
    {
      terrainMap_ = companion::TerrainMap::load(
          std::filesystem::path(terrainTrianglesFile));
      std::cerr << "peer_height_system: loaded " << terrainMap_->triangleCount()
                << " terrain triangles from " << terrainTrianglesFile << '\n';
    }
    catch (const std::exception &error)
    {
      terrainMap_.reset();
      terrainLoadFailed_ = true;
      std::cerr << "peer_height_system: configured terrain load failed: "
                << error.what() << '\n';
    }
  }
  if (terrainMap_)
    std::cerr << "peer_height_system: NLOS is an uncalibrated stress model, "
              << "not radio physics (additional drop=" << nlosDropProbability_
              << ", positive bias=" << nlosRangeBiasM_
              << " m, half-normal noise sigma=" << nlosRangeNoiseStdM_ << " m)\n";
  seed_ = GetOr<uint64_t>(_sdf, "seed", seed_);
  random_.seed(seed_);

  for (size_t i = 0; i < vehicles_.size(); ++i)
  {
    const auto index = std::to_string(i);
    vehicles_[i].modelName = GetOr<std::string>(
        _sdf, "vehicle_" + index + "_model", "micro_" + index);
  }

  socketFd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (socketFd_ < 0)
  {
    std::perror("peer_height_system: socket");
    return;
  }

  sockaddr_in destination{};
  destination.sin_family = AF_INET;
  if (::inet_pton(AF_INET, destinationHost_.c_str(), &destination.sin_addr) != 1)
  {
    std::cerr << "peer_height_system: destination_host must be an IPv4 address: "
              << destinationHost_ << '\n';
    ::close(socketFd_);
    socketFd_ = -1;
    return;
  }

  std::cerr << "peer_height_system: UDP observations to " << destinationHost_
            << ':' << portBase_ << ".." << portBase_ + vehicles_.size()-1
            << ", height assist at " << publishHz << " Hz\n";
}

/**
 * @brief 按模型名重新查找实体，未找到的车辆保持空实体编号。
 * @param _ecm 当前仿真实体组件视图。
 * @note 每次发布重新查询，以适应模型延后加入或移除。
 */
void PeerHeightSystem::RefreshEntities(
    const gz::sim::EntityComponentManager &_ecm)
{
  for (auto &vehicle : vehicles_)
    vehicle.modelEntity = gz::sim::kNullEntity;
  _ecm.Each<gz::sim::components::Model, gz::sim::components::Name>(
      [this](const gz::sim::Entity &_entity,
             const gz::sim::components::Model *,
             const gz::sim::components::Name *_name)
      {
        if (!_name)
          return true;
        for (auto &vehicle : vehicles_)
        {
          if (_name->Data() == vehicle.modelName)
            vehicle.modelEntity = _entity;
        }
        return true;
      });
}

/**
 * @brief 把物理更新的仿真时间转换为微秒。
 * @param _info Gazebo 更新信息。
 * @return 仿真时间，单位 us，不使用本机墙钟。
 */
int64_t PeerHeightSystem::SimTimeUs(const gz::sim::UpdateInfo &_info)
{
  return std::chrono::duration_cast<std::chrono::microseconds>(
      _info.simTime).count();
}

/**
 * @brief 向 portBase + vehicle 发送一条完整 JSON 观测。
 * @param _vehicle 配置中有效的车辆编号。
 * @param _json 单个数据报的内容。
 * @return 套接字有效且全部字节被 sendto 接受时为 true。
 */
bool PeerHeightSystem::Send(int _vehicle, const std::string &_json) const
{
  if (socketFd_ < 0 || _vehicle < 0 || static_cast<size_t>(_vehicle) >= vehicles_.size())
    return false;

  sockaddr_in destination{};
  destination.sin_family = AF_INET;
  destination.sin_port = htons(static_cast<uint16_t>(portBase_ + _vehicle));
  if (::inet_pton(AF_INET, destinationHost_.c_str(), &destination.sin_addr) != 1)
    return false;

  const auto result = ::sendto(socketFd_, _json.data(), _json.size(), 0,
      reinterpret_cast<const sockaddr *>(&destination), sizeof(destination));
  return result == static_cast<ssize_t>(_json.size());
}

/**
 * @brief 将当前物理位姿和相邻时刻差分转换为带误差与质量标志的观测。
 * @param _info 当前更新信息；暂停时不发送。
 * @param _ecm 用于读取模型世界位姿的组件视图。
 * @note 世界坐标为 ENU，机体速度和方位角使用 FLU；无有效位姿的车辆跳过。
 */
void PeerHeightSystem::Publish(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
  RefreshEntities(_ecm);
  const auto simUs = SimTimeUs(_info);
  if (_info.paused)
    return;
  if (lastPublishSimUs_ >= 0 && simUs < lastPublishSimUs_)
    random_.seed(seed_);
  std::vector<gz::math::Pose3d> poses(vehicles_.size());
  std::vector<bool> poseValid(vehicles_.size(),false);

  for (size_t i = 0; i < vehicles_.size(); ++i)
  {
    if (vehicles_[i].modelEntity == gz::sim::kNullEntity)
      continue;
    poses[i] = gz::sim::worldPose(vehicles_[i].modelEntity, _ecm);
    poseValid[i] = std::isfinite(poses[i].Pos().X()) &&
                   std::isfinite(poses[i].Pos().Y()) &&
                   std::isfinite(poses[i].Pos().Z());
  }

  std::bernoulli_distribution dropout(dropProbability_);
  std::bernoulli_distribution ambiguity(bearingPiAmbiguityProbability_);
  std::normal_distribution<double> rangeNoise(0.0, rangeNoiseStdM_);
  std::normal_distribution<double> bearingNoise(0.0, bearingNoiseStdRad_);
  std::normal_distribution<double> nlosRangeNoise(0.0, 1.0);
  std::bernoulli_distribution nlosDrop(nlosDropProbability_);
  std::normal_distribution<double> flowNoise(0,flowNoiseStdMps_);
  std::bernoulli_distribution flowDrop(flowDropProbability_);

  for (size_t i = 0; i < vehicles_.size(); ++i)
  {
    if (!poseValid[i])
      continue;

    // 1. 高度观测直接取世界 ENU 的 z（m），以 SIM_HEIGHT_ASSIST 标记其来源。
    {
      std::ostringstream height;
      height << std::setprecision(9)
             << "{\"schema\":1,\"kind\":\"height_assist\","
             << "\"vehicle\":" << i << ",\"sim_time_us\":" << simUs
             << ",\"z_world_m\":" << poses[i].Pos().Z()
             << ",\"source\":\"SIM_HEIGHT_ASSIST\"}\n";
      Send(i, height.str());
    }

    // 2. 世界位移除以仿真间隔得到速度，再用当前姿态逆旋转到机体 FLU。
    // 间隔须小于 0.25 s；倾角超过 0.65 rad 或随机丢帧时质量置零，速度单位 m/s。
    auto&vehicle=vehicles_[i];
    if(flowEnabled_&&vehicle.previousTimeUs>=0&&simUs>vehicle.previousTimeUs){
      const double dt=(simUs-vehicle.previousTimeUs)*1e-6;
      if(dt<.25){
        const auto velocity=poses[i].Rot().Inverse().RotateVector((poses[i].Pos()-vehicle.previousPosition)/dt);
        const bool valid=!flowDrop(random_)&&std::abs(poses[i].Rot().Roll())<.65&&std::abs(poses[i].Rot().Pitch())<.65;
        std::ostringstream flow;
        flow<<std::setprecision(9)<<"{\"schema\":1,\"kind\":\"optical_flow\",\"vehicle\":"<<i<<",\"sim_time_us\":"<<simUs
            <<",\"vx_body_mps\":"<<velocity.X()*(1+flowScaleError_)+flowNoise(random_)
            <<",\"vy_body_mps\":"<<velocity.Y()*(1+flowScaleError_)+flowNoise(random_)
            <<",\"quality\":"<<(valid?.95:0)<<",\"variance_m2s2\":"<<std::max(.0004,flowNoiseStdMps_*flowNoiseStdMps_)
            <<",\"source\":\"SIM_FLOW_MODULE\"}";
        Send(i,flow.str());
      }
    }
    vehicle.previousPosition=poses[i].Pos();vehicle.previousTimeUs=simUs;
    // 3. 基站提供三维距离；地形遮挡附加正偏差并降低质量，地形加载失败则 valid=false。
    // 基站分支只使用普通丢帧和距离噪声，额外 NLOS 丢帧与半正态噪声用于后面的机间分支。
    for(const auto&anchor:anchors_){
      const auto delta=anchor.position-poses[i].Pos();
      const Eigen::Vector3d from(poses[i].Pos().X(),poses[i].Pos().Y(),poses[i].Pos().Z());
      const Eigen::Vector3d to(anchor.position.X(),anchor.position.Y(),anchor.position.Z());
      const bool nlos=TerrainOccludesPeer(terrainMap_.get(),from,to);
      const bool valid=!terrainLoadFailed_&&delta.Length()<=rangeMaxM_&&!dropout(random_);
      const double range=std::max(.01,delta.Length()+rangeNoise(random_)+(nlos?nlosRangeBiasM_:0));
      std::ostringstream message;message<<std::setprecision(9)<<"{\"schema\":1,\"kind\":\"uwb_anchor\",\"vehicle\":"<<i<<",\"anchor_id\":"<<anchor.id
         <<",\"sim_time_us\":"<<simUs<<",\"range_m\":"<<range<<",\"variance_m2\":"<<(nlos?.25:.0025)<<",\"quality\":"<<(nlos?.25:.95)
         <<",\"valid\":"<<(valid?"true":"false")<<"}";
      Send(i,message.str());
    }
    // 4. 对每个有向机对，将相对位置转到观察者机体系：模长为三维距离，atan2(y,x) 为水平方位。
    // 先检查视场、量程、遮挡和丢帧，再叠加测量误差；无效帧仍发送 valid=false。
    for (size_t j = 0; j < vehicles_.size(); ++j)
    {
      if (i == j || !poseValid[j])
        continue;

      const auto deltaWorld = poses[j].Pos() - poses[i].Pos();
      const auto deltaBody = poses[i].Rot().Inverse().RotateVector(deltaWorld);
      const auto trueRange = deltaBody.Length();
      auto trueBearing = std::atan2(deltaBody.Y(), deltaBody.X());
      const auto inFov = bearingFovRad_ >= kTwoPi - 1e-9 ||
          std::abs(WrapBearing(trueBearing)) <= bearingFovRad_ * 0.5;
      const auto inRange = trueRange >= rangeMinM_ && trueRange <= rangeMaxM_;
      const Eigen::Vector3d observerPosition(
          poses[i].Pos().X(), poses[i].Pos().Y(), poses[i].Pos().Z());
      const Eigen::Vector3d peerPosition(
          poses[j].Pos().X(), poses[j].Pos().Y(), poses[j].Pos().Z());
      const bool nlos = TerrainOccludesPeer(terrainMap_.get(),
          observerPosition, peerPosition);
      const auto valid = !terrainLoadFailed_ && inFov && inRange && !dropout(random_) &&
          (!nlos || !nlosDrop(random_));

      double range = 0.0;
      double bearing = 0.0;
      if (valid)
      {
        range = trueRange + rangeNoise(random_);
        if (nlos)
        {
          // 遮挡距离增加固定正偏差和半正态随机量，用于模拟偏大的测距误差。
          range += nlosRangeBiasM_;
          if (nlosRangeNoiseStdM_ > 0.0)
            range += nlosRangeNoiseStdM_ * std::abs(nlosRangeNoise(random_));
        }
        range = std::clamp(range, rangeMinM_, rangeMaxM_);
        bearing = WrapBearing(trueBearing + bearingNoise(random_));
        if (ambiguity(random_))
          bearing = WrapBearing(bearing + kPi);
      }

      std::ostringstream uwb;
      uwb << std::setprecision(9)
          << "{\"schema\":1,\"kind\":\"uwb\",\"vehicle\":" << i
          << ",\"peer\":" << j << ",\"sim_time_us\":" << simUs
          << ",\"range_m\":" << range
          << ",\"bearing_rad\":" << bearing
          << ",\"nlos\":" << (nlos ? "true" : "false")
          << ",\"quality\":" << (nlos ? .25 : .95)
          << ",\"valid\":" << (valid ? "true" : "false") << "}\n";
      Send(i, uwb.str());
    }
  }
}

/**
 * @brief 在物理更新后按仿真周期触发观测发布。
 * @param _info 仿真时刻与暂停状态。
 * @param _ecm 只读实体组件视图。
 * @note 时间倒退会绕过周期等待；不补发未执行的历史周期。
 */
void PeerHeightSystem::PostUpdate(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
  if (socketFd_ < 0)
    return;
  const auto simUs = SimTimeUs(_info);
  if (lastPublishSimUs_ >= 0 && simUs >= lastPublishSimUs_ &&
      simUs - lastPublishSimUs_ < publishPeriodUs_)
    return;
  lastPublishSimUs_ = simUs;
  Publish(_info, _ecm);
}

}  // namespace drone_sim

GZ_ADD_PLUGIN(drone_sim::PeerHeightSystem,
              gz::sim::System,
              gz::sim::ISystemConfigure,
              gz::sim::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(drone_sim::PeerHeightSystem, "drone_sim::PeerHeightSystem")
