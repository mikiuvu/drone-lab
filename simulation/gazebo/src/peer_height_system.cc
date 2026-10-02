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

double WrapBearing(double _angle)
{
  while (_angle > kPi)
    _angle -= kTwoPi;
  while (_angle < -kPi)
    _angle += kTwoPi;
  return _angle;
}

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

void PeerHeightSystem::Configure(
    const gz::sim::Entity &,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &,
    gz::sim::EventManager &)
{
  destinationHost_ = GetOr<std::string>(_sdf, "destination_host", destinationHost_);
  portBase_ = GetOr<int>(_sdf, "port_base", portBase_);
  const auto publishHz = GetOr<double>(_sdf, "publish_hz", 50.0);
  if (!std::isfinite(publishHz) || publishHz <= 0.0 || publishHz > 1000.0 ||
      portBase_ < 1 || portBase_ > 65532)
  {
    std::cerr << "peer_height_system: invalid publish_hz or port_base\n";
    return;
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

  for (int i = 0; i < 4; ++i)
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
            << ':' << portBase_ << ".." << portBase_ + 3
            << ", height assist at " << publishHz << " Hz\n";
}

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

int64_t PeerHeightSystem::SimTimeUs(const gz::sim::UpdateInfo &_info)
{
  return std::chrono::duration_cast<std::chrono::microseconds>(
      _info.simTime).count();
}

bool PeerHeightSystem::Send(int _vehicle, const std::string &_json) const
{
  if (socketFd_ < 0 || _vehicle < 0 || _vehicle >= 4)
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
  std::array<gz::math::Pose3d, 4> poses{};
  std::array<bool, 4> poseValid{};

  for (int i = 0; i < 4; ++i)
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

  for (int i = 0; i < 4; ++i)
  {
    if (!poseValid[i])
      continue;

    // Height assistance is explicitly authorized as a scalar world ENU z.
    // It is marked separately so the companion can keep the z source distinct
    // from peer localization and avoid treating it as a full pose truth feed.
    {
      std::ostringstream height;
      height << std::setprecision(9)
             << "{\"schema\":1,\"kind\":\"height_assist\","
             << "\"vehicle\":" << i << ",\"sim_time_us\":" << simUs
             << ",\"z_world_m\":" << poses[i].Pos().Z()
             << ",\"source\":\"SIM_HEIGHT_ASSIST\"}\n";
      Send(i, height.str());
    }

    for (int j = 0; j < 4; ++j)
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
          // Positive excess path is a configurable stress assumption only.
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
          << ",\"valid\":" << (valid ? "true" : "false") << "}\n";
      Send(i, uwb.str());
    }
  }
}

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
