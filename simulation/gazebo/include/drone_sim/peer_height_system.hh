#pragma once

#include <array>
#include <cstdint>
#include <random>
#include <string>
#include <memory>

#include <gz/sim/System.hh>
#include <gz/sim/Types.hh>

namespace companion { class TerrainMap; }

namespace drone_sim
{

/// Gazebo-side peer observation publisher.
///
/// Model poses are read only inside this plugin. The public UDP stream contains
/// pairwise UWB-like observations and the separately authorized scalar height
/// assistance field; it never contains a vehicle's global x, y, yaw or velocity.
class PeerHeightSystem final : public gz::sim::System,
                               public gz::sim::ISystemConfigure,
                               public gz::sim::ISystemPostUpdate
{
  public: PeerHeightSystem();
  public: ~PeerHeightSystem() override;

  public: void Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &_eventMgr) override;

  public: void PostUpdate(
      const gz::sim::UpdateInfo &_info,
      const gz::sim::EntityComponentManager &_ecm) override;

  private: struct Vehicle
  {
    std::string modelName;
    gz::sim::Entity modelEntity{gz::sim::kNullEntity};
  };

  private: void RefreshEntities(const gz::sim::EntityComponentManager &_ecm);
  private: void Publish(const gz::sim::UpdateInfo &_info,
                        const gz::sim::EntityComponentManager &_ecm);
  private: bool Send(int _vehicle, const std::string &_json) const;
  private: static int64_t SimTimeUs(const gz::sim::UpdateInfo &_info);

  private: std::array<Vehicle, 4> vehicles_{};
  private: int socketFd_{-1};
  private: std::string destinationHost_{"127.0.0.1"};
  private: int portBase_{16000};
  private: int64_t publishPeriodUs_{20000};
  private: double rangeNoiseStdM_{0.03};
  private: double bearingNoiseStdRad_{0.015};
  private: double rangeMinM_{0.05};
  private: double rangeMaxM_{80.0};
  private: double bearingFovRad_{6.283185307179586};
  private: double dropProbability_{0.02};
  private: double bearingPiAmbiguityProbability_{0.0};
  private: std::shared_ptr<const ::companion::TerrainMap> terrainMap_;
  private: bool terrainLoadFailed_{false};
  private: double nlosDropProbability_{0.75};
  private: double nlosRangeBiasM_{0.4};
  private: double nlosRangeNoiseStdM_{0.03};
  private: uint64_t seed_{0x50444f415f534954ULL};
  private: int64_t lastPublishSimUs_{-1};
  private: std::mt19937_64 random_;
};

}  // namespace drone_sim
