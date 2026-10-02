#pragma once

#include <array>
#include <cstdint>
#include <random>
#include <string>
#include <memory>
#include <vector>
#include <gz/math/Vector3.hh>

#include <gz/sim/System.hh>
#include <gz/sim/Types.hh>

namespace companion { class TerrainMap; }

namespace drone_sim
{

/// Gazebo-side peer observation publisher.
///
/// Publishes module-level optical-flow velocity, UWB ranges and scalar height.
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
    gz::math::Vector3d previousPosition;
    int64_t previousTimeUs{-1};
  };

  private: void RefreshEntities(const gz::sim::EntityComponentManager &_ecm);
  private: void Publish(const gz::sim::UpdateInfo &_info,
                        const gz::sim::EntityComponentManager &_ecm);
  private: bool Send(int _vehicle, const std::string &_json) const;
  private: static int64_t SimTimeUs(const gz::sim::UpdateInfo &_info);

  private: std::vector<Vehicle> vehicles_;
  private: struct Anchor { int id; gz::math::Vector3d position; };
  private: std::vector<Anchor> anchors_;
  private: bool flowEnabled_{true};
  private: double flowNoiseStdMps_{0.015};
  private: double flowScaleError_{0.005};
  private: double flowDropProbability_{0.01};
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
