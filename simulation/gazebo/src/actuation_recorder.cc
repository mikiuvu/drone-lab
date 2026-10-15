/** @brief 只读记录仿真现有转子状态与关节速度命令，供独立执行链路诊断。 */
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Actuators.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <nlohmann/json.hpp>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace drone_sim {

class ActuationRecorder : public gz::sim::System,
                          public gz::sim::ISystemConfigure,
                          public gz::sim::ISystemConfigurePriority,
                          public gz::sim::ISystemPreUpdate,
                          public gz::sim::ISystemPostUpdate {
 public:
  // 默认电机控制器优先级为 0；在其计算后、物理积分前读取现有命令。
  gz::sim::System::PriorityType ConfigurePriority() override { return 1; }
  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &config,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &) override {
    model_ = gz::sim::Model(entity);
    if (!model_.Valid(ecm) || !config || !config->HasElement("output") || !config->HasElement("vehicle"))
      throw std::invalid_argument("actuation_recorder requires a model, vehicle and output path");
    vehicle_ = config->Get<int>("vehicle");
    const double hz = config->Get<double>("publish_hz", 20.0).first;
    if (vehicle_ < 0 || vehicle_ > 245 || !std::isfinite(hz) || hz <= 0 || hz > 100)
      throw std::invalid_argument("actuation_recorder: invalid vehicle or publish_hz");
    period_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / hz));
    const std::filesystem::path output(config->Get<std::string>("output"));
    if (!output.is_absolute() || std::filesystem::exists(output))
      throw std::invalid_argument("actuation_recorder output must be a new absolute path");
    if (config->HasElement("joint_name")) {
      auto joint = config->FindElement("joint_name");
      while (joint) {
        const auto name = joint->Get<std::string>();
        if (name.empty() || names_.size() >= 32)
          throw std::invalid_argument("actuation_recorder: invalid joint_name list");
        names_.push_back(name);
        joint = joint->GetNextElement("joint_name");
      }
    } else {
      for (int i = 0; i < 4; ++i) names_.push_back("rotor_" + std::to_string(i) + "_joint");
    }
    output_.open(output);
    if (!output_) throw std::runtime_error("actuation_recorder cannot open output");
    output_.exceptions(std::ios::failbit | std::ios::badbit);
    model_name_ = model_.Name(ecm);
  }

  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override {
    if (!due(info)) return;
    const auto &read_only = std::as_const(ecm);
    pre_commands_ = nlohmann::json::object();
    for (const auto &name : names_) {
      pre_commands_[name] = nullptr;
      const auto entity = model_.JointByName(read_only, name);
      if (entity != gz::sim::kNullEntity) {
        if (const auto *command = read_only.Component<gz::sim::components::JointVelocityCmd>(entity))
          pre_commands_[name] = command->Data();
      }
    }
    pre_time_ = info.simTime;
    pre_sampled_ = true;
  }

  void PostUpdate(const gz::sim::UpdateInfo &info,
                  const gz::sim::EntityComponentManager &ecm) override {
    if (!due(info)) return;
    previous_ = info.simTime;
    sampled_ = true;
    nlohmann::json row = {{"kind", "actuation_evaluation_only"}, {"source", "GAZEBO_ECM"},
        {"vehicle", vehicle_}, {"model", model_name_},
        {"sim_time_us", std::chrono::duration_cast<std::chrono::microseconds>(info.simTime).count()},
        {"actuator_component_velocity_rad_s", nullptr},
        {"joints", nlohmann::json::object()}};
    if (const auto *actuators = ecm.Component<gz::sim::components::Actuators>(model_.Entity())) {
      const auto &velocity = actuators->Data().velocity();
      row["actuator_component_velocity_rad_s"] = std::vector<double>(velocity.begin(), velocity.end());
    }
    for (const auto &name : names_) {
      const auto entity = model_.JointByName(ecm, name);
      nlohmann::json joint = {{"present", entity != gz::sim::kNullEntity},
          {"velocity_rad_s", nullptr}, {"velocity_command_rad_s", nullptr},
          {"pre_update_velocity_command_rad_s", nullptr}};
      if (pre_sampled_ && pre_time_ == info.simTime)
        joint["pre_update_velocity_command_rad_s"] = pre_commands_.at(name);
      // 缺失组件保留为 null；不创建组件，也不把命令回显当作物理速度。
      if (entity != gz::sim::kNullEntity) {
        if (const auto *velocity = ecm.Component<gz::sim::components::JointVelocity>(entity))
          joint["velocity_rad_s"] = velocity->Data();
        if (const auto *command = ecm.Component<gz::sim::components::JointVelocityCmd>(entity))
          joint["velocity_command_rad_s"] = command->Data();
      }
      row["joints"][name] = std::move(joint);
    }
    try {
      output_ << row.dump() << '\n';
      output_.flush();
    } catch (const std::ios_base::failure &error) {
      std::cerr << "actuation_recorder output error: " << error.what() << '\n';
      output_.exceptions(std::ios::goodbit);
      output_.close();
    }
  }

 private:
  bool due(const gz::sim::UpdateInfo &info) const {
    return output_.is_open() && !info.paused &&
        (!sampled_ || info.simTime < previous_ || info.simTime - previous_ >= period_);
  }
  gz::sim::Model model_{gz::sim::kNullEntity};
  std::ofstream output_;
  std::string model_name_;
  std::vector<std::string> names_;
  int vehicle_{0};
  bool sampled_{false};
  bool pre_sampled_{false};
  nlohmann::json pre_commands_;
  std::chrono::steady_clock::duration pre_time_{};
  std::chrono::steady_clock::duration previous_{}, period_{};
};

}  // namespace drone_sim

GZ_ADD_PLUGIN(drone_sim::ActuationRecorder, gz::sim::System,
              gz::sim::ISystemConfigure, gz::sim::ISystemConfigurePriority,
              gz::sim::ISystemPreUpdate, gz::sim::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(drone_sim::ActuationRecorder, "drone_sim::ActuationRecorder")
