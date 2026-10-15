#include "../gazebo/src/actuation_recorder.cc"
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <sdf/Root.hh>
#include <cassert>
#include <unistd.h>

int main() {
  using namespace std::chrono_literals;
  const auto directory = std::filesystem::temp_directory_path() /
      ("drone-actuation-" + std::to_string(getpid()));
  std::filesystem::create_directory(directory);
  const auto path = directory / "actual.jsonl";
  sdf::Root document;
  auto errors = document.LoadSdfString(
      "<sdf version='1.9'><model name='micro_0'><link name='base_link'/>"
      "<plugin name='recorder' filename='libactuation_recorder.so'><vehicle>0</vehicle>"
      "<publish_hz>20</publish_hz><output>" + path.string() +
      "</output></plugin></model></sdf>");
  assert(errors.empty());
  auto config = document.Element()->GetElement("model")->GetElement("plugin");
  gz::sim::EntityComponentManager ecm;
  const auto model = ecm.CreateEntity();
  ecm.CreateComponent(model, gz::sim::components::Model());
  ecm.CreateComponent(model, gz::sim::components::Name("micro_0"));
  const auto joint = ecm.CreateEntity();
  ecm.CreateComponent(joint, gz::sim::components::Joint());
  ecm.CreateComponent(joint, gz::sim::components::Name("rotor_0_joint"));
  ecm.CreateComponent(joint, gz::sim::components::ParentEntity(model));
  gz::sim::EventManager events;
  {
    drone_sim::ActuationRecorder recorder;
    assert(recorder.ConfigurePriority() == 1);
    recorder.Configure(model, config, ecm, events);
    gz::sim::UpdateInfo info;
    info.paused = false;
    info.simTime = 1s;
    recorder.PreUpdate(info, ecm);
    recorder.PostUpdate(info, ecm);
    assert(!ecm.Component<gz::sim::components::Actuators>(model));
    assert(!ecm.Component<gz::sim::components::JointVelocity>(joint));
    assert(!ecm.Component<gz::sim::components::JointVelocityCmd>(joint));
    ecm.CreateComponent(joint, gz::sim::components::JointVelocity({12.3}));
    ecm.CreateComponent(joint, gz::sim::components::JointVelocityCmd({15.0}));
    info.simTime = 1010ms;
    recorder.PostUpdate(info, ecm);
    info.simTime = 1050ms;
    recorder.PreUpdate(info, ecm);
    ecm.SetComponentData<gz::sim::components::JointVelocityCmd>(joint, {0.0});
    gz::msgs::Actuators actuators;
    actuators.add_velocity(600);
    ecm.CreateComponent(model, gz::sim::components::Actuators(actuators));
    recorder.PostUpdate(info, ecm);
    assert(ecm.Component<gz::sim::components::JointVelocityCmd>(joint)->Data()[0] == 0.0);
    assert(ecm.Component<gz::sim::components::Actuators>(model)->Data().velocity(0) == 600);
    ecm.SetComponentData<gz::sim::components::JointVelocity>(joint, {0.0});
    info.simTime = 1100ms;
    info.paused = true;
    recorder.PostUpdate(info, ecm);
    info.simTime = 1200ms;
    info.paused = false;
    recorder.PostUpdate(info, ecm);
  }
  std::ifstream input(path);
  std::vector<nlohmann::json> rows;
  for (std::string line; std::getline(input, line);) rows.push_back(nlohmann::json::parse(line));
  assert(rows.size() == 3);
  assert(rows[0]["joints"]["rotor_0_joint"]["velocity_rad_s"].is_null());
  assert(rows[0]["joints"]["rotor_0_joint"]["velocity_command_rad_s"].is_null());
  assert(rows[0]["joints"]["rotor_0_joint"]["pre_update_velocity_command_rad_s"].is_null());
  assert(rows[0]["actuator_component_velocity_rad_s"].is_null());
  assert(rows[0]["joints"]["rotor_1_joint"]["present"] == false);
  assert(rows[1]["sim_time_us"] == 1050000);
  assert(rows[1]["joints"]["rotor_0_joint"]["velocity_rad_s"][0] == 12.3);
  assert(rows[1]["joints"]["rotor_0_joint"]["velocity_command_rad_s"][0] == 0.0);
  assert(rows[1]["joints"]["rotor_0_joint"]["pre_update_velocity_command_rad_s"][0] == 15.0);
  assert(rows[1]["actuator_component_velocity_rad_s"][0] == 600);
  assert(rows[2]["joints"]["rotor_0_joint"]["velocity_rad_s"][0] == 0.0);
  assert(rows[2]["joints"]["rotor_0_joint"]["pre_update_velocity_command_rad_s"].is_null());
  bool rejected = false;
  try {
    drone_sim::ActuationRecorder duplicate;
    duplicate.Configure(model, config, ecm, events);
  } catch (const std::invalid_argument &) { rejected = true; }
  assert(rejected);
  const auto broken = directory / "broken.jsonl";
  config->GetElement("output")->Set(broken.string());
  {
    drone_sim::ActuationRecorder recorder;
    recorder.Configure(model, config, ecm, events);
    int descriptor = -1;
    for (const auto &entry : std::filesystem::directory_iterator("/proc/self/fd")) {
      std::error_code error;
      if (std::filesystem::read_symlink(entry.path(), error) == broken && !error)
        descriptor = std::stoi(entry.path().filename().string());
    }
    assert(descriptor >= 0 && close(descriptor) == 0);
    gz::sim::UpdateInfo info;
    info.paused = false;
    info.simTime = 1s;
    recorder.PostUpdate(info, ecm);
    info.simTime = 2s;
    recorder.PostUpdate(info, ecm);
    assert(std::filesystem::file_size(broken) == 0);
  }
  std::filesystem::remove(path);
  std::filesystem::remove(broken);
  std::filesystem::remove(directory);
}
