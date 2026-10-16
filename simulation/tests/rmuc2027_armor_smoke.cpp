#include "companion/rm_auto_aim.hpp"
#include "companion/vision.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>

int main(int argc, char **argv) {
  if (argc != 3) return 2;
  const auto image = companion::readPpm(argv[1]);
  if (!image) return 3;
  std::ifstream stream(argv[2]);
  nlohmann::json map;
  stream >> map;
  const auto &mission = map.at("team_mission");
  const auto &dive = mission.at("dive_attack");
  companion::CameraConfig camera;
  camera.fx_px = camera.fy_px = 320;
  camera.cx_px = 319.5;
  camera.cy_px = 179.5;
  camera.strip_center_separation_m = mission.at("visual_strip_separation_m");
  const auto result = companion::detectRmArmor(*image, camera, mission.at("visual_strip_height_m"),
                                               mission.value("visual_bottom_separation_m", 0.0));
  const Eigen::Vector3d expected(0, dive.at("entry_height_m"), dive.at("entry_standoff_m"));
  const double error = (result.position_camera_m - expected).norm();
  const nlohmann::json output = {
      {"valid", result.valid}, {"position_camera_m", {result.position_camera_m.x(), result.position_camera_m.y(), result.position_camera_m.z()}},
      {"expected_position_camera_m", {expected.x(), expected.y(), expected.z()}},
      {"position_error_m", error}, {"reprojection_error_px", result.reprojection_error_px}};
  std::cout << output.dump(2) << '\n';
  return result.valid && error <= .05 ? 0 : 1;
}
