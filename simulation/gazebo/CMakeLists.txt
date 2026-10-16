cmake_minimum_required(VERSION 3.16)
project(drone_gazebo_sensors LANGUAGES CXX)

find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(Threads REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(nlohmann_json 3.10 REQUIRED)

add_library(actuation_recorder SHARED src/actuation_recorder.cc)
target_compile_features(actuation_recorder PRIVATE cxx_std_17)
target_compile_options(actuation_recorder PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(actuation_recorder PRIVATE gz-sim8::gz-sim8 nlohmann_json::nlohmann_json)

# 基地滑轨装甲板：模型关节驱动，接触由世界 Contact 系统处理。
add_library(moving_armor_system SHARED src/moving_armor_system.cc)
target_compile_features(moving_armor_system PRIVATE cxx_std_17)
target_compile_options(moving_armor_system PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(moving_armor_system PRIVATE gz-sim8::gz-sim8)

add_library(peer_height_system SHARED
  src/peer_height_system.cc
  ../companion/src/terrain.cpp
)
target_include_directories(peer_height_system PRIVATE
  include ../companion/include)
target_compile_features(peer_height_system PRIVATE cxx_std_17)
target_compile_options(peer_height_system PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(peer_height_system PRIVATE gz-sim8::gz-sim8 Eigen3::Eigen)

# ToF 射线可视化：订阅三向深度图，向 /marker 服务发送 GUI 线标记。
add_library(tof_ray_visualizer SHARED src/tof_ray_visualizer.cc)
target_compile_features(tof_ray_visualizer PRIVATE cxx_std_17)
target_compile_options(tof_ray_visualizer PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(tof_ray_visualizer PRIVATE
  gz-sim8::gz-sim8
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)

option(DRONE_GAZEBO_BUILD_TESTS "Build sensor and visualization tests" ON)
if(DRONE_GAZEBO_BUILD_TESTS)
  enable_testing()
  add_executable(test_actuation_recorder ../tests/test_actuation_recorder.cpp)
  target_compile_features(test_actuation_recorder PRIVATE cxx_std_17)
  target_compile_options(test_actuation_recorder PRIVATE -Wall -Wextra -Wpedantic -UNDEBUG)
  target_link_libraries(test_actuation_recorder PRIVATE gz-sim8::gz-sim8 nlohmann_json::nlohmann_json)
  add_test(NAME actuation_recorder COMMAND test_actuation_recorder)
  add_executable(test_uwb_occlusion
    ../tests/test_uwb_occlusion.cpp ../companion/src/terrain.cpp)
  target_include_directories(test_uwb_occlusion PRIVATE include ../companion/include)
  target_compile_features(test_uwb_occlusion PRIVATE cxx_std_17)
  target_compile_options(test_uwb_occlusion PRIVATE -Wall -Wextra -Wpedantic -UNDEBUG)
  target_link_libraries(test_uwb_occlusion PRIVATE Eigen3::Eigen)
  add_test(NAME uwb_occlusion COMMAND test_uwb_occlusion)

  add_executable(test_tof_ray_visualizer ../tests/test_tof_ray_visualizer.cpp)
  target_compile_features(test_tof_ray_visualizer PRIVATE cxx_std_17)
  target_compile_options(test_tof_ray_visualizer PRIVATE -Wall -Wextra -Wpedantic -UNDEBUG)
  target_link_libraries(test_tof_ray_visualizer PRIVATE
    gz-sim8::gz-sim8 gz-transport13::gz-transport13 gz-msgs10::gz-msgs10
    Threads::Threads)
  add_test(NAME tof_ray_visualizer COMMAND test_tof_ray_visualizer)
endif()

option(DRONE_GAZEBO_BUILD_INTEGRATION_TESTS "Build tests requiring installed Gazebo physics systems" OFF)
if(DRONE_GAZEBO_BUILD_INTEGRATION_TESTS)
  enable_testing()
  add_executable(test_gazebo_imu_impulse ../tests/test_gazebo_imu_impulse.cpp)
  target_compile_features(test_gazebo_imu_impulse PRIVATE cxx_std_17)
  target_compile_options(test_gazebo_imu_impulse PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(test_gazebo_imu_impulse PRIVATE
    gz-sim8::gz-sim8 gz-transport13::gz-transport13 gz-msgs10::gz-msgs10 nlohmann_json::nlohmann_json)
  add_test(NAME gazebo_imu_impulse COMMAND test_gazebo_imu_impulse
    ${CMAKE_CURRENT_SOURCE_DIR}/../tests/fixtures/imu_impulse.sdf
    ${CMAKE_CURRENT_BINARY_DIR}/imu_impulse_samples.json)
  set_tests_properties(gazebo_imu_impulse PROPERTIES
    ENVIRONMENT "GZ_PARTITION=drone-imu-impulse-check" TIMEOUT 30)
endif()

add_executable(sensor_gateway src/sensor_gateway.cc)
target_compile_features(sensor_gateway PRIVATE cxx_std_17)
target_compile_options(sensor_gateway PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(sensor_gateway PRIVATE
  gz-transport13::gz-transport13 gz-msgs10::gz-msgs10 Threads::Threads)
