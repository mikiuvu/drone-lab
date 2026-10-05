cmake_minimum_required(VERSION 3.16)
project(drone_gazebo_sensors LANGUAGES CXX)

find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(Threads REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

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

# ToF 射线可视化：订阅三向深度图，向 /marker 发布只对 GUI 可见的线标记。
add_library(tof_ray_visualizer SHARED src/tof_ray_visualizer.cc)
target_compile_features(tof_ray_visualizer PRIVATE cxx_std_17)
target_compile_options(tof_ray_visualizer PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(tof_ray_visualizer PRIVATE
  gz-sim8::gz-sim8
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)

option(DRONE_GAZEBO_BUILD_TESTS "Build independent UWB terrain occlusion tests" ON)
if(DRONE_GAZEBO_BUILD_TESTS)
  enable_testing()
  add_executable(test_uwb_occlusion
    ../tests/test_uwb_occlusion.cpp ../companion/src/terrain.cpp)
  target_include_directories(test_uwb_occlusion PRIVATE include ../companion/include)
  target_compile_features(test_uwb_occlusion PRIVATE cxx_std_17)
  target_compile_options(test_uwb_occlusion PRIVATE -Wall -Wextra -Wpedantic -UNDEBUG)
  target_link_libraries(test_uwb_occlusion PRIVATE Eigen3::Eigen)
  add_test(NAME uwb_occlusion COMMAND test_uwb_occlusion)
endif()

add_executable(sensor_gateway src/sensor_gateway.cc)
target_compile_features(sensor_gateway PRIVATE cxx_std_17)
target_compile_options(sensor_gateway PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(sensor_gateway PRIVATE
  gz-transport13::gz-transport13 gz-msgs10::gz-msgs10 Threads::Threads)
