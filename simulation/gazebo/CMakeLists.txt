cmake_minimum_required(VERSION 3.16)
project(drone_gazebo_sensors LANGUAGES CXX)

find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(Threads REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_library(peer_height_system SHARED
  src/peer_height_system.cc
  ../companion/src/terrain.cpp
)
target_include_directories(peer_height_system PRIVATE
  include ../companion/include)
target_compile_features(peer_height_system PRIVATE cxx_std_17)
target_compile_options(peer_height_system PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(peer_height_system PRIVATE gz-sim8::gz-sim8 Eigen3::Eigen)

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
