cmake_minimum_required(VERSION 3.20)
project(drone_research_localization LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(COMPANION_BUILD_RUNNER OFF CACHE BOOL "" FORCE)
set(COMPANION_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(COMPANION_BUILD_LOAD_TEST OFF CACHE BOOL "" FORCE)
set(COMPANION_WITH_OPENCV OFF CACHE BOOL "" FORCE)

add_subdirectory(../companion companion)

find_package(nlohmann_json REQUIRED)

add_executable(localization_study localization_study.cpp)
target_link_libraries(localization_study PRIVATE companion_core nlohmann_json::nlohmann_json)
target_compile_features(localization_study PRIVATE cxx_std_17)
target_compile_options(localization_study PRIVATE -Wall -Wextra -Wpedantic)
