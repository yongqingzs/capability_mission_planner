cmake_minimum_required(VERSION 3.16)
project(capability_mission_planner LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(BUILD_TESTING "Build tests" ON)

add_library(capability_mission_planner
  src/grid_path_planner.cpp
  src/mission_planner.cpp
  src/cbs_coordinator.cpp
  src/task_metadata.cpp)

target_include_directories(capability_mission_planner
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_compile_options(capability_mission_planner PRIVATE -Wall -Wextra -Wpedantic)

add_executable(mission_demo examples/mission_demo.cpp)
target_link_libraries(mission_demo PRIVATE capability_mission_planner)

if(BUILD_TESTING)
  enable_testing()
  add_executable(mission_tests test/mission_tests.cpp)
  target_link_libraries(mission_tests PRIVATE capability_mission_planner)
  add_test(NAME mission_tests COMMAND mission_tests)
endif()
