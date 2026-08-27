cmake_minimum_required(VERSION 3.16)
project(capability_mission_planner LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(BUILD_TESTING "Build tests" ON)
option(BUILD_IMAGE_MAP_DEMO "Build the PNG map validation example" ON)
option(BUILD_OFFLINE_MAP_ADAPTER "Build the offline Nav2 map adapter" ON)

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

if(BUILD_OFFLINE_MAP_ADAPTER)
  find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)
  find_package(yaml-cpp REQUIRED)
  add_library(capability_mission_planner_map_adapter
    src/nav2_multi_map_adapter/map_bundle_loader.cpp
    src/nav2_multi_map_adapter/multi_map_path_planner.cpp
    src/nav2_multi_map_adapter/offline_mission_planner.cpp
    src/nav2_multi_map_adapter/multi_map_coordinator.cpp
    src/nav2_multi_map_adapter/plan_exporter.cpp)
  target_include_directories(capability_mission_planner_map_adapter
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(capability_mission_planner_map_adapter
    PUBLIC capability_mission_planner
    PRIVATE yaml-cpp ${OpenCV_LIBS})
  target_compile_options(capability_mission_planner_map_adapter
    PRIVATE -Wall -Wextra -Wpedantic)

  add_executable(offline_map_demo examples/offline_map_demo.cpp)
  target_link_libraries(offline_map_demo PRIVATE capability_mission_planner_map_adapter)
endif()

if(BUILD_IMAGE_MAP_DEMO)
  find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)
  add_executable(image_map_demo examples/image_map_demo.cpp)
  target_include_directories(image_map_demo PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(image_map_demo PRIVATE
    capability_mission_planner ${OpenCV_LIBS})
endif()

if(BUILD_TESTING)
  enable_testing()
  add_executable(mission_tests test/mission_tests.cpp)
  target_link_libraries(mission_tests PRIVATE capability_mission_planner)
  add_test(NAME mission_tests COMMAND mission_tests)
  if(TARGET image_map_demo)
    add_test(NAME image_map_validation COMMAND image_map_demo
      ${CMAKE_CURRENT_SOURCE_DIR}/tmp/image.png
      ${CMAKE_CURRENT_BINARY_DIR}/capability_mission_plan.png)
  endif()
  if(TARGET offline_map_demo)
    add_executable(offline_adapter_tests test/offline_adapter_tests.cpp)
    target_link_libraries(offline_adapter_tests PRIVATE
      capability_mission_planner_map_adapter)
    add_test(NAME offline_adapter_tests COMMAND offline_adapter_tests
      ${CMAKE_CURRENT_SOURCE_DIR}/tmp/栅格示例/1
      ${CMAKE_CURRENT_SOURCE_DIR}/tmp/栅格示例/2)
    add_test(NAME offline_single_map_validation COMMAND offline_map_demo
      ${CMAKE_CURRENT_SOURCE_DIR}/tmp/栅格示例/1
      ${CMAKE_CURRENT_BINARY_DIR}/offline_single_map_output)
    add_test(NAME offline_multi_map_validation COMMAND offline_map_demo
      ${CMAKE_CURRENT_SOURCE_DIR}/tmp/栅格示例/2
      ${CMAKE_CURRENT_BINARY_DIR}/offline_multi_map_output)
  endif()
endif()
