cmake_minimum_required(VERSION 3.16)
project(capability_mission_planner LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(BUILD_TESTING "Build tests" ON)
option(BUILD_IMAGE_MAP_DEMO "Build the PNG map validation example" ON)

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
endif()
