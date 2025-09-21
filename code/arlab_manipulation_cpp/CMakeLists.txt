cmake_minimum_required(VERSION 3.8)
project(arlab_manipulation_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(std_msgs REQUIRED)
# find_package(moveit_visual_tools REQUIRED)
find_package(arlab_common_interfaces REQUIRED)

add_executable(OrchestratorSubscriber src/OrchestratorSubscriber.cpp src/utils.cpp src/job_runner.cpp)

target_include_directories(OrchestratorSubscriber PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_compile_features(OrchestratorSubscriber PUBLIC cxx_std_17)

ament_target_dependencies(
  OrchestratorSubscriber
  moveit_ros_planning_interface
  rclcpp
  geometry_msgs
  std_msgs
  arlab_common_interfaces
)

install(TARGETS OrchestratorSubscriber
  DESTINATION lib/${PROJECT_NAME})

ament_package()
