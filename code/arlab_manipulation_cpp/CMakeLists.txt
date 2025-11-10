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
find_package(arlab_common_interfaces REQUIRED)

add_executable(Manipulation_CPP src/orchestrator_listener.cpp src/job_runner.cpp src/arm_motion.cpp src/hand_motion.cpp)

target_include_directories(Manipulation_CPP PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_compile_features(Manipulation_CPP PUBLIC cxx_std_17)

ament_target_dependencies(
  Manipulation_CPP
  moveit_ros_planning_interface
  rclcpp
  geometry_msgs
  std_msgs
  arlab_common_interfaces
)

install(TARGETS Manipulation_CPP
  DESTINATION lib/${PROJECT_NAME})

ament_package()
