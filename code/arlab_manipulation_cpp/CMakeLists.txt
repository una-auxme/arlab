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
find_package(moveit_visual_tools REQUIRED)

add_executable(PoseSubscriber src/PoseSubscriber.cpp)

target_include_directories(PoseSubscriber PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_compile_features(PoseSubscriber PUBLIC cxx_std_17)

ament_target_dependencies(
  PoseSubscriber
  moveit_ros_planning_interface
  moveit_visual_tools
  rclcpp
  geometry_msgs
  std_msgs
)

install(TARGETS PoseSubscriber
  DESTINATION lib/${PROJECT_NAME})

add_executable(Moveit_init src/moveit_main.cpp src/utils.cpp)

target_include_directories(moveit_main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_compile_features(moveit_main PUBLIC cxx_std_17)

ament_target_dependencies(
  moveit_main
  moveit_ros_planning_interface
  moveit_visual_tools
  rclcpp
  geometry_msgs
  std_msgs
)

install(TARGETS moveit_main
  DESTINATION lib/${PROJECT_NAME})


ament_package()
