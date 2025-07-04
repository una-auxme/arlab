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

add_executable(Ur5MoveToGoal src/Ur5MoveToGoal.cpp)

target_include_directories(Ur5MoveToGoal PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_compile_features(Ur5MoveToGoal PUBLIC cxx_std_17)

ament_target_dependencies(
  Ur5MoveToGoal
  rclcpp
  geometry_msgs
  std_msgs
)

install(TARGETS Ur5MoveToGoal
  DESTINATION lib/${PROJECT_NAME})

ament_package()
