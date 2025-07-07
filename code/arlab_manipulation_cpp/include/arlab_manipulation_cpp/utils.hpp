#ifndef UTILS_HPP
#define UTILS_HPP

#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <rclcpp/logger.hpp>

bool planAndExecutePose(
  moveit::planning_interface::MoveGroupInterface &move_group_interface,
  const geometry_msgs::msg::Pose &target_pose,
  moveit_visual_tools::MoveItVisualTools &visual_tools,
  const rclcpp::Logger &logger
);

geometry_msgs::msg::Pose createPose(double x, double y, double z, double ox, double oy, double oz, double ow);

moveit_msgs::msg::CollisionObject createCollisionBox(const std::string &frame_id, std::string object_id,geometry_msgs::msg::Pose pose, double size_x, double size_y, double size_z);


#endif
