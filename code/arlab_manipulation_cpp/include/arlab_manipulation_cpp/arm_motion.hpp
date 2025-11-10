#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

class ArmMotion {
public:
  ArmMotion(const rclcpp::Node::SharedPtr& node, const std::string& group);
  bool moveToPose(const geometry_msgs::msg::Pose& target);
  bool moveToHome(); // Nutzt konstante Pose

private:
  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::MoveGroupInterface mgi_;
};
