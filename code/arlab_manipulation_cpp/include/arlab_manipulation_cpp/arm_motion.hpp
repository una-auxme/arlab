#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <map>
#include <string>

class ArmMotion {
public:
  ArmMotion(const rclcpp::Node::SharedPtr& node, const std::string& group);

  void moveToPose(const geometry_msgs::msg::Pose& target);
  void moveToPoseBoxGoal(const geometry_msgs::msg::Pose &target,
                       double pos_tol,
                       bool use_orientation = false,
                       double ori_tol = 5e-2,
                       const std::string &eef_link = "tool0",
                       const std::string &frame_id = "world");
  void moveToJointPos(const std::map<std::string, double>& joints);
  void moveToHome(); // Nutzt konstante Pose
  

private:
  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::MoveGroupInterface mgi_;
  std::map<std::string, double> joints_home_;
};
