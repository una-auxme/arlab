#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <map>
#include <string>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit_msgs/action/move_group.hpp>

class ArmMotion {
public:
  ArmMotion(const rclcpp::Node::SharedPtr& node, const std::string& group);

  void moveToPose(const geometry_msgs::msg::Pose& target);
  void moveLinearToPose(const geometry_msgs::msg::Pose &target,
                       double eef_step = 0.01,
                       double jump_threshold = 0.0,
                       double min_fraction = 0.95);
  void moveToPoseBoxGoal(const geometry_msgs::msg::Pose &target,
                       double pos_tol,
                       bool use_orientation = false,
                       double ori_tol = 5e-2,
                       const std::string &eef_link = "tool0",
                       const std::string &frame_id = "world",
                       const std::string &planner_id = "");
  void moveToJointPos(const std::map<std::string, double>& joints);
  void moveToHome();
  geometry_msgs::msg::Pose makeApproachPose(const geometry_msgs::msg::Pose &target, double dz_world, double dz_eef);

private:
  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::MoveGroupInterface mgi_;
  rclcpp_action::Client<moveit_msgs::action::MoveGroup>::SharedPtr move_group_client_;
  std::map<std::string, double> joints_home_;
};
