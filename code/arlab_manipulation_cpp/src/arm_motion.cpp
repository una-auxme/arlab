#include "arlab_manipulation_cpp/arm_motion.hpp"

ArmMotion::ArmMotion(const rclcpp::Node::SharedPtr& node, const std::string& group)
: node_(node), mgi_(node_, group) {
  mgi_.setPlanningTime(5.0);
  mgi_.setGoalPositionTolerance(1e-3);
  mgi_.setGoalOrientationTolerance(5e-3);
}

bool ArmMotion::moveToPose(const geometry_msgs::msg::Pose& target) {
  mgi_.setPoseTarget(target);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (mgi_.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "Planung fehlgeschlagen");
    return false;
  }
  if (mgi_.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "Ausführung fehlgeschlagen");
    return false;
  }
  return true;
}

bool ArmMotion::moveToHome() {
  geometry_msgs::msg::Pose home;
  home.position.x = -0.12; home.position.y = 0.5; home.position.z = 0.6;
  home.orientation.x = 0.996; home.orientation.y = 0.041;
  home.orientation.z = 0.009; home.orientation.w = 0.076;
  return moveToPose(home);
}
