#include "arlab_manipulation_cpp/arm_motion.hpp"

ArmMotion::ArmMotion(const rclcpp::Node::SharedPtr& node, const std::string& group)
: node_(node), mgi_(node_, group) {
  mgi_.setPlanningTime(5.0);
  mgi_.setGoalPositionTolerance(1e-3);
  mgi_.setGoalOrientationTolerance(5e-3);

  joints_home_ = {
    {"shoulder_pan_joint", -1.5707},
    {"shoulder_lift_joint", -0.7853},
    {"elbow_joint", -2.3561},
    {"wrist_1_joint", -1.3089},
    {"wrist_2_joint", 1.5707},
    {"wrist_3_joint", -3.1415}
  };

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

bool ArmMotion::moveToJointPos(const std::map<std::string, double>& joints)
{
  mgi_.setJointValueTarget(joints);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (mgi_.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "Planung (Joints) fehlgeschlagen");
    return false;
  }
  if (mgi_.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "Ausführung (Joints) fehlgeschlagen");
    return false;
  }
  return true;
}

bool ArmMotion::moveToHome() {
  return moveToJointPos(joints_home_);
}
