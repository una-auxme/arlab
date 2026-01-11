#include "arlab_manipulation_cpp/arm_motion.hpp"

ArmMotion::ArmMotion(const rclcpp::Node::SharedPtr &node, const std::string &group)
    : node_(node), mgi_(node_, group)
{
  mgi_.setNumPlanningAttempts(20);
  mgi_.setPlanningTime(5.0);
  mgi_.setGoalPositionTolerance(1e-2);
  mgi_.setGoalOrientationTolerance(5e-2);
  mgi_.setPoseReferenceFrame("world");
  // mgi_.setPlannerId("RRTConnectkConfigDefault");

  joints_home_ = {
      {"shoulder_pan_joint", -1.6},
      {"shoulder_lift_joint", -1.1449},
      {"elbow_joint", -2.4225},
      {"wrist_1_joint", -3.4335},
      {"wrist_2_joint", -1.6580},
      {"wrist_3_joint", -0.0698}};
}

void ArmMotion::moveToPose(const geometry_msgs::msg::Pose &target)
{
  mgi_.clearPoseTargets();
  mgi_.setStartStateToCurrentState();
  if (!mgi_.setPoseTarget(target, "tool0"))
  {
    RCLCPP_ERROR(node_->get_logger(), "Set pose target failed");
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto plan_result = mgi_.plan(plan);
  if (plan_result != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "MoveIt planning failed: %d", plan_result.val);

    std::string code_str = moveit::core::errorCodeToString(plan_result);
    throw std::runtime_error(code_str.c_str());
    return;
  }

  auto execute_result = mgi_.execute(plan);
  if (execute_result != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "MoveIt execution failed: %d", execute_result.val);

    std::string code_str = moveit::core::errorCodeToString(execute_result);
    throw std::runtime_error(code_str.c_str());
    return;
  }
  return;
}

void ArmMotion::moveToJointPos(const std::map<std::string, double> &joints)
{
  mgi_.setJointValueTarget(joints);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto plan_result = mgi_.plan(plan);
  if (plan_result != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "MoveIt planning (Joints) failed: %d", plan_result.val);
    std::string code_str = moveit::core::errorCodeToString(plan_result);
    throw std::runtime_error(code_str.c_str());
    return;
  }

  auto execute_result = mgi_.execute(plan);
  if (execute_result != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "MoveIt execution (Joints) failed: %d", execute_result.val);
    std::string code_str = moveit::core::errorCodeToString(execute_result);
    throw std::runtime_error(code_str.c_str());
    return;
  }
  return;
}

void ArmMotion::moveToHome()
{
  moveToJointPos(joints_home_);
}
