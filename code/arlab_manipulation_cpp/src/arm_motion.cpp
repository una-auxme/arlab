#include "arlab_manipulation_cpp/arm_motion.hpp"

#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/bounding_volume.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>


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

  void ArmMotion::moveToPoseBoxGoal(const geometry_msgs::msg::Pose &target,
                                  double pos_tol,
                                  bool use_orientation,
                                  double ori_tol,
                                  const std::string &eef_link,
                                  const std::string &frame_id)
{
  // Clear old targets/constraints
  mgi_.clearPoseTargets();
  mgi_.clearPathConstraints();   // important if you use constraints elsewhere
  mgi_.setStartStateToCurrentState();

  // -------------------------
  // Build BoundingVolume (BOX)
  // -------------------------
  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions.resize(3);
  box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = pos_tol * 2.0;
  box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = pos_tol * 2.0;
  box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = pos_tol * 2.0;

  geometry_msgs::msg::Pose box_pose;
  box_pose.position = target.position;
  // BoundingVolume primitive_poses expects a pose; orientation doesn’t matter for an axis-aligned box.
  box_pose.orientation.w = 1.0;

  moveit_msgs::msg::BoundingVolume bv;
  bv.primitives.push_back(box);
  bv.primitive_poses.push_back(box_pose);

  // -------------------------
  // PositionConstraint
  // -------------------------
  moveit_msgs::msg::PositionConstraint pc;
  pc.header.frame_id = frame_id;
  pc.link_name = eef_link;

  // optional offset from link frame; usually 0,0,0
  pc.target_point_offset.x = 0.0;
  pc.target_point_offset.y = 0.0;
  pc.target_point_offset.z = 0.0;

  pc.constraint_region = bv;
  pc.weight = 1.0;

  // -------------------------
  // Assemble Constraints
  // -------------------------
  moveit_msgs::msg::Constraints goal;
  goal.position_constraints.push_back(pc);

  // Optional OrientationConstraint (like your python)
  if (use_orientation)
  {
    moveit_msgs::msg::OrientationConstraint oc;
    oc.header.frame_id = frame_id;
    oc.link_name = eef_link;
    oc.orientation = target.orientation;

    oc.absolute_x_axis_tolerance = ori_tol;
    oc.absolute_y_axis_tolerance = ori_tol;
    oc.absolute_z_axis_tolerance = ori_tol;

    oc.weight = 1.0;
    goal.orientation_constraints.push_back(oc);
  }

  // -------------------------
  // IMPORTANT:
  // We want this as GOAL constraints (not path constraints).
  // MoveGroupInterface supports this via setGoalConstraints().
  // -------------------------
  mgi_.setGoalConstraints(goal);

  // Plan + execute (same style as your moveToPose)
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto plan_result = mgi_.plan(plan);
  if (plan_result != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "MoveIt planning (BoxGoal) failed: %d", plan_result.val);
    std::string code_str = moveit::core::errorCodeToString(plan_result);
    mgi_.clearGoalConstraints();
    throw std::runtime_error(code_str.c_str());
  }

  auto execute_result = mgi_.execute(plan);
  mgi_.clearGoalConstraints(); // clean up so later calls behave normally

  if (execute_result != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "MoveIt execution (BoxGoal) failed: %d", execute_result.val);
    std::string code_str = moveit::core::errorCodeToString(execute_result);
    throw std::runtime_error(code_str.c_str());
  }
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
