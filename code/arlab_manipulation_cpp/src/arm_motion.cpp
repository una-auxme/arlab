#include "arlab_manipulation_cpp/arm_motion.hpp"
#include "arlab_manipulation_cpp/orchestrator_listener.hpp"

#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit_msgs/msg/bounding_volume.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <moveit/robot_state/conversions.hpp>

#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit_msgs/action/move_group.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>

ArmMotion::ArmMotion(const rclcpp::Node::SharedPtr &node, const std::string &group)
    : node_(node), mgi_(node_, group)
{
  mgi_.setNumPlanningAttempts(20);
  mgi_.setPlanningTime(5.0);
  mgi_.setGoalPositionTolerance(1e-2);
  mgi_.setGoalOrientationTolerance(5e-2);
  mgi_.setPoseReferenceFrame("world");
  // mgi_.setPlannerId("RRTConnectkConfigDefault");

  move_group_client_ = rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(node_, "move_action");

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
    RCLCPP_ERROR(node_->get_logger(), "MoveIt planning failed: %s", plan_result.message.c_str());

    throw ManipulationException(plan_result);
    return;
  }

  auto execute_result = mgi_.execute(plan);
  if (execute_result != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "MoveIt execution failed: %s", execute_result.message.c_str());

    throw ManipulationException(execute_result);
    return;
  }
  return;
}

void ArmMotion::moveToPoseBoxGoal(const geometry_msgs::msg::Pose &target,
                                  double pos_tol,
                                  bool use_orientation,
                                  double ori_tol,
                                  const std::string &eef_link,
                                  const std::string &frame_id,
                                  const std::string &planner_id)
{
  using MoveGroup = moveit_msgs::action::MoveGroup;

  // Ensure server is up
  if (!move_group_client_->wait_for_action_server(std::chrono::seconds(3)))
  {
    throw ManipulationException(arlab_common_interfaces::msg::ManipulationResponse::TIMED_OUT);
  }

  // -----------------------------
  // Build BoundingVolume BOX
  // -----------------------------
  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions.resize(3);
  box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = pos_tol * 2.0;
  box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = pos_tol * 2.0;
  box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = pos_tol * 2.0;

  geometry_msgs::msg::Pose box_pose;
  box_pose.position = target.position;
  box_pose.orientation.w = 1.0; // axis-aligned box

  moveit_msgs::msg::BoundingVolume bv;
  bv.primitives.push_back(box);
  bv.primitive_poses.push_back(box_pose);

  // -----------------------------
  // PositionConstraint
  // -----------------------------
  moveit_msgs::msg::PositionConstraint pc;
  pc.header.frame_id = frame_id;
  pc.link_name = eef_link;
  pc.target_point_offset.x = 0.0;
  pc.target_point_offset.y = 0.0;
  pc.target_point_offset.z = 0.0;
  pc.constraint_region = bv;
  pc.weight = 1.0;

  moveit_msgs::msg::Constraints cons;
  cons.position_constraints.push_back(pc);

  // Optional OrientationConstraint
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
    cons.orientation_constraints.push_back(oc);
  }

  // -----------------------------
  // Build MotionPlanRequest
  // -----------------------------
  moveit_msgs::msg::MotionPlanRequest req;
  req.group_name = mgi_.getName();
  req.allowed_planning_time = mgi_.getPlanningTime();
  if (!planner_id.empty())
    req.planner_id = planner_id;

  // IMPORTANT: goal_constraints is a list of Constraints (each a conjunction)
  req.goal_constraints.clear();
  req.goal_constraints.push_back(cons);

  // Start state = current state from MoveGroupInterface
  // (MoveIt will also accept empty start_state and use current, but explicit is safer)
  auto current_state = mgi_.getCurrentState(1.0);
  if (current_state)
  {
    moveit::core::robotStateToRobotStateMsg(*current_state, req.start_state);
    req.start_state.is_diff = true;
  }

  // -----------------------------
  // Send action goal
  // -----------------------------
  MoveGroup::Goal goal_msg;
  goal_msg.request = req;

  goal_msg.planning_options.plan_only = false; // execute
  goal_msg.planning_options.look_around = false;
  goal_msg.planning_options.replan = false;

  auto goal_handle_future = move_group_client_->async_send_goal(goal_msg);
  auto goal_handle = goal_handle_future.get();
  if (!goal_handle)
  {
    throw ManipulationException(arlab_common_interfaces::msg::ManipulationResponse::UNKNOWN_FAILURE, std::string{"Failed to send MoveGroup goal."});
  }

  auto result_future = move_group_client_->async_get_result(goal_handle);
  auto wrapped_result = result_future.get();

  auto code_to_str = [](rclcpp_action::ResultCode c)
  {
    switch (c)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
      return "SUCCEEDED";
    case rclcpp_action::ResultCode::ABORTED:
      return "ABORTED";
    case rclcpp_action::ResultCode::CANCELED:
      return "CANCELED";
    default:
      return "UNKNOWN";
    }
  };

  if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED)
  {
    std::stringstream ss;
    ss << "MoveGroup action result_code=" << code_to_str(wrapped_result.code);

    // Oft ist result trotzdem gesetzt → MoveItErrorCodes mitloggen
    if (wrapped_result.result)
      ss << ", moveit_error_code=" << wrapped_result.result->error_code.val;

    RCLCPP_ERROR(node_->get_logger(), "%s", ss.str().c_str());
    throw ManipulationException(arlab_common_interfaces::msg::ManipulationResponse::UNKNOWN_FAILURE, ss.str());
  }

  const auto &res = wrapped_result.result;
  if (res->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    throw ManipulationException(res->error_code.val);
  }
}

void ArmMotion::moveToJointPos(const std::map<std::string, double> &joints)
{
  mgi_.setJointValueTarget(joints);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto plan_result = mgi_.plan(plan);
  if (plan_result != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "MoveIt planning (Joints) failed: %s", plan_result.message.c_str());
    throw ManipulationException(plan_result);
    return;
  }

  auto execute_result = mgi_.execute(plan);
  if (execute_result != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "MoveIt execution (Joints) failed: %s", execute_result.message.c_str());
    throw ManipulationException(execute_result);
    return;
  }
  return;
}

void ArmMotion::moveToHome()
{
  moveToJointPos(joints_home_);
}
