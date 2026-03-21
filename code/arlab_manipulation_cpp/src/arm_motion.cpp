#include "arlab_manipulation_cpp/arm_motion.hpp"

#include "arlab_manipulation_cpp/orchestrator_listener.hpp"
#include "arlab_manipulation_cpp/manipulator_exception.hpp"

#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit_msgs/msg/bounding_volume.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <moveit/robot_state/conversions.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/trajectory_processing/trajectory_tools.hpp>


#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit_msgs/action/move_group.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace {

  constexpr char kMoveActionName[] = "move_action";
  constexpr char kDefaultReferenceFrame[] = "world";
  constexpr char kDefaultEndEffectorLink[] = "tool0";
  constexpr int kPlanningAttempts = 20;
  constexpr double kPlanningTimeSeconds = 5.0;
  constexpr double kGoalPositionTolerance = 1e-2;
  constexpr double kGoalOrientationTolerance = 5e-2;
  constexpr int kActionServerWaitSeconds = 3;

  const std::map<std::string, double> kHomeJoints = {
      {"shoulder_pan_joint", -1.6},
      {"shoulder_lift_joint", -1.1449},
      {"elbow_joint", -2.4225},
      {"wrist_1_joint", -3.4335},
      {"wrist_2_joint", -1.6580},
      {"wrist_3_joint", -0.0698},
  };
}  // namespace

ArmMotion::ArmMotion(const rclcpp::Node::SharedPtr& node,
                    const std::string& group)
    : node_(node), mgi_(node_, group) {
  if (!node_) {
    throw ManipulationException(
      arlab_common_interfaces::msg::ManipulationResponse::
      ORCHESTRATOR_LISTENER_NODE_NULL);
  }

  mgi_.setNumPlanningAttempts(kPlanningAttempts);
  mgi_.setPlanningTime(kPlanningTimeSeconds);
  mgi_.setGoalPositionTolerance(kGoalPositionTolerance);
  mgi_.setGoalOrientationTolerance(kGoalOrientationTolerance);
  mgi_.setPoseReferenceFrame(kDefaultReferenceFrame);

  move_group_client_ =
      rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(
        node_, kMoveActionName);

  joints_home_ = kHomeJoints;
}

geometry_msgs::msg::Pose ArmMotion::MakeApproachPose(
    const geometry_msgs::msg::Pose& target,
    double dz_tool,
    double dz_world) {
  geometry_msgs::msg::Pose approach = target;

  tf2::Quaternion q;
  tf2::fromMsg(target.orientation, q);
  tf2::Matrix3x3 rotation(q);

  const tf2::Vector3 tool_z_world =
        rotation * tf2::Vector3(0.0, 0.0, 1.0);

  approach.position.x += tool_z_world.x() * dz_tool;
  approach.position.y += tool_z_world.y() * dz_tool;
  approach.position.z += tool_z_world.z() * dz_tool;
  approach.position.z += dz_world;

  return approach;
}

void ArmMotion::MoveToPose(const geometry_msgs::msg::Pose& target)
{
  mgi_.clearPoseTargets();
  mgi_.setStartStateToCurrentState();
  if (!mgi_.setPoseTarget(target, "tool0")) {
    RCLCPP_ERROR(node_->get_logger(), "Set pose target failed");
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto plan_result = mgi_.plan(plan);
  if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "MoveIt planning failed: %s", plan_result.message.c_str());
    throw ManipulationException(plan_result);
  }

  auto execute_result = mgi_.execute(plan);
  if (execute_result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "MoveIt execution failed: %s", execute_result.message.c_str());
    throw ManipulationException(execute_result);
  }
  return;
}

void ArmMotion::MoveLinearToPose(const geometry_msgs::msg::Pose& target,
                                 double eef_step, double jump_threshold,
                                 double min_fraction) {
  mgi_.setStartStateToCurrentState();

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(target);

  moveit_msgs::msg::RobotTrajectory traj_msg;

  const double fraction = mgi_.computeCartesianPath(
      waypoints, eef_step, jump_threshold,
      traj_msg, true
  );

  if (fraction < min_fraction) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Cartesian path fraction too low: %.3f (min %.3f)", fraction, min_fraction);
    throw ManipulationException(moveit::core::MoveItErrorCode::PLANNING_FAILED);
  }

  robot_trajectory::RobotTrajectory rt(mgi_.getRobotModel(), mgi_.getName());
  rt.setRobotTrajectoryMsg(*mgi_.getCurrentState(), traj_msg);

  const double vel = mgi_.getMaxVelocityScalingFactor();
  const double acc = mgi_.getMaxAccelerationScalingFactor();

  const bool time_ok = trajectory_processing::applyTOTGTimeParameterization(rt, vel, acc);
  if (!time_ok){
    RCLCPP_ERROR(node_->get_logger(), "TOTG time parameterization failed");
    throw ManipulationException(moveit::core::MoveItErrorCode::FAILURE);
  }

  rt.getRobotTrajectoryMsg(traj_msg);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory = traj_msg;

  auto exec_res = mgi_.execute(plan);
  if (exec_res != moveit::core::MoveItErrorCode::SUCCESS){
    RCLCPP_ERROR(node_->get_logger(), "Cartesian execution failed: %s", exec_res.message.c_str());
    throw ManipulationException(exec_res);
  }
}

void ArmMotion::MoveToPoseBoxGoal(const geometry_msgs::msg::Pose& target,
                                  double pos_tol, bool use_orientation,
                                  double ori_tol, const std::string& eef_link,
                                  const std::string& frame_id,
                                  const std::string& planner_id) {
  using MoveGroup = moveit_msgs::action::MoveGroup;

  if (!move_group_client_->wait_for_action_server(std::chrono::seconds(3))) {
    throw ManipulationException(
      arlab_common_interfaces::msg::ManipulationResponse::
      TIMED_OUT);
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
  box_pose.orientation.w = 1.0;

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

  if (use_orientation){
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
  if (!planner_id.empty()) {
    req.planner_id = planner_id;
  }

  // IMPORTANT: goal_constraints is a list of Constraints (each a conjunction)
  req.goal_constraints.clear();
  req.goal_constraints.push_back(cons);

  // Start state = current state from MoveGroupInterface
  // (MoveIt will also accept empty start_state and use current, but explicit is safer)
  auto current_state = mgi_.getCurrentState(1.0);
  if (current_state) {
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
  if (!goal_handle) {
    throw ManipulationException(
      arlab_common_interfaces::msg::ManipulationResponse::
      UNKNOWN_FAILURE, std::string{"Failed to send MoveGroup goal."});
  }

  auto result_future = move_group_client_->async_get_result(goal_handle);
  auto wrapped_result = result_future.get();

  auto code_to_str = [](rclcpp_action::ResultCode c) {
    switch (c) {
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

  if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    std::stringstream ss;
    ss << "MoveGroup action result_code=" << code_to_str(wrapped_result.code);

    if (wrapped_result.result)
      ss << ", moveit_error_code=" << wrapped_result.result->error_code.val;

    RCLCPP_ERROR(node_->get_logger(), "%s", ss.str().c_str());
    throw ManipulationException(arlab_common_interfaces::msg::ManipulationResponse::UNKNOWN_FAILURE, ss.str());
  }
}

void ArmMotion::MoveToJointPos(const std::map<std::string, double>& joints)
{
  mgi_.setJointValueTarget(joints);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto plan_result = mgi_.plan(plan);
  if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(),
      "MoveIt planning (Joints) failed: %s", plan_result.message.c_str());
    throw ManipulationException(plan_result);
    return;
  }

  auto execute_result = mgi_.execute(plan);
  if (execute_result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(),
      "MoveIt execution (Joints) failed: %s", execute_result.message.c_str());
    throw ManipulationException(execute_result);
    return;
  }
  return;
}

void ArmMotion::MoveToHome()
{
  MoveToJointPos(joints_home_);
}
