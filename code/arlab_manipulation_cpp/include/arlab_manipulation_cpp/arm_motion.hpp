// -----------------------------------------------------------------------------
// File: arm_motion.hpp
// Package: arlab_manipulation_cpp
// Maintainer: Leonie Schmidt <leonie1.schmidt@uni-a.de>
//
// Declares ArmMotion, a high-level wrapper around MoveIt's MoveGroupInterface
// for the manipulator. It exposes joint-space and Cartesian planning,
// box-constrained goal planning via the MoveGroup action, and a utility method
// for computing approach poses. All motion calls throw ManipulationException
// on failure so that higher-level components (e.g. JobRunner) receive a
// uniform error type.
// -----------------------------------------------------------------------------

#ifndef ARLAB_MANIPULATION_CPP_ARM_MOTION_HPP_
#define ARLAB_MANIPULATION_CPP_ARM_MOTION_HPP_

#include <map>
#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

/**
 * High-level wrapper around MoveIt's MoveGroupInterface that provides pose,
 * Cartesian, box-constrained and joint-space motion for the manipulator.
 */
class ArmMotion {
  public:

    /**
     * Creates an ArmMotion helper and initialises the MoveGroupInterface with
     * package-level planning defaults.
     * @param node      Shared pointer to the owning ROS 2 node. Must not be null.
     * @param group     MoveIt planning group name (e.g. "ur_manipulator").
     * @throws ManipulationException if node is null.
     */
    explicit ArmMotion(const rclcpp::Node::SharedPtr& node, const std::string& group);

    /**
     * Plans and executes a motion to a given end-effector pose.
     * @param target    Desired end-effector pose in the world frame.
     * @throws ManipulationException on planning or execution failure.
     */
    void MoveToPose(const geometry_msgs::msg::Pose& target);

    /**
     * Plans and executes a straight-line Cartesian motion to the given pose.
     * The trajectory is time-parameterised with TOTG before execution.
     * @param target            Desired end-effector pose.
     * @param eef_step          Interpolation step size along the path [m]. Defaults to 0.01.
     * @param jump_threshold    Maximum allowed joint-space jump between waypoints.
     *                          Set to 0.0 to disable. Defaults to 0.0.
     * @param min_fraction      Minimum acceptable planned path fraction [0, 1]. Defaults to 0.95.
     * @throws ManipulationException if the planned fraction is below min_fraction,
     *         TOTG fails, or execution fails.
     */
    void MoveLinearToPose(const geometry_msgs::msg::Pose& target,
                          double eef_step = 0.01,
                          double jump_threshold = 0.0,
                          double min_fraction = 0.95);

    /**
     * Plans and executes a motion to a box-constrained region around the target
     * pose using the raw MoveGroup action.
     * @param target            Centre of the goal region and optional desired orientation.
     * @param pos_tol           Half-size of the bounding box per axis [m].
     * @param use_orientation   If true, adds an orientation constraint. Defaults to false.
     * @param ori_tol           Per-axis orientation tolerance [rad]. Defaults to 0.05.
     * @param eef_link          End-effector link name for the constraint. Defaults to "tool0".
     * @param frame_id          Reference frame for the constraint. Defaults to "world".
     * @param planner_id        MoveIt planner plugin ID. Pass empty to use the default.
     * @throws ManipulationException if the server is unavailable, the goal is
     *         rejected, or execution fails.
     */
    void MoveToPoseBoxGoal(const geometry_msgs::msg::Pose &target,
                            double pos_tol,
                            bool use_orientation = false,
                            double ori_tol = 5e-2,
                            const std::string &eef_link = "tool0",
                            const std::string &frame_id = "world",
                            const std::string &planner_id = "");

    /**
     * Plans and executes a motion to the provided joint target values.
     * @param joints    Map from joint name to target value [rad].
     * @throws ManipulationException on planning or execution failure.
     */
    void MoveToJointPos(const std::map<std::string, double>& joints);

    /**
     * Moves the manipulator to the predefined home configuration.
     * The home joint values are defined in kHomeJoints in arm_motion.cpp.
     * @throws ManipulationException on planning or execution failure.
     */
    void MoveToHome();

    /**
     * Computes an approach pose by offsetting the target along the tool Z axis
     * and the world Z axis.
     * @param target      Reference target pose.
     * @param dz_tool     Offset along the tool Z axis [m].
     * @param dz_world    Offset along the world Z axis [m].
     * @returns Approach pose.
     */
    geometry_msgs::msg::Pose MakeApproachPose(
      const geometry_msgs::msg::Pose& target,
      double dz_world, double dz_eef);

  private:
    rclcpp::Node::SharedPtr node_;
    moveit::planning_interface::MoveGroupInterface mgi_;
    rclcpp_action::Client<moveit_msgs::action::MoveGroup>::SharedPtr move_group_client_;
    std::map<std::string, double> joints_home_;
};

#endif  // ARLAB_MANIPULATION_CPP_ARM_MOTION_HPP_
