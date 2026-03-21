#ifndef ARLAB_MANIPULATION_CPP_ARM_MOTION_HPP_
#define ARLAB_MANIPULATION_CPP_ARM_MOTION_HPP_

#include <map>
#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// Provides high-level motion helpers for controlling the manipulator arm
// through MoveIt.
class ArmMotion {
  public:
    // Creates an arm motion helper for the given ROS 2 node and planning group.
    explicit ArmMotion(const rclcpp::Node::SharedPtr& node, const std::string& group);

    // Plans and executes a motion to the given end-effector pose.
    void MoveToPose(const geometry_msgs::msg::Pose& target);

    // Plans and executes a Cartesian motion to the given target pose.
    //
    // eef_step controls the interpolation step size in meters.
    // jump_threshold controls the allowed joint-space jump threshold.
    // min_fraction specifies the minimum acceptable fraction of the Cartesian
    // path that must be planned successfully.
    void MoveLinearToPose(const geometry_msgs::msg::Pose& target,
                          double eef_step = 0.01,
                          double jump_threshold = 0.0,
                          double min_fraction = 0.95);

    // Plans and executes a motion to a box-constrained pose region around the
    // target pose.
    void MoveToPoseBoxGoal(const geometry_msgs::msg::Pose &target,
                            double pos_tol,
                            bool use_orientation = false,
                            double ori_tol = 5e-2,
                            const std::string &eef_link = "tool0",
                            const std::string &frame_id = "world",
                            const std::string &planner_id = "");

    // Plans and executes a motion to the provided joint target values.
    void MoveToJointPos(const std::map<std::string, double>& joints);

    // Moves the manipulator to the predefined home configuration.
    void MoveToHome();

    // Creates an approach pose by offsetting the target pose along the tool Z
    // axis and the world Z axis.
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
