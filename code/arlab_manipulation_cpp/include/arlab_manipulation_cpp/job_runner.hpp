// -----------------------------------------------------------------------------
// File: job_runner.hpp
// Package: arlab_manipulation_cpp
// Maintainer: Leonie Schmidt <leonie1.schmidt@uni-a.de>
//             Christopher Müller <christopher.mueller@uni-a.de>
//
// Declares JobRunner, the central command dispatcher of the manipulation
// stack. It receives an OrchestratorData message from the action server,
// interprets the command, and delegates the corresponding motion
// sequence to ArmMotion and HandMotion.
// -----------------------------------------------------------------------------

#ifndef ARLAB_MANIPULATION_CPP_JOB_RUNNER_HPP_
#define ARLAB_MANIPULATION_CPP_JOB_RUNNER_HPP_

#include <map>
#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>

#include "arlab_common_interfaces/msg/orchestrator_data.hpp"

class ArmMotion;
class HandMotion;

/**
 * Central command dispatcher that maps incoming command strings to concrete
 * motion sequences executed by ArmMotion and HandMotion.
 */
class JobRunner
{
public:
  /**
   * Creates a JobRunner that delegates commands to the provided motion components.
   * @param node    ROS 2 node whose logger is used for status messages.
   * @param arm     Reference to the arm motion helper.
   * @param hand    Reference to the hand motion helper.
   */
  JobRunner(rclcpp::Node &node, ArmMotion &arm, HandMotion &hand);

  /**
   * Executes a single manipulation command described by the orchestrator message.
   * Reads incoming msg, dispatches the corresponding motion sequence, and forwards
   * msg.pose when required.
   * @param msg     OrchestratorData message.
   * @throws ManipulationException if the command is unknown or a motion operation fails.
   */
  void Run(const arlab_common_interfaces::msg::OrchestratorData &msg);

private:
  /**
   * Creates a Pose message from individual position and quaternion components.
   * Used mainly for testing and demonstration purposes to avoid the need
   * for constructing full Pose messages in the client.
   * @param x     Position X [m].
   * @param y     Position Y [m].
   * @param z     Position Z [m].
   * @param qx    Quaternion X component.
   * @param qy    Quaternion Y component.
   * @param qz    Quaternion Z component.
   * @param qw    Quaternion W component.
   * @returns Fully populated Pose message.
   */
  geometry_msgs::msg::Pose CreatePose(double x, double y, double z,
                                      double qx, double qy, double qz, double qw) const;

  /**
   * Creates a joint target map for the six manipulator arm joints.
   * Used mainly for testing and demonstration purposes to avoid the need
   * for constructing full Pose messages in the client.
   * @param shoulder_pan_joint    Target value [rad].
   * @param shoulder_lift_joint   Target value [rad].
   * @param elbow_joint           Target value [rad].
   * @param wrist_1_joint         Target value [rad].
   * @param wrist_2_joint         Target value [rad].
   * @param wrist_3_joint         Target value [rad].
   * @returns Map from joint name to target value [rad].
   */
  std::map<std::string, double> CreateJointPos(
      double shoulder_pan_joint, double shoulder_lift_joint,
      double elbow_joint, double wrist_1_joint, double wrist_2_joint,
      double wrist_3_joint) const;

  rclcpp::Logger logger_;
  ArmMotion &arm_;
  HandMotion &hand_;
};

#endif // ARLAB_MANIPULATION_CPP_JOB_RUNNER_HPP_
