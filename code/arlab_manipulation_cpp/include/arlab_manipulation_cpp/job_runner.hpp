#ifndef ARLAB_MANIPULATION_CPP_JOB_RUNNER_HPP_
#define ARLAB_MANIPULATION_CPP_JOB_RUNNER_HPP_

#include <map>
#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>

#include "arlab_common_interfaces/msg/orchestrator_data.hpp"

class ArmMotion;
class HandMotion;

// Executes manipulation jobs received from the orchestrator by dispatching
// them to the arm and hand motion components.
class JobRunner {
  public:
    // Creates a job runner that delegates commands to the provided motion
    // components.
    JobRunner(rclcpp::Node& node, ArmMotion& arm, HandMotion& hand);

    // Executes a single manipulation command described by the orchestrator
    // message.
    //
    // Throws ManipulationException if the command is unknown or if one of the
    // delegated motion operations fails.
    void Run(const arlab_common_interfaces::msg::OrchestratorData& msg);

  private:
    // Creates a pose from Cartesian position and quaternion orientation values.
    geometry_msgs::msg::Pose CreatePose(double x, double y, double z,
                                        double qx, double qy, double qz, double qw) const;

    // Creates a joint target map for the manipulator joints.
    std::map<std::string, double> CreateJointPos(
      double shoulder_pan_joint, double shoulder_lift_joint,
      double elbow_joint, double wrist_1_joint, double wrist_2_joint,
      double wrist_3_joint) const;

    rclcpp::Logger logger_;
    ArmMotion& arm_;
    HandMotion& hand_;
};

#endif  // ARLAB_MANIPULATION_CPP_JOB_RUNNER_HPP_

