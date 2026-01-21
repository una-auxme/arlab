
#include "arlab_manipulation_cpp/job_runner.hpp"
#include "arlab_manipulation_cpp/arm_motion.hpp"
#include "arlab_manipulation_cpp/hand_motion.hpp"
#include "arlab_manipulation_cpp/orchestrator_listener.hpp"
#include "arlab_manipulation_cpp/manipulator_exception.hpp"
#include "arlab_common_interfaces/msg/manipulation_response.hpp"
#include "arlab_common_interfaces/msg/manipulation_command.hpp"

JobRunner::JobRunner(rclcpp::Node &node, ArmMotion &arm, HandMotion &hand)
    : logger_(node.get_logger()), arm_(arm), hand_(hand) {}

geometry_msgs::msg::Pose JobRunner::createPose(double x, double y, double z, double qx, double qy, double qz, double qw) const
{
  geometry_msgs::msg::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.position.z = z;
  p.orientation.x = qx;
  p.orientation.y = qy;
  p.orientation.z = qz;
  p.orientation.w = qw;
  return p;
}

std::map<std::string, double> JobRunner::createJointPos(
    double shoulder_pan_joint, double shoulder_lift_joint, double elbow_joint,
    double wrist_1_joint, double wrist_2_joint, double wrist_3_joint)
{
  return {
      {"shoulder_pan_joint", shoulder_pan_joint},
      {"shoulder_lift_joint", shoulder_lift_joint},
      {"elbow_joint", elbow_joint},
      {"wrist_1_joint", wrist_1_joint},
      {"wrist_2_joint", wrist_2_joint},
      {"wrist_3_joint", wrist_3_joint}};
}

void JobRunner::run(const arlab_common_interfaces::msg::OrchestratorData &msg)
{
  const std::string cmd = msg.cmd.data;
  RCLCPP_INFO(logger_, "JobRunner received cmd='%s'", cmd.c_str());

  if (cmd == arlab_common_interfaces::msg::ManipulationCommand::COMMAND_OPEN)
  {
    hand_.open();
    return;
  }
  if (cmd == arlab_common_interfaces::msg::ManipulationCommand::COMMAND_CLOSE)
  {
    hand_.close();
    return;
  }
  if (cmd == arlab_common_interfaces::msg::ManipulationCommand::COMMAND_HOME)
  {
    arm_.moveToHome();
    return;
  }
  if (cmd == arlab_common_interfaces::msg::ManipulationCommand::COMMAND_MOVE)
  {
    arm_.moveToPose(msg.pose);
    return;
  }
  if (cmd == arlab_common_interfaces::msg::ManipulationCommand::COMMAND_MOVE_TO_BOX)
  {
    arm_.moveToPoseBoxGoal(
        msg.pose,
        0.1,                       // pos_tol
        false,                     // Orientierung
        0.05,                      // or_tol
        "tcp_helper",              // Endeffektor-Link
        "world",                   // Referenzframe
        "RRTConnectkConfigDefault" // optionaler Planner
    );
    return;
  }
  if (cmd == arlab_common_interfaces::msg::ManipulationCommand::COMMAND_PICK)
  {
    hand_.open();
    //(void)arm_.moveToHome();
    //(void)arm_.moveToPose(msg.pose);
    hand_.close();
    (void)arm_.moveToHome();
    return;
  }
  if (cmd == arlab_common_interfaces::msg::ManipulationCommand::COMMAND_PLACE)
  {
    arm_.moveToPose(msg.pose);
    hand_.open();
    arm_.moveToHome();
    return;
  }

  RCLCPP_WARN(logger_, "Unknown command: %s", cmd.c_str());
  throw ManipulationException(arlab_common_interfaces::msg::ManipulationResponse::UNKNOWN_JOB_COMMAND);
}
