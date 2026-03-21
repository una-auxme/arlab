
#include "arlab_manipulation_cpp/job_runner.hpp"

#include "arlab_common_interfaces/msg/manipulation_response.hpp"
#include "arlab_common_interfaces/msg/manipulation_command.hpp"
#include "arlab_manipulation_cpp/arm_motion.hpp"
#include "arlab_manipulation_cpp/hand_motion.hpp"
#include "arlab_manipulation_cpp/manipulator_exception.hpp"

namespace {

constexpr double kBoxPositionTolerance = 0.1;
constexpr double kBoxOrientationTolerance = 0.05;
constexpr double kApproachDistance = 0.1;
constexpr double kLiftOffset = 0.05;
constexpr char kEndEffectorLink[] = "tcp_helper";
constexpr char kReferenceFrame[] = "world";
constexpr char kPlannerId[] = "RRTConnectkConfigDefault";

}  // namespace

JobRunner::JobRunner(rclcpp::Node& node, ArmMotion& arm, HandMotion& hand)
    : logger_(node.get_logger()), arm_(arm), hand_(hand) {}

geometry_msgs::msg::Pose JobRunner::CreatePose(double x, double y, double z,
                                              double qx, double qy, double qz,
                                              double qw) const {
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.x = qx;
  pose.orientation.y = qy;
  pose.orientation.z = qz;
  pose.orientation.w = qw;
  return pose;
}

std::map<std::string, double> JobRunner::CreateJointPos(
    double shoulder_pan_joint, double shoulder_lift_joint,
    double elbow_joint, double wrist_1_joint, double wrist_2_joint,
    double wrist_3_joint) const {
  return {
    {"shoulder_pan_joint", shoulder_pan_joint},
    {"shoulder_lift_joint", shoulder_lift_joint},
    {"elbow_joint", elbow_joint},
    {"wrist_1_joint", wrist_1_joint},
    {"wrist_2_joint", wrist_2_joint},
    {"wrist_3_joint", wrist_3_joint}
  };
}

void JobRunner::Run(const arlab_common_interfaces::msg::OrchestratorData& msg) {
  const std::string cmd = msg.cmd.data;
  RCLCPP_INFO(logger_, "JobRunner received cmd='%s'", cmd.c_str());

  if (cmd == arlab_common_interfaces::msg::ManipulationCommand::COMMAND_OPEN) {
    hand_.Open();
  } else if (cmd == arlab_common_interfaces::msg::ManipulationCommand::COMMAND_CLOSE) {
    hand_.Close();
  } else if (cmd == arlab_common_interfaces::msg::ManipulationCommand::COMMAND_HOME) {
    arm_.MoveToHome();
  } else if (cmd == arlab_common_interfaces::msg::ManipulationCommand::COMMAND_MOVE) {
    arm_.MoveToPose(msg.pose);
  } else if (
      cmd == arlab_common_interfaces::msg::ManipulationCommand::COMMAND_MOVE_TO_BOX) {
    arm_.MoveToPoseBoxGoal(msg.pose, kBoxPositionTolerance, false,
                          kBoxOrientationTolerance, kEndEffectorLink,
                          kReferenceFrame,kPlannerId);
  } else if (cmd == arlab_common_interfaces::msg::ManipulationCommand::COMMAND_PICK) {
    hand_.Open();
    auto approach_pose = arm_.MakeApproachPose(msg.pose, 0.1, 0.05);
    arm_.MoveToPose(approach_pose);
    arm_.MoveToPose(msg.pose);
    hand_.Close();
    arm_.MoveToPose(approach_pose);
    arm_.MoveToHome();
  } else if (cmd == arlab_common_interfaces::msg::ManipulationCommand::COMMAND_PLACE) {
    auto approach_pose = arm_.MakeApproachPose(msg.pose, 0.1, 0.05);
    arm_.MoveToPose(approach_pose);
    arm_.MoveToPose(msg.pose);
    hand_.Open();
    arm_.MoveToPose(approach_pose);
    arm_.MoveToHome();
  } else {
    RCLCPP_WARN(logger_, "Unknown command: %s", cmd.c_str());
    throw ManipulationException(arlab_common_interfaces::msg::ManipulationResponse::UNKNOWN_JOB_COMMAND);
  }

  return;
}
