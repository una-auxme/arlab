
#include "arlab_manipulation_cpp/job_runner.hpp"
#include "arlab_manipulation_cpp/arm_motion.hpp"
#include "arlab_manipulation_cpp/hand_motion.hpp"

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

  if (cmd == "open")
  {
    hand_.open();
    return;
  }
  if (cmd == "close")
  {
    hand_.close();
    return;
  }
  if (cmd == "point")
  {
    // hand_.point();
    return;
  }
  if (cmd == "home")
  {
    (void)arm_.moveToHome();
    return;
  }
  if (cmd == "move")
  {
    (void)arm_.moveToPoseBoxGoal(msg.pose);
    return;
  }
  if (cmd == "pick")
  {
    // einfache Sequenz
    hand_.open();
    (void)arm_.moveToHome();
    (void)arm_.moveToJointPos(createJointPos(0.0, -1.5707, 0.0, 0.0, 0.0, 0.0));
    (void)arm_.moveToJointPos(createJointPos(0.0, -1.7104, 1.2915, 0.4188, 1.5358, -1.5708));
    // kurz vor Pose fahren:
    //(void)arm_.moveToPose(createPose(0.1, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0));
    //  zur Pose fahren:
    //(void)arm_.moveToPose(createPose(0.3, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0));
    //(void)arm_.moveToPose(msg.pose);
    hand_.close();
    // von Pose weg fahren:
    //(void)arm_.moveToPose(createPose(0.3, 0.0, 0.4, 0.0, 0.0, 0.0, 1.0));
    (void)arm_.moveToHome();
    return;
  }

  RCLCPP_WARN(logger_, "Unknown command: %s", cmd.c_str());
  //Unknown Job Command
  throw std::runtime_error("-35");
}
