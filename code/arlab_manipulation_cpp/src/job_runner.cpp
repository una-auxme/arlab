
#include "arlab_manipulation_cpp/job_runner.hpp"
#include "arlab_manipulation_cpp/arm_motion.hpp"
#include "arlab_manipulation_cpp/hand_motion.hpp"

JobRunner::JobRunner(rclcpp::Node& node, ArmMotion& arm, HandMotion& hand)
: logger_(node.get_logger()), arm_(arm), hand_(hand) {}

geometry_msgs::msg::Pose JobRunner::createPose(double x,double y,double z,double qx,double qy,double qz,double qw) const {
  geometry_msgs::msg::Pose p; p.position.x=x; p.position.y=y; p.position.z=z;
  p.orientation.x=qx; p.orientation.y=qy; p.orientation.z=qz; p.orientation.w=qw;
  return p;
}

void JobRunner::run(const arlab_common_interfaces::msg::OrchestratorData& msg) {
  const std::string cmd = msg.cmd.data;
  RCLCPP_INFO(logger_, "JobRunner received cmd='%s'", cmd.c_str());

  if (cmd == "open") {
    hand_.open();
    return;
  }
  if (cmd == "close") {
    hand_.close();
    return;
  }
  if (cmd == "point") {
    hand_.point();
    return;
  }
  if (cmd == "home") {
    (void)arm_.moveToHome();
    return;
  }
  if (cmd == "move") {
    // Beispielziel; oder nimm msg.pose direkt:
    (void)arm_.moveToPose(msg.pose);
    return;
  }
  if (cmd == "pick") {
    // einfache Sequenz
    hand_.open();
    (void)arm_.moveToHome();
    (void)arm_.moveToPose(createPose(0.372,0.124,0.3,0.999,0.041,0.006,0.004));
    (void)arm_.moveToPose(msg.pose);
    hand_.close();
    (void)arm_.moveToHome();
    return;
  }

  RCLCPP_WARN(logger_, "Unknown command: %s", cmd.c_str());
}
