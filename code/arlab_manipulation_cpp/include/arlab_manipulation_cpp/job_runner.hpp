#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "arlab_common_interfaces/msg/orchestrator_data.hpp"

class ArmMotion;
class HandMotion;

class JobRunner {
public:
  JobRunner(rclcpp::Node& node, ArmMotion& arm, HandMotion& hand);

  void run(const arlab_common_interfaces::msg::OrchestratorData& msg);

private:
  rclcpp::Logger logger_;
  ArmMotion& arm_;
  HandMotion& hand_;

  geometry_msgs::msg::Pose createPose(double x,double y,double z,double qx,double qy,double qz,double qw) const;

  std::map<std::string, double> createJointPos(
    double shoulder_pan_joint,double shoulder_lift_joint,double elbow_joint,
    double wrist_1_joint,double wrist_2_joint,double wrist_3_joint);

};

