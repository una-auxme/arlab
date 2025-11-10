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
};

