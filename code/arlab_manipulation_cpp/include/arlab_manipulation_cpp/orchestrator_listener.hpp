#pragma once
#include <rclcpp/rclcpp.hpp>
#include "arlab_common_interfaces/msg/orchestrator_data.hpp"

class JobRunner;

class OrchestratorListener : public rclcpp::Node {
public:
  explicit OrchestratorListener(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  void init();
  
private:

  void onMsg(const arlab_common_interfaces::msg::OrchestratorData::SharedPtr msg);

  rclcpp::Subscription<arlab_common_interfaces::msg::OrchestratorData>::SharedPtr sub_;

  // Owned capabilities
  std::unique_ptr<class ArmMotion> arm_;
  std::unique_ptr<class HandMotion> hand_;
  std::unique_ptr<JobRunner> runner_;
};
