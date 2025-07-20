#pragma once

#include "arlab_common_interfaces/msg/orchestrator_data.hpp"
#include <rclcpp/rclcpp.hpp>

int run_job(const arlab_common_interfaces::msg::OrchestratorData &msg,std::shared_ptr<rclcpp::Node> node);

