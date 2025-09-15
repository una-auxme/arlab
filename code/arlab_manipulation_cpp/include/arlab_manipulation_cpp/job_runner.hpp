#pragma once

#include <rclcpp/rclcpp.hpp>

#include "arlab_common_interfaces/msg/orchestrator_data.hpp"

/**
 * @brief Run a job based on an OrchestratorData message.
 *
 * Supported commands are: "pick", "place", "open", "close", "move", "home".
 *
 * @param msg OrchestratorData message containing job command information.
 * @param node Shared pointer to the ROS 2 node.
 * @return int Exit code
 */
int run_job(const arlab_common_interfaces::msg::OrchestratorData &msg,std::shared_ptr<rclcpp::Node> node);

