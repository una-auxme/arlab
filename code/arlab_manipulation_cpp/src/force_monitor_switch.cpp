// -----------------------------------------------------------------------------
// File: force_monitor_switch.cpp
// Package: arlab_manipulation_cpp
// Maintainer: Marc Stumpp <marc.stumpp@uni-a.de>
//
// Activates the force monitor with the used grip type
// -----------------------------------------------------------------------------

#include <chrono>
#include <memory>
#include <string>
#include <functional>
#include "rclcpp/rclcpp.hpp"

#include "arlab_manipulation_cpp/force_monitor_switch.hpp"

ForceMonitorSwitch::ForceMonitorSwitch(const rclcpp::Node::SharedPtr& node, const std::string& service_name)
    : switch_client_(node->create_client<ActivateForceMonitor>(service_name)),
      logger_(node->get_logger()) {}

void ForceMonitorSwitch::ActivateMonitor(const std::string& grip_type, std::chrono::milliseconds timeout) {
  SetMonitor(true, grip_type, timeout);
}
 
void ForceMonitorSwitch::DeactivateMonitor(std::chrono::milliseconds timeout) {
  SetMonitor(false, "", timeout);
}
 
void ForceMonitorSwitch::SetMonitor(bool activate, const std::string& grip_type, std::chrono::milliseconds timeout) {
  if (!switch_client_->wait_for_service(timeout)) {
    RCLCPP_ERROR(logger_,
                 "Force_Monitor switch service not available");
    return;
  }
 
  // Build the request. ActivateForceMonitor wants a bool value and a string
  auto request = std::make_shared<ActivateForceMonitor::Request>();
  request->activate = activate;  
  request->grip_type = grip_type;

  switch_client_->async_send_request(
      request,
      [this, activate](rclcpp::Client<ActivateForceMonitor>::SharedFuture future) {
        auto response = future.get();
        RCLCPP_INFO(logger_, "Force Monitor (%s): success=%d msg='%s'",
                    activate ? "activate" : "disable", response->success,
                    response->message.c_str());
      });
}