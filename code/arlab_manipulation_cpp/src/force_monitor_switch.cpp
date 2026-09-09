// -----------------------------------------------------------------------------
// File: force_monitor_switch.cpp
// Package: arlab_manipulation_cpp
// Maintainer: Marc Stumpp <marc.stumpp@uni-a.de>
//
// Implements the ForceMonitorSwitch class. ActivateMonitor and
// DeactivateMonitor both call SetMonitor, which waits for the service,
// builds the request and sends it asynchronously.
// The response is only logged, so a monitor that is missing or fails to
// activate does not interrupt the running motion sequence.
// -----------------------------------------------------------------------------

#include "arlab_manipulation_cpp/force_monitor_switch.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <functional>

#include "rclcpp/rclcpp.hpp"

ForceMonitorSwitch::ForceMonitorSwitch(const rclcpp::Node::SharedPtr &node, const std::string &service_name)
    : switch_client_(node->create_client<ActivateForceMonitor>(service_name)),
      logger_(node->get_logger()) {}

void ForceMonitorSwitch::ActivateMonitor(const std::string &grip_type, std::chrono::milliseconds timeout)
{
  SetMonitor(true, grip_type, timeout);
}

void ForceMonitorSwitch::DeactivateMonitor(std::chrono::milliseconds timeout)
{
  SetMonitor(false, "", timeout);
}

void ForceMonitorSwitch::SetMonitor(bool activate, const std::string &grip_type, std::chrono::milliseconds timeout)
{
  // The timeout only covers waiting for the service to appear.
  // The request itself is sent asynchronously and its response is not awaited.
  if (!switch_client_->wait_for_service(timeout))
  {
    RCLCPP_ERROR(logger_, "Force Monitor switch service not available");
    return;
  }

  // Build the request. When deactivating, grip_type stays empty and the monitor ignores it.
  auto request = std::make_shared<ActivateForceMonitor::Request>();
  request->activate = activate;
  request->grip_type = grip_type;

  switch_client_->async_send_request(
      request,
      [this, activate](rclcpp::Client<ActivateForceMonitor>::SharedFuture future)
      {
        try
        {
          auto response = future.get();
          RCLCPP_INFO(logger_, "Force Monitor (%s): success=%d msg='%s'",
                      activate ? "activate" : "deactivate", response->success,
                      response->message.c_str());
        }
        catch (const std::exception &e)
        {
          RCLCPP_WARN(logger_, "Force Monitor response not received: %s", e.what());
        }
      });
}
