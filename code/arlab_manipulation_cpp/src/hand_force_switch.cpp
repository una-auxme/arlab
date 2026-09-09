// -----------------------------------------------------------------------------
// File: hand_force_switch.cpp
// Package: arlab_manipulation_cpp
// Maintainer: Marc Stumpp <marc.stumpp@uni-a.de>
//
// Implements the HandForceSwitch class. EnableStream and DisableStream
// both call SetStream, which waits for the service, builds the request
// and sends it asynchronously. The response is only logged, so a service
// that is missing or fails to enable the stream does not interrupt the
// running motion sequence.
// -----------------------------------------------------------------------------

#include "arlab_manipulation_cpp/hand_force_switch.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <functional>

#include "rclcpp/rclcpp.hpp"

HandForceSwitch::HandForceSwitch(const rclcpp::Node::SharedPtr &node, const std::string &service_name)
    : switch_client_(node->create_client<SetBool>(service_name)),
      logger_(node->get_logger()) {}

void HandForceSwitch::EnableStream(std::chrono::milliseconds timeout)
{
  SetStream(true, timeout);
}

void HandForceSwitch::DisableStream(std::chrono::milliseconds timeout)
{
  SetStream(false, timeout);
}

void HandForceSwitch::SetStream(bool enable, std::chrono::milliseconds timeout)
{
  // The timeout only covers waiting for the service to appear.
  // The request itself is sent asynchronously and its response is not awaited.
  if (!switch_client_->wait_for_service(timeout))
  {
    RCLCPP_ERROR(logger_,
                 "Force stream switch service not available - "
                 "is the Mia Hand driver running?");
    return;
  }

  // Build the request.
  auto request = std::make_shared<SetBool::Request>();
  request->data = enable;

  // The response is not awaited, the request is only fired off.
  // Waiting here would delay the motion sequence for a value nobody reads.
  switch_client_->async_send_request(
      request,
      [this, enable](rclcpp::Client<SetBool>::SharedFuture future)
      {
        try
        {
          auto response = future.get();
          RCLCPP_INFO(logger_, "Force stream switch (%s): success=%d msg='%s'",
                      enable ? "on" : "off", response->success,
                      response->message.c_str());
        }
        catch (const std::exception &e)
        {
          RCLCPP_WARN(logger_, "Force stream switch response not received: %s", e.what());
        }
      });
}
