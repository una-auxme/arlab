// -----------------------------------------------------------------------------
// File: hand_force_test.cpp
// Package: arlab_manipulation_cpp
// Maintainer: Marc Stumpp <marc.stumpp@uni-a.de>
//
// Tests the output of the hand force sensors. 
// -----------------------------------------------------------------------------

#include <chrono>
#include <memory>
#include <string>
#include <functional>
#include "rclcpp/rclcpp.hpp"

// #include "rclcpp/rclcpp.hpp" // standart ros2 geplänker
// aus modules/mia_hand_ros2_pkgs/mia_hand_msgs/msg/ForceData.msg
// wird zu force_data.hpp
#include "arlab_manipulation_cpp/hand_force_switch.hpp"

// #include "std_srvs/srv/set_bool.hpp"
 
HandForceSwitch::HandForceSwitch(const rclcpp::Node::SharedPtr& node)
    : switch_client_(node->create_client<SetBool>(
          "mia_hand/data_streams/fingers/forces/switch")),
      logger_(node->get_logger()) {}
 
void HandForceSwitch::EnableStream(std::chrono::milliseconds timeout) {
  SetStream(true, timeout);
}
 
void HandForceSwitch::DisableStream(std::chrono::milliseconds timeout) {
  SetStream(false, timeout);
}
 
void HandForceSwitch::SetStream(bool enable, std::chrono::milliseconds timeout) {
  if (!switch_client_->wait_for_service(timeout)) {
    RCLCPP_ERROR(logger_,
                 "Force stream switch service not available - is the Mia Hand "
                 "driver running?");
    return;
  }
 
  // Build the request. SetBool has exactly one field: data.
  auto request = std::make_shared<SetBool::Request>();
  request->data = enable;  // true = stream on, false = stream off
 
  // Send asynchronously (non-blocking). The response is handled later in
  // the lambda, once the surrounding executor spins. Blocking here could
  // deadlock if this method is ever called from within a callback.
  switch_client_->async_send_request(
      request,
      [this, enable](rclcpp::Client<SetBool>::SharedFuture future) {
        auto response = future.get();
        RCLCPP_INFO(logger_, "Force stream switch (%s): success=%d msg='%s'",
                    enable ? "on" : "off", response->success,
                    response->message.c_str());
      });
}
