// -----------------------------------------------------------------------------
// File: hand_force_switch.hpp
// Package: arlab_manipulation_cpp
// Maintainer: Marc Stumpp <marc.stumpp@uni-a.de>
//
// Declares HandForceSwitch, a service client for the Mia Hand force data
// stream. It enables and disables the stream through the SetBool service of
// the hand driver, so that the finger forces are only published while they
// are needed. JobRunner uses it to start the stream before a grasp and to
// stop it afterwards.
// -----------------------------------------------------------------------------

#ifndef ARLAB_MANIPULATION_CPP_HAND_FORCE_SWITCH_HPP_
#define ARLAB_MANIPULATION_CPP_HAND_FORCE_SWITCH_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

/**
 * Service client that enables and disables the Mia Hand force data stream.
 */
class HandForceSwitch
{
public:
  using SetBool = std_srvs::srv::SetBool;

  /**
   * Creates a switch client to enable the force data stream.
   * @param node            Existing node the client is created on.
   * @param service_name    Name of the SetBool service of the hand driver.
   */
  HandForceSwitch(const rclcpp::Node::SharedPtr &node, const std::string &service_name);

  /**
   * Enables the hand force stream.
   * @param timeout   Maximum time to wait for the service to appear.
   *                  Defaults to 3000 ms.
   */
  void EnableStream(
      std::chrono::milliseconds timeout = std::chrono::milliseconds{3000});

  /**
   * Disables the hand force stream.
   * @param timeout   Maximum time to wait for the service to appear.
   *                  Defaults to 3000 ms.
   */
  void DisableStream(
      std::chrono::milliseconds timeout = std::chrono::milliseconds{3000});

private:
  /**
   * Shared implementation for Enable/Disable: waits for the service,
   * then sends the request asynchronously and logs the response.
   * @param enable    true = stream on, false = stream off.
   * @param timeout   Maximum time to wait for the service to appear.
   */
  void SetStream(bool enable, std::chrono::milliseconds timeout);

  rclcpp::Client<SetBool>::SharedPtr switch_client_;
  rclcpp::Logger logger_;
};

#endif  // ARLAB_MANIPULATION_CPP_HAND_FORCE_SWITCH_HPP_