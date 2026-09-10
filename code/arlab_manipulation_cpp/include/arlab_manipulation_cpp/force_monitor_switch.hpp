// -----------------------------------------------------------------------------
// File: force_monitor_switch.hpp
// Package: arlab_manipulation_cpp
// Maintainer: Marc Stumpp <marc.stumpp@uni-a.de>
//
// Declares ForceMonitorSwitch, a service client for the force monitor node.
// It arms and disarms the monitor through the ActivateForceMonitor 
// service and passes the grip type when activating, so that only the sensors 
// relevant for the current grip are evaluated. JobRunner uses it to arm the 
// monitor after closing the hand and to disarm it before opening it again.
// -----------------------------------------------------------------------------

#ifndef ARLAB_MANIPULATION_CPP_FORCE_MONITOR_SWITCH_HPP_
#define ARLAB_MANIPULATION_CPP_FORCE_MONITOR_SWITCH_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "arlab_common_interfaces/srv/activate_force_monitor.hpp"

/**
 * Service client that arms and disarms the force monitor and passes the
 * grip type currently in use.
 */
class ForceMonitorSwitch
{
public:
  using ActivateForceMonitor = arlab_common_interfaces::srv::ActivateForceMonitor;

  /**
   * Creates a service client to activate the force monitor
   * and to hand over the grip type.
   * @param node            Existing node the client is created on.
   * @param service_name    Name of the ActivateForceMonitor service.
   */
  ForceMonitorSwitch(
      const rclcpp::Node::SharedPtr &node,
      const std::string &service_name);

  /**
   * Activates the force monitor with the given grip type.
   * @param grip_type   Grip in use.
   * @param timeout     Maximum time to wait for the service to appear.
   *                    Defaults to 3000 ms.
   */
  void ActivateMonitor(
      const std::string &grip_type,
      std::chrono::milliseconds timeout = std::chrono::milliseconds{3000});

  /**
   * Deactivates the force monitor.
   * @param timeout   Maximum time to wait for the service to appear.
   *                  Defaults to 3000 ms.
   */
  void DeactivateMonitor(
      std::chrono::milliseconds timeout = std::chrono::milliseconds{3000});

private:
  /**
   * Shared implementation for Activate/Deactivate: waits for the service,
   * then sends the request asynchronously and logs the response.
   * @param activate    true = monitor on, false = monitor off.
   * @param grip_type   Grip in use (irrelevant when deactivating).
   * @param timeout     Maximum time to wait for the service to appear.
   */
  void SetMonitor(
      bool activate,
      const std::string &grip_type,
      std::chrono::milliseconds timeout);

  rclcpp::Client<ActivateForceMonitor>::SharedPtr switch_client_;
  rclcpp::Logger logger_;
};

#endif  // ARLAB_MANIPULATION_CPP_FORCE_MONITOR_SWITCH_HPP_
