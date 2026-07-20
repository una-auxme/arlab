// -----------------------------------------------------------------------------
// File: force_monitor_switch.hpp
// Package: arlab_manipulation_cpp
// Maintainer: Marc Stumpp <marc.stumpp@uni-a.de>
//
// Activates the force monitor, a fuction that measures force drops to detect dropped objects.
// It gives the opportunity to enable the force monitor, 
// by using the custom ActivateForceMonitor service. It hands the grip type over 
// aswell to enable the force monitor to just measure the relevant joints for the grip.
// -----------------------------------------------------------------------------

#ifndef ARLAB_MANIPULATION_CPP_FORCE_MONITOR_SWITCH_HPP_
#define ARLAB_MANIPULATION_CPP_FORCE_MONITOR_SWITCH_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "arlab_common_interfaces/srv/activate_force_monitor.hpp"

class ForceMonitorSwitch {
  public:
    using ActivateForceMonitor = arlab_common_interfaces::srv::ActivateForceMonitor;

    /**
     * Creates a service_client to activate the force monitor.
     * and to hand over the grip type.
     * @param service_name Existing node the Client is created on.
     */
    ForceMonitorSwitch(const rclcpp::Node::SharedPtr& node,
                     const std::string& service_name);
    /**
     * Activates force monitor with the given grip type.
     * @param grip_type Grip in use.
     * @param timeout Maximum time to wait for the service to appear.
                      Defaults to 3000 ms.
     */
    void ActivateMonitor(
        const std::string& grip_type,
        std::chrono::milliseconds timeout = std::chrono::milliseconds{3000});
    
    /**
     * Deactivates the force monitor.
     * @param timeout Maximum time to wait for the service to appear.
                      Defaults to 3000 ms.
     */
    void DeactivateMonitor(
      std::chrono::milliseconds timeout = std::chrono::milliseconds{3000});


  private:
    /**
    * Shared implementation for Activate/Deactivate: waits for the service,
    * then sends the request asynchronously and logs the response.
    * @param activate  true = monitor on, false = monitor off.
    * @param grip_type Grip in use (irrelevant when deactivating).
    * @param timeout Maximum time to wait for the service to appear.
    */
  void SetMonitor(bool activate, const std::string& grip_type, std::chrono::milliseconds timeout);

  rclcpp::Client<ActivateForceMonitor>::SharedPtr switch_client_;
  rclcpp::Logger logger_;
};

#endif  // ARLAB_MANIPULATION_CPP_FORCE_MONITOR_SWITCH_HPP_