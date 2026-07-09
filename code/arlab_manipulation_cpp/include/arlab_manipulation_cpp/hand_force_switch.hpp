// -----------------------------------------------------------------------------
// File: hand_force.hpp
// Package: arlab_manipulation_cpp
// Maintainer: Marc Stumpp <marc.stumpp@uni-a.de>
//
// Declares HandForce, a high-level wrapper around the Mia Hand's finger force
// sensors... It gives the opportunity to enable the force data streaam from the 
// mia hand, by using the hand driver's SetBool switch Service. It enables thereby
// the latest normal and tangential forces (thumb / index / mrl) to higher-level 
// components (e.g. safety) without exposing the underlying ROS plumbing.
// -----------------------------------------------------------------------------

// The relevant topic name is: mia_hand/data_streams/fingers/forces/data
// Further information can be found in:
// /workspace/src/modules/mia_hand_ros2_pkgs/mia_hand_driver/README.md

#ifndef ARLAB_MANIPULATION_CPP_HAND_FORCE_SWITCH_HPP_
#define ARLAB_MANIPULATION_CPP_HAND_FORCE_SWITCH_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <functional>

#include "rclcpp/rclcpp.hpp"
// #include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/set_bool.hpp"

/**
 * High-level wrapper around the Mia Hand's finger force sensors
 */
class HandForceSwitch {
  public:
    using SetBool = std_srvs::srv::SetBool;

    /**
     * Creates a switch_client to enable the force data stream
     * @param node Existing node the Client is created on
     */
    explicit HandForceSwitch(const rclcpp::Node::SharedPtr& node);

    /**
     * Enables the hand force stream.
     * @param timeout Maximum time to wait for goal acceptance.
                      Defaults to 3000 ms.
     */
    void EnableStream(
      std::chrono::milliseconds timeout = std::chrono::milliseconds{3000});
    
    /**
     * Disables the hand force stream.
     * @param timeout Maximum time to wait for goal acceptance.
                      Defaults to 3000 ms.
     */
    void DisableStream(
      std::chrono::milliseconds timeout = std::chrono::milliseconds{3000});


  private:
    /**
    * Shared implementation for Enable/Disable: waits for the service,
    * then sends the request asynchronously and logs the response.
    * @param enable  true = stream on, false = stream off.
    * @param timeout Maximum time to wait for the service to appear.
    */
  void SetStream(bool enable, std::chrono::milliseconds timeout);

  rclcpp::Client<SetBool>::SharedPtr switch_client_;
  rclcpp::Logger logger_;
};


#endif  // ARLAB_MANIPULATION_CPP_HAND_FORCE_SWITCH_HPP_