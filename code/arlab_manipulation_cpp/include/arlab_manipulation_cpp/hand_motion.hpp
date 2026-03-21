#ifndef ARLAB_MANIPULATION_CPP_HAND_MOTION_HPP_
#define ARLAB_MANIPULATION_CPP_HAND_MOTION_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "mia_hand_msgs/action/grasp.hpp"

// Provides high-level helper methods for controlling the robotic hand through
// the grasp action interface.
class HandMotion {
  public:
    using GraspAction = mia_hand_msgs::action::Grasp;
    using Client = rclcpp_action::Client<GraspAction>;
    using GoalHandle = rclcpp_action::ClientGoalHandle<GraspAction>;

    // Creates a hand motion helper for the given ROS 2 node.
    explicit HandMotion(rclcpp::Node::SharedPtr node);

    // Opens the hand to a predefined low-closure configuration.
    void Open(
      std::chrono::milliseconds timeout = std::chrono::milliseconds{3000});

    // Closes the hand to a predefined grasp configuration.
    void Close(
      std::chrono::milliseconds timeout = std::chrono::milliseconds{3000});

    // Sends a grasp goal to the hand action server.
    //
    // target_closure_percent specifies the desired closure in percent and is
    // clamped to the range [0, 100]. speed_for_percent specifies the closing
    // speed in percent.
    void Grasp(
      int target_closure_percent,
      int speed_for_percent = 15,
      std::chrono::milliseconds timeout = std::chrono::milliseconds{3000},
      std::chrono::milliseconds server_wait =
          std::chrono::milliseconds{2000});

  private:
    rclcpp::Node::SharedPtr node_;
    Client::SharedPtr client_;
};

#endif  // ARLAB_MANIPULATION_CPP_HAND_MOTION_HPP_
