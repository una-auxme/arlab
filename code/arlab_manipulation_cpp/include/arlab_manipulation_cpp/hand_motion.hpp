// -----------------------------------------------------------------------------
// File: hand_motion.hpp
// Package: arlab_manipulation_cpp
// Maintainer: Leonie Schmidt <leonie1.schmidt@uni-a.de>
//
// Declares HandMotion, a high-level wrapper around the Mia Hand grasp action
// interface. It abstracts the open/close/grasp lifecycle into simple method
// calls for higher-level components (e.g. JobRunner)
// -----------------------------------------------------------------------------

#ifndef ARLAB_MANIPULATION_CPP_HAND_MOTION_HPP_
#define ARLAB_MANIPULATION_CPP_HAND_MOTION_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "mia_hand_msgs/action/grasp.hpp"

/**
 * High-level wrapper around the Mia Hand grasp action client.
 * Provides simple methods for opening, closing and grasping with the hand.
 */
class HandMotion {
  public:
    using GraspAction = mia_hand_msgs::action::Grasp;
    using Client = rclcpp_action::Client<GraspAction>;
    using GoalHandle = rclcpp_action::ClientGoalHandle<GraspAction>;

    /**
     * Creates a HandMotion and initialises the internal grasp action client.
     * @param node    Shared pointer to the owning ROS 2 node. Must not be null.
     * @throws ManipulationException if node is null.
     */
    explicit HandMotion(rclcpp::Node::SharedPtr node);

    /**
     * Opens the hand to a predefined low-closure configuration.
     * @param timeout   Maximum time to wait for goal acceptance.
                        Defaults to 3000 ms.
     * @throws ManipulationException on any action-level failure.
     */
    void Open(
      std::chrono::milliseconds timeout = std::chrono::milliseconds{3000});

    /**
     * Closes the hand to a predefined grasp configuration.
     * @param timeout   Maximum time to wait for goal acceptance.
                        Defaults to 3000 ms.
     * @throws ManipulationException on any action-level failure.
     */
    void Close(
      std::chrono::milliseconds timeout = std::chrono::milliseconds{3000});

    /**
     * Sends a grasp goal to the hand action server and waits for a result.
     * @param target_closure_percent  Desired hand closure in percent [0, 100].
     * @param speed_for_percent       Closing speed in percent. Defaults to 15.
     * @param timeout                 Per-step wait duration for goal acceptance
     *                                and result retrieval. Defaults to 3000 ms.
     * @param server_wait             Time to wait for the action server to become
     *                                available. Defaults to 2000 ms.
     * @throws ManipulationException if the server is not ready, the goal is
     *         rejected, or goal acceptance times out.
     */
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
