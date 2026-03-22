// -----------------------------------------------------------------------------
// File: hand_motion.cpp
// Package: arlab_manipulation_cpp
// Maintainer: Leonie Schmidt <leonie1.schmidt@uni-a.de>
//
// Implements the HandMotion class. Manages the lifecycle of a grasp action
// goal: waiting for the action server, sending the goal, waiting for
// acceptance, and finally waiting for the result.
// -----------------------------------------------------------------------------

#include "arlab_manipulation_cpp/hand_motion.hpp"

#include <utility>

#include "arlab_manipulation_cpp/manipulator_exception.hpp"

namespace {

// --- Solution: Zirbi Manipulation and Robot Model ---
// Change Mia Hand action to pinch grasp action
constexpr char kGraspActionName[] = "/mia_hand/grasps/pinch/action";
constexpr int kOpenClosurePercent = 10;
constexpr int kClosedClosurePercent = 80;
constexpr int kDefaultSpeedPercent = 15;
constexpr int kMinClosurePercent = 0;
constexpr int kMaxClosurePercent = 100;

}  // namespace

HandMotion::HandMotion(rclcpp::Node::SharedPtr node)
    : node_(std::move(node)) {
  if (!node_) {
    throw ManipulationException(
      arlab_common_interfaces::msg::ManipulationResponse::
      ORCHESTRATOR_LISTENER_NODE_NULL);
  }

  client_ = rclcpp_action::create_client<GraspAction>(node_, kGraspActionName);
}

void HandMotion::Open(std::chrono::milliseconds timeout) {
  Grasp(kOpenClosurePercent, kDefaultSpeedPercent, timeout);
}

void HandMotion::Close(std::chrono::milliseconds timeout) {
  Grasp(kClosedClosurePercent, kDefaultSpeedPercent, timeout);
}

void HandMotion::Grasp(int target_closure_percent, int speed_for_percent,
                        std::chrono::milliseconds timeout,
                        std::chrono::milliseconds server_wait) {

  // Clamp the closure value to the valid hardware range [0, 100].
  if (target_closure_percent < kMinClosurePercent) {
    target_closure_percent = kMinClosurePercent;
  }
  if (target_closure_percent > kMaxClosurePercent) {
    target_closure_percent = kMaxClosurePercent;
  }

  if (!client_->wait_for_action_server(server_wait)) {
    throw ManipulationException(
      arlab_common_interfaces::msg::ManipulationResponse::
      GRASP_ACTION_SERVER_NOT_READY);
  }

  GraspAction::Goal goal;
  goal.spe_for_percent = speed_for_percent;
  goal.target_closure_percent = target_closure_percent;

  auto goal_future = client_->async_send_goal(goal);

  if (goal_future.wait_for(timeout) != std::future_status::ready) {
    throw ManipulationException(
      arlab_common_interfaces::msg::ManipulationResponse::
      GRASP_GOAL_TIME_OUT);
  }

  auto goal_handle = goal_future.get();
  if (!goal_handle) {
    throw ManipulationException(
      arlab_common_interfaces::msg::ManipulationResponse::
      INVALID_GRASP_GOAL);
  }

  // The result timeout is currently not enforced because the Mia Hand
  // may not close entirely when an object is present in the gripper.
  // Therefore we wait indefinitely for the result.
  // TODO: Find out over sensors if an object is present.
  // Then the hand can be "closed enough" to consider the grasp successful and
  // we can enforce a timeout here.
  auto result_future = client_->async_get_result(goal_handle);
  if (result_future.wait_for(timeout) != std::future_status::ready) {
    // throw ManipulationException(
    //   arlab_common_interfaces::msg::ManipulationResponse::
    //   GRASP_RESULT_TIME_OUT);
  }
}
