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

//constexpr char kGraspActionName[] = "/mia_hand/grasps/cylindrical/action";
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

  // client_ = rclcpp_action::create_client<GraspAction>(node_, kGraspActionName);  
}

HandMotion::Client::SharedPtr
HandMotion::GetCreateClient(const std::string& action_name) {
  if (client_cache_.find(action_name) == client_cache_.end()){
    // create a new client for specific grasp action if not present already
    client_cache_[action_name] = rclcpp_action::create_client<GraspAction>(node_, action_name); 
  }
  return client_cache_[action_name];
} 

void HandMotion::Open(std::chrono::milliseconds timeout) {
  Grasp("/mia_hand/grasps/cylindrical/action", kOpenClosurePercent, kDefaultSpeedPercent, timeout);
}

void HandMotion::Close(std::chrono::milliseconds timeout) {
  Grasp("/mia_hand/grasps/cylindrical/action", kClosedClosurePercent, kDefaultSpeedPercent, timeout);
}

void HandMotion::Pinch(std::chrono::milliseconds timeout) {
  Grasp("/mia_hand/grasps/pinch/action", kClosedClosurePercent, kDefaultSpeedPercent, timeout);
}

void HandMotion::Lateral(std::chrono::milliseconds timeout) {
  Grasp("/mia_hand/grasps/lateral/action", kClosedClosurePercent, kDefaultSpeedPercent, timeout);
}

void HandMotion::PointUp(std::chrono::milliseconds timeout) {
  Grasp("/mia_hand/grasps/point_up/action", 0, kDefaultSpeedPercent, timeout);
}

void HandMotion::PointDown(std::chrono::milliseconds timeout) {
  Grasp("/mia_hand/grasps/point_down/action", 100, kDefaultSpeedPercent, timeout);
}
void HandMotion::Spherical(std::chrono::milliseconds timeout) {
  Grasp("/mia_hand/grasps/spherical/action", kClosedClosurePercent, kDefaultSpeedPercent, timeout);
}

void HandMotion::Tridigital(std::chrono::milliseconds timeout) {
  Grasp("/mia_hand/grasps/tridigital/action", kClosedClosurePercent, kDefaultSpeedPercent, timeout);
}

void HandMotion::Grasp(const std::string& action_name,
                        int target_closure_percent, int speed_for_percent,
                        std::chrono::milliseconds timeout,
                        std::chrono::milliseconds server_wait) {

  // Clamp the closure value to the valid hardware range [0, 100].
  if (target_closure_percent < kMinClosurePercent) {
    target_closure_percent = kMinClosurePercent;
  }
  if (target_closure_percent > kMaxClosurePercent) {
    target_closure_percent = kMaxClosurePercent;
  }

  // DEBUG
  RCLCPP_INFO(
    node_->get_logger(),
    "Waiting for action server '%s'",
    action_name.c_str());

  auto client = GetCreateClient(action_name);

  // more DEBUG
  bool ready = client->wait_for_action_server(server_wait);
  RCLCPP_INFO(
    node_->get_logger(),
    "Server ready: %s",
    ready ? "true" : "false");

  if (!client->wait_for_action_server(server_wait)) {
    throw ManipulationException(
      arlab_common_interfaces::msg::ManipulationResponse::
      GRASP_ACTION_SERVER_NOT_READY);
  }

  GraspAction::Goal goal;
  goal.spe_for_percent = speed_for_percent;
  goal.target_closure_percent = target_closure_percent;

  auto goal_future = client->async_send_goal(goal);

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
  auto result_future = client->async_get_result(goal_handle);
  if (result_future.wait_for(timeout) != std::future_status::ready) {
    // throw ManipulationException(
    //   arlab_common_interfaces::msg::ManipulationResponse::
    //   GRASP_RESULT_TIME_OUT);
  }
}
