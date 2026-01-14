#pragma once

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "mia_hand_msgs/action/grasp.hpp"

class HandMotion{
public:
  using Grasp = mia_hand_msgs::action::Grasp;
  using Client = rclcpp_action::Client<Grasp>;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Grasp>;

  HandMotion(rclcpp::Node::SharedPtr node);

  void open(std::chrono::milliseconds timeout = std::chrono::milliseconds{3000});
  void close(std::chrono::milliseconds timeout = std::chrono::milliseconds{3000});

  void grasp(
    int target_closure_percent,
    int speed_for_percent = 15,
    std::chrono::milliseconds timeout = std::chrono::milliseconds{3000},
    std::chrono::milliseconds server_wait = std::chrono::milliseconds{2000});

private:
  rclcpp::Node::SharedPtr node_;
  Client::SharedPtr client_;
};
