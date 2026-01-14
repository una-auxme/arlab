#include "arlab_manipulation_cpp/hand_motion.hpp"

#include <stdexcept>

HandMotion::HandMotion(rclcpp::Node::SharedPtr node)
: node_(std::move(node))
{
  if (!node_) {
    throw std::invalid_argument("HandMotion: node ist null");
  }
  client_ = rclcpp_action::create_client<Grasp>(node_, "/mia_hand/grasps/cylindrical/action");
}

void HandMotion::open(std::chrono::milliseconds timeout)
{
  grasp(0, 15, timeout);
}

void HandMotion::close(std::chrono::milliseconds timeout)
{
  grasp(100, 15, timeout);
}

void HandMotion::grasp(
  int target_closure_percent,
  int speed_for_percent,
  std::chrono::milliseconds timeout,
  std::chrono::milliseconds server_wait)
{
  // optional
  if (target_closure_percent < 0) target_closure_percent = 0;
  if (target_closure_percent > 100) target_closure_percent = 100;

  // 1) Action-Server?
  if (!client_->wait_for_action_server(server_wait)) {
    //throw std::runtime_error("Grasp Action-Server not ready: " + "/mia_hand/grasps/cylindrical/action");
  }

  // 2) Goal
  Grasp::Goal goal;
  goal.spe_for_percent = speed_for_percent;
  goal.target_closure_percent = target_closure_percent;

  // 3) Goal send
  auto goal_future = client_->async_send_goal(goal);
  auto rc = rclcpp::spin_until_future_complete(node_, goal_future, timeout);
  if (rc != rclcpp::FutureReturnCode::SUCCESS) {
    //throw std::runtime_error("Timeout/Error while sending the Grasp-Goal");
  }

  auto goal_handle = goal_future.get();
  if (!goal_handle) {
    //throw std::runtime_error("Grasp-Goal wurde abgelehnt (goal_handle null)");
  }

  // 4) Result
  auto result_future = client_->async_get_result(goal_handle);
  rc = rclcpp::spin_until_future_complete(node_, result_future, timeout);
  if (rc != rclcpp::FutureReturnCode::SUCCESS) {
    // optional cancel
    try {
      auto cancel_future = client_->async_cancel_goal(goal_handle);
      (void)rclcpp::spin_until_future_complete(node_, cancel_future, std::chrono::milliseconds{1000});
    } catch (...) {
      // ignore
    }
    //throw std::runtime_error("Timeout/Error while waiting for Grasp-Result");
  }

  auto wrapped = result_future.get();
  if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED) {
    //throw std::runtime_error("Grasp Action not successful");
  }

  const auto & result = wrapped.result;
  if (result && !result->err_message.empty()) {
    //throw std::runtime_error("Hand-Grasp error: " + result->err_message);
  }
}
