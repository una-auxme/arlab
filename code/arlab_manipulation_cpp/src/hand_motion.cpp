#include "arlab_manipulation_cpp/hand_motion.hpp"
#include "arlab_manipulation_cpp/manipulator_exception.hpp"

#include <stdexcept>

HandMotion::HandMotion(rclcpp::Node::SharedPtr node)
    : node_(std::move(node))
{
  if (!node_)
  {
    throw ManipulationException(arlab_common_interfaces::msg::ManipulationResponse::ORCHESTRATOR_LISTENER_NODE_NULL);
  }
  client_ = rclcpp_action::create_client<Grasp>(node_, "/mia_hand/grasps/cylindrical/action");
}

void HandMotion::open(std::chrono::milliseconds timeout)
{
  grasp(10, 15, timeout);
}

void HandMotion::close(std::chrono::milliseconds timeout)
{
  grasp(80, 15, timeout);
}

void HandMotion::grasp(
    int target_closure_percent,
    int speed_for_percent,
    std::chrono::milliseconds timeout,
    std::chrono::milliseconds server_wait)
{
  // optional
  if (target_closure_percent < 0)
    target_closure_percent = 0;
  if (target_closure_percent > 100)
    target_closure_percent = 100;

  // 1) Action-Server?
  if (!client_->wait_for_action_server(server_wait))
  {
    throw ManipulationException(arlab_common_interfaces::msg::ManipulationResponse::GRASP_ACTION_SERVER_NOT_READY);
  }

  // 2) Goal
  Grasp::Goal goal;
  goal.spe_for_percent = speed_for_percent;
  goal.target_closure_percent = target_closure_percent;

  // 3) Goal send
  auto goal_future = client_->async_send_goal(goal);
  // auto rc = rclcpp::spin_until_future_complete(node_, goal_future, timeout);
  //  if (rc != rclcpp::FutureReturnCode::SUCCESS) {
  //    throw ManipulationException(arlab_common_interfaces::msg::ManipulationResponse::GRASP_GOAL_TIME_OUT);
  //  }

  if (goal_future.wait_for(timeout) != std::future_status::ready)
  {
    throw ManipulationException(arlab_common_interfaces::msg::ManipulationResponse::GRASP_GOAL_TIME_OUT);
  }

  auto goal_handle = goal_future.get();
  if (!goal_handle)
  {
    throw ManipulationException(arlab_common_interfaces::msg::ManipulationResponse::INVALID_GRASP_GOAL);
  }

  // 4) Result
  auto result_future = client_->async_get_result(goal_handle);

  // rc = rclcpp::spin_until_future_complete(node_, result_future, timeout);
  // if (rc != rclcpp::FutureReturnCode::SUCCESS) {
  //   // optional cancel
  //   // try {
  //   //   auto cancel_future = client_->async_cancel_goal(goal_handle);
  //   //   (void)rclcpp::spin_until_future_complete(node_, cancel_future, std::chrono::milliseconds{1000});
  //   // } catch (...) {
  //   //   // ignore
  //   // }
  //   throw ManipulationException(arlab_common_interfaces::msg::ManipulationResponse::GRASP_RESULT_TIME_OUT);
  // }

  if (result_future.wait_for(timeout) != std::future_status::ready)
  {
    // throw ManipulationException(arlab_common_interfaces::msg::ManipulationResponse::GRASP_RESULT_TIME_OUT);
  }

  // auto wrapped = result_future.get();
  // if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED)
  // {
  //   throw ManipulationException(arlab_common_interfaces::msg::ManipulationResponse::GRASP_ACTION_FAILED);
  // }

  // const auto & result = wrapped.result;
  // if (result && !result->err_message.empty()) {
  //   //throw std::runtime_error("Hand-Grasp error: " + result->err_message);
  // }
}
