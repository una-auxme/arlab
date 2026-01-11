#include "hand_motion.hpp"

#include <stdexcept>

HandMotion::HandMotion(rclcpp::Node::SharedPtr node, std::string action_name)
: node_(std::move(node)), action_name_(std::move(action_name))
{
  if (!node_) {
    throw std::invalid_argument("HandMotion: node ist null");
  }
  client_ = rclcpp_action::create_client<Grasp>(node_, action_name_);
}

void HandMotion::open(std::chrono::milliseconds timeout)
{
  grasp(/*target_closure_percent=*/0, /*speed_for_percent=*/15, timeout);
}

void HandMotion::close(std::chrono::milliseconds timeout)
{
  grasp(/*target_closure_percent=*/100, /*speed_for_percent=*/15, timeout);
}

void HandMotion::grasp(
  int target_closure_percent,
  int speed_for_percent,
  std::chrono::milliseconds timeout,
  std::chrono::milliseconds server_wait)
{
  // optionale Begrenzung
  if (target_closure_percent < 0) target_closure_percent = 0;
  if (target_closure_percent > 100) target_closure_percent = 100;

  // 1) Action-Server da?
  if (!client_->wait_for_action_server(server_wait)) {
    throw std::runtime_error("Grasp Action-Server nicht erreichbar: " + action_name_);
  }

  // 2) Goal bauen
  Grasp::Goal goal;
  goal.spe_for_percent = speed_for_percent;
  goal.target_closure_percent = target_closure_percent;

  // 3) Goal senden
  auto goal_future = client_->async_send_goal(goal);
  auto rc = rclcpp::spin_until_future_complete(node_, goal_future, timeout);
  if (rc != rclcpp::FutureReturnCode::SUCCESS) {
    throw std::runtime_error("Timeout/Fehler beim Senden des Grasp-Goals");
  }

  auto goal_handle = goal_future.get();
  if (!goal_handle) {
    throw std::runtime_error("Grasp-Goal wurde abgelehnt (goal_handle null)");
  }

  // 4) Result holen
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
    throw std::runtime_error("Timeout/Fehler beim Warten auf Grasp-Result");
  }

  auto wrapped = result_future.get();
  if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED) {
    throw std::runtime_error("Grasp Action nicht erfolgreich (ResultCode != SUCCEEDED)");
  }

  const auto & result = wrapped.result;
  if (result && !result->err_message.empty()) {
    throw std::runtime_error("Hand-Grasp Fehler: " + result->err_message);
  }
}
