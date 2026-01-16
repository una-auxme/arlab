#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit_core/moveit/utils/moveit_error_code.hpp>
#include "arlab_common_interfaces/msg/orchestrator_data.hpp"
#include "arlab_common_interfaces/action/orchestrator_action.hpp"
#include "mia_hand_msgs/action/grasp.hpp"

class ManipulationException : public std::exception
{
public:
  explicit ManipulationException(int code);
  explicit ManipulationException(int code, std::string &&msg);
  explicit ManipulationException(const moveit::core::MoveItErrorCode &code);
  const char *what() const noexcept override;
  int code() const noexcept { return code_; }

private:
  int code_;
  std::string msg_;
};

// class JobRunner;

class OrchestratorActionServer : public rclcpp::Node
{
public:
  using OrchestratorAction = arlab_common_interfaces::action::OrchestratorAction;
  using GoalHandleOrchestrator = rclcpp_action::ServerGoalHandle<OrchestratorAction>;
  // using Grasp = Mia_hand_msgs::action::Grasp;

  explicit OrchestratorActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  void init();

private:
  // Action-Server Callbacks
  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const OrchestratorAction::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandleOrchestrator> goal_handle);
  void handleAccepted(const std::shared_ptr<GoalHandleOrchestrator> goal_handle);
  void execute(const std::shared_ptr<GoalHandleOrchestrator> goal_handle);

  // Action-Server
  rclcpp_action::Server<OrchestratorAction>::SharedPtr action_server_;
  // rclcpp_action::Client<Grasp>::SharedPtr grasp_client_;

  // Owned capabilities
  std::unique_ptr<class ArmMotion> arm_;
  std::unique_ptr<class HandMotion> hand_;
  std::unique_ptr<class JobRunner> runner_;
};

std::string errorMessageFromCode(int code);
