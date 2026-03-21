#ifndef ARLAB_MANIPULATION_CPP_ORCHESTRATOR_LISTENER_H_
#define ARLAB_MANIPULATION_CPP_ORCHESTRATOR_LISTENER_H_

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "arlab_common_interfaces/action/orchestrator_action.hpp"

class ArmMotion;
class HandMotion;
class JobRunner;

// ROS 2 action server that receives orchestration requests and delegates
// execution to the JobRunner.
class OrchestratorActionServer : public rclcpp::Node {

  public:
    using OrchestratorAction = arlab_common_interfaces::action::OrchestratorAction;
    using GoalHandleOrchestrator =
      rclcpp_action::ServerGoalHandle<OrchestratorAction>;

    explicit OrchestratorActionServer(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    // Initializes owned motion components and starts the action server.
    void Init();

  private:
    // Validates an incoming goal request.
    rclcpp_action::GoalResponse HandleGoal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const OrchestratorAction::Goal> goal);

    // Handles a cancel request for a goal.
    rclcpp_action::CancelResponse HandleCancel(
      const std::shared_ptr<GoalHandleOrchestrator> goal_handle);

    // Accepts a goal for asynchronous execution.
    void HandleAccepted(std::shared_ptr<GoalHandleOrchestrator> goal_handle);

    // Executes the orchestration job and reports the result to the client.
    void Execute(std::shared_ptr<GoalHandleOrchestrator> goal_handle);

    rclcpp_action::Server<OrchestratorAction>::SharedPtr action_server_;

    std::unique_ptr<ArmMotion> arm_;
    std::unique_ptr<HandMotion> hand_;
    std::unique_ptr<JobRunner> runner_;
};

#endif  // ARLAB_MANIPULATION_CPP_ORCHESTRATOR_LISTENER_H_
