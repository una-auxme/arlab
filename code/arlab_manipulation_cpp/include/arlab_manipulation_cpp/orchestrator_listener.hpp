// -----------------------------------------------------------------------------
// File: orchestrator_listener.hpp
// Package: arlab_manipulation_cpp
// Maintainer: Leonie Schmidt <leonie1.schmidt@uni-a.de>
//
// Declares OrchestratorActionServer, the ROS 2 action server that is the
// single external entry point for all manipulation requests. It owns the arm
// and hand motion components and delegates job execution to JobRunner.
// -----------------------------------------------------------------------------

#ifndef ARLAB_MANIPULATION_CPP_ORCHESTRATOR_LISTENER_H_
#define ARLAB_MANIPULATION_CPP_ORCHESTRATOR_LISTENER_H_

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "arlab_common_interfaces/action/orchestrator_action.hpp"

class ArmMotion;
class HandMotion;
class HandForceSwitch;
class JobRunner;

/**
 * ROS 2 action server that validates incoming manipulation goals, manages
 * execution threads, and delegates job execution to the JobRunner.
 */
class OrchestratorActionServer : public rclcpp::Node {

  public:
    using OrchestratorAction = arlab_common_interfaces::action::OrchestratorAction;
    using GoalHandleOrchestrator =
      rclcpp_action::ServerGoalHandle<OrchestratorAction>;

    /**
     * Constructs the ROS 2 node.
     * @param options   Optional ROS 2 node options forwarded to rclcpp::Node.
     */
    explicit OrchestratorActionServer(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    /**
     * Initialises the motion components and starts the action server.
     * Must be called exactly once after construction.
     */
    void Init();

  private:

    /**
     * Validates an incoming goal and accepts it if cmd is non-empty.
     * @param uuid    Unique identifier of the goal (unused).
     * @param goal    Shared pointer to the incoming goal message.
     * @returns ACCEPT_AND_EXECUTE if cmd is non-empty, REJECT otherwise.
     */
    rclcpp_action::GoalResponse HandleGoal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const OrchestratorAction::Goal> goal);

    /**
     * Handles a cancel request.
     * Note that cancellation is not supported and always returns REJECT.
     * @param goal_handle     Handle to the goal for which cancellation was requested.
     * @returns REJECT unconditionally.
     */
    rclcpp_action::CancelResponse HandleCancel(
      const std::shared_ptr<GoalHandleOrchestrator> goal_handle);

     /**
     * Spawns a detached thread to execute the accepted goal asynchronously.
     * @param goal_handle     Shared pointer to the accepted goal handle.
     */
    void HandleAccepted(std::shared_ptr<GoalHandleOrchestrator> goal_handle);

     /**
     * Executes the manipulation job and sends a structured result to the client.
     * @param goal_handle     Shared pointer to the goal handle used to send the result.
     */
    void Execute(std::shared_ptr<GoalHandleOrchestrator> goal_handle);

    rclcpp_action::Server<OrchestratorAction>::SharedPtr action_server_;

    std::unique_ptr<ArmMotion> arm_;
    std::unique_ptr<HandMotion> hand_;
    std::unique_ptr<HandForceSwitch> force_switch_;
    std::unique_ptr<JobRunner> runner_;
};

#endif  // ARLAB_MANIPULATION_CPP_ORCHESTRATOR_LISTENER_H_
