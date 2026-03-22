// -----------------------------------------------------------------------------
// File: orchestrator_listener.cpp
// Package: arlab_manipulation_cpp
// Maintainer: Leonie Schmidt <leonie1.schmidt@uni-a.de>
//
// Implements OrchestratorActionServer and the main() entry point.
// The server lifecycle is:
//   main() → make_shared<OrchestratorActionServer>() → Init() → spin().
// Each accepted goal is executed in a separate detached thread via Execute()
// to keep the action callback group unblocked during robot motion.
// -----------------------------------------------------------------------------

#include "arlab_manipulation_cpp/orchestrator_listener.hpp"

#include <memory>
#include <thread>
#include <utility>

#include "arlab_manipulation_cpp/arm_motion.hpp"
#include "arlab_manipulation_cpp/hand_motion.hpp"
#include "arlab_manipulation_cpp/job_runner.hpp"
#include "arlab_manipulation_cpp/manipulator_exception.hpp"

namespace {

constexpr char kNodeName[] = "orchestrator_action_server";
constexpr char kActionName[] = "/orchestrator/action";
constexpr char kManipulatorName[] = "ur_manipulator";
constexpr int kSuccessCode = 0;
constexpr int kUnknownErrorCode = 99999;

} // namespace

OrchestratorActionServer::OrchestratorActionServer(
    const rclcpp::NodeOptions& options)
    : rclcpp::Node(kNodeName, options) {}

void OrchestratorActionServer::Init() {
  // Construct motion components after shared_from_this() is available.
  arm_ = std::make_unique<ArmMotion>(shared_from_this(), kManipulatorName);
  hand_ = std::make_unique<HandMotion>(shared_from_this());
  runner_ = std::make_unique<JobRunner>(*this, *arm_, *hand_);

  using std::placeholders::_1;
  using std::placeholders::_2;

  action_server_ = rclcpp_action::create_server<OrchestratorAction>(
      shared_from_this(),
      kActionName,
      std::bind(&OrchestratorActionServer::HandleGoal, this, _1, _2),
      std::bind(&OrchestratorActionServer::HandleCancel, this, _1),
      std::bind(&OrchestratorActionServer::HandleAccepted, this, _1));

  RCLCPP_INFO(get_logger(), "--- OrchestratorActionServer initialized ---");
}

rclcpp_action::GoalResponse OrchestratorActionServer::HandleGoal(
    const rclcpp_action::GoalUUID& /* uuid */,
    std::shared_ptr<const OrchestratorAction::Goal> goal) {

  const auto& data_msg = goal->data;
  RCLCPP_INFO(get_logger(), "OrchestratorAction Goal received (cmd=%s)",
              data_msg.cmd.data.c_str());

  // Reject goals that carry no command — JobRunner cannot process them.
  if (data_msg.cmd.data.empty()) {
    RCLCPP_WARN(get_logger(),
                "Rejecting OrchestratorAction Goal because cmd is empty.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse OrchestratorActionServer::HandleCancel(
    std::shared_ptr<GoalHandleOrchestrator> /* goal_handle */) {

  RCLCPP_INFO(get_logger(), "Received cancel request.");
  RCLCPP_INFO(get_logger(), "Cancellation is not supported.");

  return rclcpp_action::CancelResponse::REJECT;
}

void OrchestratorActionServer::HandleAccepted(
    std::shared_ptr<GoalHandleOrchestrator> goal_handle) {

  // Execute the job in a detached thread so that the action server callbacks
  // remain responsive while the robot is in motion. The goal_handle is moved
  // into the thread to extend its lifetime until Execute() returns.
  std::thread(&OrchestratorActionServer::Execute, this, std::move(goal_handle))
    .detach();
}

void OrchestratorActionServer::Execute(
    std::shared_ptr<GoalHandleOrchestrator> goal_handle) {

  RCLCPP_INFO(get_logger(), "Start JobRunner execution");

  const auto goal = goal_handle->get_goal();
  const auto& data_msg = goal->data;

  auto result = std::make_shared<OrchestratorAction::Result>();

  try {

    runner_->Run(data_msg);
    result->response.error_code = kSuccessCode;
    result->response.message = "Manipulation completed successfully";
    goal_handle->succeed(result);

    RCLCPP_INFO(get_logger(), "JobRunner finished. Result sent to client.");
    return;

  } catch (const ManipulationException& e) {
    // Known manipulation failure: propagate the typed error code.
    RCLCPP_ERROR(get_logger(), "JobRunner exception: %s", e.what());

    result->response.error_code = e.code();
    result->response.message = "Manipulation Error: " + std::string(e.what());
    goal_handle->succeed(result);

    return;

  } catch (const std::exception& e) {
    // Unexpected failure: use the generic unknown error code.
    RCLCPP_ERROR(get_logger(), "JobRunner unknown exception: %s", e.what());

    result->response.error_code = kUnknownErrorCode;
    result->response.message = "Manipulation Unknown Error: " + std::string(e.what());
    goal_handle->succeed(result);

    return;
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  // Two-step initialisation: construct the node first so that
  // shared_from_this() is valid inside Init().
  auto node = std::make_shared<OrchestratorActionServer>();
  node->Init();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
