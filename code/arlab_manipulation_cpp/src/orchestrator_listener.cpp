#include "arlab_manipulation_cpp/orchestrator_listener.hpp"
#include "arlab_manipulation_cpp/arm_motion.hpp"
#include "arlab_manipulation_cpp/hand_motion.hpp"
#include "arlab_manipulation_cpp/job_runner.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <thread>

OrchestratorActionServer::OrchestratorActionServer(const rclcpp::NodeOptions& options)
: rclcpp::Node("OrchestratorActionServer", options)
{

}

void OrchestratorActionServer::init()
{
  // Capabilities initialisieren
  arm_  = std::make_unique<ArmMotion>(shared_from_this(), "ur_manipulator");
  hand_ = std::make_unique<HandMotion>(*this);
  runner_ = std::make_unique<JobRunner>(*this, *arm_, *hand_);

  using std::placeholders::_1;
  using std::placeholders::_2;

  // ---- Action-Server ----
  action_server_ = rclcpp_action::create_server<OrchestratorAction>(
    shared_from_this(),
    "/orchestrator/action",
    std::bind(&OrchestratorActionServer::handleGoal, this, _1, _2),
    std::bind(&OrchestratorActionServer::handleCancel, this, _1),
    std::bind(&OrchestratorActionServer::handleAccepted, this, _1));


  RCLCPP_INFO(get_logger(), "--- OrchestratorActionServer initialized ---");
}

// void OrchestratorActionServer::onMsg(
//     const arlab_common_interfaces::msg::OrchestratorData::SharedPtr msg)
// {
//   RCLCPP_INFO(get_logger(), "orchestrator_data received (cmd=%s)", msg->cmd.data.c_str());
//   runner_->run(*msg);
// }

// Gets called when a new Goal is received from a client
rclcpp_action::GoalResponse OrchestratorActionServer::handleGoal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const OrchestratorAction::Goal> goal)
{

  const auto & data_msg = goal->data;
  RCLCPP_INFO(get_logger(), "OrchestratorAction Goal received (cmd=%s)", data_msg.cmd.data.c_str());

  if (data_msg.cmd.data.empty()) {
    RCLCPP_WARN(get_logger(), "OrchestratorAction Goal has empty cmd, rejecting.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// Gets called when a Cancel-Request is received from a client
rclcpp_action::CancelResponse OrchestratorActionServer::handleCancel(
  std::shared_ptr<GoalHandleOrchestrator> goal_handle)
{
  RCLCPP_INFO(get_logger(), "Cancel-Request erhalten");
  RCLCPP_INFO(get_logger(), "Canceling not supported, rejecting.");

  return rclcpp_action::CancelResponse::REJECT;
}

// Gets called when a Goal was accepted
void OrchestratorActionServer::handleAccepted(
  const std::shared_ptr<GoalHandleOrchestrator> goal_handle)
{
  // Job execution starts in its own threat to not block callbacks
  std::thread{std::bind(&OrchestratorActionServer::execute, this, goal_handle)}.detach();
}

void OrchestratorActionServer::execute(
  const std::shared_ptr<GoalHandleOrchestrator> goal_handle)
{
  RCLCPP_INFO(get_logger(), "Start JobRunner execution");

  const auto goal = goal_handle->get_goal();
  const auto & data_msg = goal->data;

  try {

    runner_->run(data_msg);

  } catch (const std::exception & e) {

    RCLCPP_ERROR(get_logger(), "JobRunner exception: %s", e.what());

    auto result = std::make_shared<OrchestratorAction::Result>();
    //result->success = false;
    result->response.message = "error";
    goal_handle->abort(result);

    return;

  }

  auto result = std::make_shared<OrchestratorAction::Result>();
  //result->success = true;
  result->response.message = "done";
  goal_handle->succeed(result);

  RCLCPP_INFO(get_logger(), "JobRunner done, Result send back to Client");
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OrchestratorActionServer>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
