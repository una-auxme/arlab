#include "arlab_manipulation_cpp/orchestrator_listener.hpp"
#include "arlab_manipulation_cpp/arm_motion.hpp"
#include "arlab_manipulation_cpp/hand_motion.hpp"
#include "arlab_manipulation_cpp/job_runner.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <thread>

OrchestratorActionServer::OrchestratorActionServer(const rclcpp::NodeOptions &options)
    : rclcpp::Node("OrchestratorActionServer", options)
{
}

void OrchestratorActionServer::init()
{
  // Capabilities initialisieren
  arm_ = std::make_unique<ArmMotion>(shared_from_this(), "ur_manipulator");
  hand_ = std::make_unique<HandMotion>(shared_from_this());
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

  const auto &data_msg = goal->data;
  RCLCPP_INFO(get_logger(), "OrchestratorAction Goal received (cmd=%s)", data_msg.cmd.data.c_str());

  if (data_msg.cmd.data.empty())
  {
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
  const auto &data_msg = goal->data;

  try
  {

    runner_->run(data_msg);
  }
  catch (const std::exception &e)
  {
    const char* msg = e.what();
    int code = (msg == nullptr || msg[0] == '\0') ? 0 : std::stoi(msg);
    RCLCPP_ERROR(get_logger(), "JobRunner exception: %d", code);

    auto result = std::make_shared<OrchestratorAction::Result>();
    result->response.error_code = code;
    result->response.message = "Manipulation Error: " + OrchestratorActionServer::errorMessageFromCode(code);

    goal_handle->succeed(result);

    return;
  }

  auto result = std::make_shared<OrchestratorAction::Result>();
  result->response.error_code = 1;
  result->response.message = "Manipulation completed successfully";

  goal_handle->succeed(result);

  RCLCPP_INFO(get_logger(), "JobRunner done, Result send back to Client");
}

std::string OrchestratorActionServer::errorMessageFromCode(int code)
{
  switch (code)
  {
  case 0:
    return "Undefined";
  case 1:
    return "Success";
  case -1:
    return "Planning failed";
  case -2:
    return "Invalid motion plan";
  case -3:
    return "Motion plan invalidated by environment change";
  case -4:
    return "Control failed";
  case -5:
    return "Unable to acquire sensor data";
  case -6:
    return "Timed out";
  case -7:
    return "Preempted";
  case -10:
    return "Start state in collision";
  case -11:
    return "Start state violates path constraints";
  case -12:
    return "Goal in collision";
  case -13:
    return "Goal violates path constraints";
  case -14:
    return "Goal constraints violated";
  case -15:
    return "Invalid group name";
  case -16:
    return "Invalid goal constraints";
  case -17:
    return "Invalid robot state";
  case -18:
    return "Invalid link name";
  case -19:
    return "Invalid object name";
  case -21:
    return "Frame transform failure";
  case -22:
    return "Collision checking unavailable";
  case -23:
    return "Robot state stale";
  case -24:
    return "Sensor info stale";
  case -25:
    return "Communication failure";
  case -26:
    return "Start state invalid";
  case -27:
    return "Goal state invalid";
  case -28:
    return "Unrecognized goal type";
  case -29:
    return "crash";
  case -30:
    return "abort";
  case -31:
    return "no IK solution";
  case -35:
    return "Unknown job command";
  case 99999:
    return "Unknown failure";
  default:
    return "Unknown error code";
  }

}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OrchestratorActionServer>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
