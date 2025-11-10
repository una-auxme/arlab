#include "arlab_manipulation_cpp/orchestrator_listener.hpp"
#include "arlab_manipulation_cpp/arm_motion.hpp"
#include "arlab_manipulation_cpp/hand_motion.hpp"
#include "arlab_manipulation_cpp/job_runner.hpp"

#include <geometry_msgs/msg/pose.hpp>

OrchestratorListener::OrchestratorListener(const rclcpp::NodeOptions& options)
: rclcpp::Node("OrchestratorListener", options)
{

}

void OrchestratorListener::init()
{
  // Capabilities initialisieren
  arm_  = std::make_unique<ArmMotion>(shared_from_this(), "ur_manipulator");
  hand_ = std::make_unique<HandMotion>(*this);
  runner_ = std::make_unique<JobRunner>(*this, *arm_, *hand_);

  // Subscriber
  sub_ = create_subscription<arlab_common_interfaces::msg::OrchestratorData>(
      "/orchestrator_data", rclcpp::QoS(10),
      std::bind(&OrchestratorListener::onMsg, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "--- OrchestratorListener initialized ---");
}

void OrchestratorListener::onMsg(
    const arlab_common_interfaces::msg::OrchestratorData::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "orchestrator_data received (cmd=%s)", msg->cmd.data.c_str());
  runner_->run(*msg);
}

// main
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OrchestratorListener>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
