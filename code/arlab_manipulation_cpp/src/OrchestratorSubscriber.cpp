
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>

#include "arlab_common_interfaces/msg/orchestrator_data.hpp"
#include "arlab_manipulation_cpp/job_runner.hpp"


/**
 * @brief Node that subscribes to orchestrator data and logs pose, gripping force and job command information.
 *
 * This node listens on the `/orchestrator_data` topic and processes messages
 * of type `arlab_common_interfaces::msg::OrchestratorData`. It extracts
 * position, orientation, gripping force and job command data from the received message and logs them.
 */
class OrchestratorListener : public rclcpp::Node {
  public:

    /**
    * @brief Construct a new OrchestratorListener Node.
    *
    * Initializes the node and creates a subscription to the
    * `/orchestrator_data` topic.
    */
    OrchestratorListener() : Node("OrchestratorListener") {
      data_sub_ =
        this->create_subscription<arlab_common_interfaces::msg::OrchestratorData>(
          "/orchestrator_data", 10,
          std::bind(&OrchestratorListener::orchestrator_data_callback, this, std::placeholders::_1)
        );
    }

  private:

    /**
    * @brief Callback for orchestrator data messages.
    *
    * Extracts the pose (position + orientation), gripping force and job command from the incoming message and
    * prints it to the ROS 2 logger. Runs the function "run_job" after extracting the data.
    *
    * @param msg Shared pointer to the incoming orchestrator data message.
    */
    void orchestrator_data_callback(const arlab_common_interfaces::msg::OrchestratorData::SharedPtr msg) {

      const auto& pose = msg->pose;
      const auto& position = pose.position;
      const auto& orientation = pose.orientation;

      RCLCPP_INFO(this->get_logger(),
        "Received data:\n"
        " Position: x=%.3f, y=%.3f, z=%.3f\n"
        " Orientation: ox=%.3f, oy=%.3f, oz=%.3f, ow=%.3f\n"
        " Grip force: %.2f N\n"
        " Command: '%s'",
        position.x, position.y, position.z,
        orientation.x, orientation.y, orientation.z, orientation.w,
        msg->grip_force.data,
        msg->cmd.data.c_str()
      );

      run_job(*msg,this->shared_from_this());
    }

    // Subscription to orchestrator data messages.
    rclcpp::Subscription<arlab_common_interfaces::msg::OrchestratorData>::SharedPtr data_sub_;
};

/**
 * @brief Main entry point of the program.
 *
 * Initializes ROS 2, starts the OrchestratorListener node, and spins until
 * shutdown is requested.
 *
 * @param argc Argument count.
 * @param argv Argument values.
 * @return int Exit status code.
 */
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OrchestratorListener>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
