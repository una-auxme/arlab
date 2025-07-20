#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "arlab_common_interfaces/msg/orchestrator_data.hpp"

#include "arlab_manipulation_cpp/job_runner.hpp"

class OrchestratorListener : public rclcpp::Node
{
public:
    OrchestratorListener() : Node("OrchestratorListener")
    {
        data_sub = this->create_subscription<arlab_common_interfaces::msg::OrchestratorData>(
            "/orchestrator_data", 10,
            std::bind(&OrchestratorListener::orchestrator_data_callback, this, std::placeholders::_1));
    }

private:
    void orchestrator_data_callback(const arlab_common_interfaces::msg::OrchestratorData::SharedPtr msg)
    {
        const auto &pose = msg->pose;
        const auto &position = pose.position;
        const auto &orientation = pose.orientation;

        RCLCPP_INFO(this->get_logger(),
            "Received data:\n"
            " Position: x=%.3f, y=%.3f, z=%.3f\n"
            " Orientation: ox=%.3f, oy=%.3f, oz=%.3f, ow=%.3f\n"
            " Grip force: %.2f N\n"
            " Command: '%s'",
            position.x, position.y, position.z,
            orientation.x, orientation.y, orientation.z, orientation.w,
            msg->grip_force.data,
            msg->cmd.data.c_str());

        run_job(*msg,this->shared_from_this());
    }

    rclcpp::Subscription<arlab_common_interfaces::msg::OrchestratorData>::SharedPtr data_sub;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OrchestratorListener>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
