#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "arlab_common_interfaces/msg/orchestrator_data.hpp"


class PoseListener : public rclcpp::Node
{
public:
    PoseListener() : Node("PoseListener")
    {
        data_sub = this->create_subscription<arlab_common_interfaces::msg::OrchestratorData>(
            "/gripper_goal", 10,
            std::bind(&PoseListener::gripper_goal_callback, this, std::placeholders::_1));
    }

private:
    void gripper_goal_callback(const arlab_common_interfaces::msg::OrchestratorData::SharedPtr msg)
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
    }

    rclcpp::Subscription<arlab_common_interfaces::msg::OrchestratorData>::SharedPtr data_sub;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseListener>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
