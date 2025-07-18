#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

class PoseListener : public rclcpp::Node
{
public:
    PoseListener() : Node("PoseListener")
    {
        // Subscriber für Pose
        pose_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/goalpose", 10,
            std::bind(&PoseListener::pose_callback, this, std::placeholders::_1));

        // Subscriber für Greifkraft
        grip_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "/gripforce", 10,
            std::bind(&PoseListener::grip_callback, this, std::placeholders::_1));
        
        // Subscriber für Command
        cmd_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/cmd", 10,
            std::bind(&PoseListener::cmd_callback, this, std::placeholders::_1));
    }

private:
    void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        auto &position = msg->position;
        auto &orientation = msg->orientation;

        RCLCPP_INFO(this->get_logger(),
            "Received pose - Position: x=%.3f, y=%.3f, z=%.3f | Orientation: ox=%.3f, oy=%.3f, oz=%.3f, ow=%.3f",
            position.x, position.y, position.z,
            orientation.x, orientation.y, orientation.z, orientation.w);
    }

    void grip_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received grip force: %.2f N", msg->data);
    }

    void cmd_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received command: '%s'", msg->data.c_str());
    }

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr grip_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseListener>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
