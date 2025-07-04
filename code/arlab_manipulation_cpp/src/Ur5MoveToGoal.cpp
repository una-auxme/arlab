#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64.hpp>

class PoseListener : public rclcpp::Node
{
public:
    PoseListener() : Node("Ur5MoveToGoal")
    {
        // goal_pose subscriber
        pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            std::bind(&PoseListener::pose_callback, this, std::placeholders::_1));

        // grip_force subscriber
        grip_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "/grip_force", 10,
            std::bind(&PoseListener::grip_callback, this, std::placeholders::_1));
    }

private:
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        auto &position = msg->pose.position;
        auto &orientation = msg->pose.orientation;

        RCLCPP_INFO(this->get_logger(),"Received pose - Orientation: w=%.3f | Position: x=%.3f, y=%.3f, z=%.3f", orientation.w, position.x, position.y, position.z);
    }
    
    void grip_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(),"Received grip force: %.2f N", msg->data);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr grip_subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseListener>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}



