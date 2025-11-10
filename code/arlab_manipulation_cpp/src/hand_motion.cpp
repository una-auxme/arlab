#include "arlab_manipulation_cpp/hand_motion.hpp"

HandMotion::HandMotion(rclcpp::Node& node) {
  pub_thumb_ = node.create_publisher<std_msgs::msg::Float64MultiArray>(
      "/thumb_joint_position_controller/commands", 1);
  pub_index_ = node.create_publisher<std_msgs::msg::Float64MultiArray>(
      "/index_joint_position_controller/commands", 1);
  pub_mrl_   = node.create_publisher<std_msgs::msg::Float64MultiArray>(
      "/mrl_joint_position_controller/commands", 1);
}

void HandMotion::publishArray(
    const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr& pub,
    const std::vector<double>& data) {
  std_msgs::msg::Float64MultiArray msg;
  msg.data = data;
  pub->publish(msg);
}

void HandMotion::open()  { send_pos(0.1,  0.0, 0.1); }
void HandMotion::close() { send_pos(0.3,  0.9, 0.9); }
void HandMotion::point() { send_pos(0.3, -0.1, 1.0); }

void HandMotion::send_pos(double thumb, double index, double mrl) {
  publishArray(pub_thumb_, {thumb});
  publishArray(pub_index_, {index});
  publishArray(pub_mrl_,   {mrl});
}
