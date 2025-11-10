#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <vector>

class HandMotion {
public:
  explicit HandMotion(rclcpp::Node& node);

  void open();
  void close();
  void point();

  void send_pos(double thumb, double index, double mrl);

private:
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_thumb_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_index_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_mrl_;

  void publishArray(const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr& pub,
                    const std::vector<double>& data);
};
