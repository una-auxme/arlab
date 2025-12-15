#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <vector>
#include "mia_hand_msgs/action/grasp.hpp"

class HandMotion
{
public:
  explicit HandMotion(rclcpp::Node &node);

  // using Grasp = mia_hand_msgs::action::Grasp;
  // using GoalHandleGrasp = rclcpp_action::ClientGoalHandle<Grasp>;

  // void open();
  // void close();
  // void point();

  // void send_pos(double thumb, double index, double mrl);

  void open_hand();
  void close_hand();

private:
  // rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_thumb_;
  // rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_index_;
  // rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_mrl_;

  // void publishArray(const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr& pub,
  //                   const std::vector<double>& data);

  void executeGrasp(int close_percent, int spe_for_percent);
  // rclcpp_action::Client<Grasp>::SharedPtr grasp_client_;
};
