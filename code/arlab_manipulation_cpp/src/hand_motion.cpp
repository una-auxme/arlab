#include "arlab_manipulation_cpp/hand_motion.hpp"
#include "mia_hand_msgs/action/grasp.hpp"

HandMotion::HandMotion(rclcpp::Node &node)
{
  // pub_thumb_ = node.create_publisher<std_msgs::msg::Float64MultiArray>(
  //     "/thumb_joint_position_controller/commands", 1);
  // pub_index_ = node.create_publisher<std_msgs::msg::Float64MultiArray>(
  //     "/index_joint_position_controller/commands", 1);
  // pub_mrl_   = node.create_publisher<std_msgs::msg::Float64MultiArray>(
  //     "/mrl_joint_position_controller/commands", 1);

  // grasp_client_ = node.grasp_client_;
}

// void HandMotion::publishArray(
//     const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr& pub,
//     const std::vector<double>& data) {
//   std_msgs::msg::Float64MultiArray msg;
//   msg.data = data;
//   pub->publish(msg);
// }

// void HandMotion::open()  { send_pos(0.1,  0.0, 0.1); }
// void HandMotion::close() { send_pos(0.3,  0.9, 0.9); }
// void HandMotion::point() { send_pos(0.3, -0.1, 1.0); }

// void HandMotion::send_pos(double thumb, double index, double mrl) {
//   publishArray(pub_thumb_, {thumb});
//   publishArray(pub_index_, {index});
//   publishArray(pub_mrl_,   {mrl});
// }

void HandMotion::executeGrasp(int close_percent, int spe_for_percent)
{
  // // Warten bis der Action Server bereit ist
  // if (!grasp_client_->wait_for_action_server(std::chrono::seconds(2)))
  // {
  //   RCLCPP_ERROR(this->get_logger(), "Hand Action Server nicht verfügbar");
  //   return;
  // }

  // // Ziel vorbereiten
  // auto goal_msg = mia_hand_msgs::action::Grasp::Goal();
  // goal_msg.close_percent = close_percent;
  // goal_msg.spe_for_percent = spe_for_percent;

  // // Optionen mit Callbacks
  // auto send_options = rclcpp_action::Client<mia_hand_msgs::action::Grasp>::SendGoalOptions();
  // send_options.goal_response_callback =
  //     [](std::shared_future<rclcpp_action::ClientGoalHandle<Grasp>::SharedPtr> future)
  // {
  //   auto handle = future.get();
  //   if (!handle)
  //   {
  //     RCLCPP_ERROR(rclcpp::get_logger("grasp"), "Goal abgelehnt");
  //   }
  //   else
  //   {
  //     RCLCPP_INFO(rclcpp::get_logger("grasp"), "Goal akzeptiert");
  //   }
  // };

  // send_options.feedback_callback =
  //     [](rclcpp_action::ClientGoalHandle<Grasp>::SharedPtr,
  //        const std::shared_ptr<const mia_hand_msgs::action::Grasp::Feedback> feedback)
  // {
  //   RCLCPP_INFO(rclcpp::get_logger("grasp"),
  //               "Hand Feedback – Positions: %d",
  //               feedback->current_positions);
  // };

  // send_options.result_callback =
  //     [](const rclcpp_action::ClientGoalHandle<Grasp>::WrappedResult &result)
  // {
  //   RCLCPP_INFO(rclcpp::get_logger("grasp"),
  //               "Action beendet, Code: %d",
  //               result.code);
  // };

  // // Goal senden
  // grasp_client_->send_goal(goal_msg, send_options);
}

void HandMotion::open_hand()
{
  // executeGrasp(0, 20);
}

void HandMotion::close_hand()
{
  // executeGrasp(70, 20);
}
