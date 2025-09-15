
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

#include "arlab_common_interfaces/msg/orchestrator_data.hpp"
#include "arlab_manipulation_cpp/utils.hpp"



void job_gripper_open(
  const rclcpp::Logger &logger) {

  // gripper_pos.joint_names.resize(1);
  // gripper_pos.joint_names[0] = "r_robotiq_85_left_knuckle_joint";

  // // Set the gripper as open
  // gripper_pos.points.resize(1);
  // gripper_pos.points[0].positions.resize(1)
  // gripper_pos.points[0].positions[0] = 0;
  // gripper_pos.points[0].time_from_start = ros::Duration(0.5);

  RCLCPP_INFO(logger,"Greifer-Befehl gesendet!");
}


void job_gripper_close(
  const rclcpp::Logger &logger) {

  // gripper_pos.joint_names.resize(1);
  // gripper_pos.joint_names[0] = "r_robotiq_85_left_knuckle_joint";

  // // Set the gripper as open
  // gripper_pos.points.resize(1);
  // gripper_pos.points[0].positions.resize(1)
  // gripper_pos.points[0].positions[0] = 0.5;
  // gripper_pos.points[0].time_from_start = ros::Duration(0.5);

  RCLCPP_INFO(logger,"Greifer-Befehl gesendet!");
}

void job_move2pose(
  moveit::planning_interface::MoveGroupInterface &move_group_interface,
  const geometry_msgs::msg::Pose &target_pose,
  const rclcpp::Logger &logger) {

  auto success = planAndExecutePose(move_group_interface,target_pose,moveit_visual_tools,logger);

}

void job_move2home(
  moveit::planning_interface::MoveGroupInterface &move_group_interface,
  const rclcpp::Logger &logger) {

  // Set Home Pose
  auto home_pose = createPose(-0.12,0.5,0.6,0.996,0.041,0.009,0.076);

  // 1. Is in Home Pos?
  // -> False: Move to Home Pos
  auto success = planAndExecutePose(move_group_interface,home_pose,moveit_visual_tools,logger);

}


void job_pick(
  moveit::planning_interface::MoveGroupInterface &move_group_interface,
  const geometry_msgs::msg::Pose &target_pose,
  const rclcpp::Logger &logger) {

  // Set a target Pose
  auto home_pose = createPose(-0.12,0.5,0.6,0.996,0.041,0.009,0.076);
  auto table_pose = createPose(0.372,0.124,0.3,0.999,0.041,0.006,0.004);
  auto standup_pose = createPose(0.3,0.233,0.7,-0.5,0.5,0.5,0.5);

  // 1. Is in Home Pos?
  // -> False: Move to Home Pos
  auto success = planAndExecutePose(move_group_interface,home_pose,logger);
  // if (!success) {
  //   planAndExecutePose(move_group_interface,standup_pose,logger);
  //   planAndExecutePose(move_group_interface,home_pose,logger);
  // }

  // 2. Move to Table Pose
  auto success2 = planAndExecutePose(move_group_interface,table_pose,logger);
  // if (!success2) {
  //   planAndExecutePose(move_group_interface,standup_pose,logger);
  //   planAndExecutePose(move_group_interface,table_pose,logger);
  // }

  // 3. Move to Target Pos

  // 4. Close Gripper

  // 5. Move to Table Pose

  // 6. Move to Home Pos
  auto success3 = planAndExecutePose(move_group_interface,home_pose,logger);
  // if (!success3) {
  //   planAndExecutePose(move_group_interface,standup_pose,logger);
  //   planAndExecutePose(move_group_interface,home_pose,logger);
  // }


}

void job_place(){
  // ----------------- Plan and Execute -----------------

  // 1. Is in Home Pos?
  // -> False: Move to Home Pos

  // 3. Move to Target Pos

  // 4. Close Gripper

  // 6. Move to Home Pos

  // ----------------------------------------------------

}


int run_job(
  const arlab_common_interfaces::msg::OrchestratorData &msg,
  std::shared_ptr<rclcpp::Node> node) {

  // -------------------- Setup --------------------

  auto logger = node->get_logger();

  RCLCPP_INFO(logger,"Start Job Run!");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");


  // ---------------- Job Execution ----------------

  // Later received from subscriber
  std::string cmd = "pick";
  auto target_pose = createPose(0.1,0.233,0.98,-0.5,0.5,0.5,0.5);

  if (cmd == "pick") {
    // Job pick
    RCLCPP_INFO(logger,"Job pick!");

    job_pick(move_group_interface,target_pose,logger);

  } else if (cmd == "place") {
    // Job place
    RCLCPP_INFO(logger,"Job place!");

    job_place();

  } else if (cmd == "open") {
    // Job gripper open
    RCLCPP_INFO(logger,"Job gripper open!");

    job_gripper_open(logger);

  } else if (cmd == "close") {
    // Job gripper close
    RCLCPP_INFO(logger,"Job gripper close!");

    job_gripper_close(logger);

  } else if (cmd == "move") {
    // Job move to pose
    RCLCPP_INFO(logger,"Job move to pose!");

    job_move2pose(move_group_interface,target_pose,logger);

  } else if (cmd == "home") {
    // Job move to home
    RCLCPP_INFO(logger,"Job move to home!");

    job_move2home(move_group_interface,logger);

  }

  return 0;
}
