
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>


// MoveIt
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

#include "arlab_manipulation_cpp/utils.hpp"


int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Spin up a SingleThreadedExecutor for MoveItVisualTools to interact with ROS
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");

  // Construct and initialize MoveItVisualTools
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
      node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
      move_group_interface.getRobotModel()};
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  // Create closures for visualization
  auto const draw_title = [&moveit_visual_tools](auto text) {
    auto const text_pose = [] {
      auto msg = Eigen::Isometry3d::Identity();
      msg.translation().z() = 1.0;  // Place text 1m above the base link
      return msg;
    }();
    moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE,rviz_visual_tools::XLARGE);
  };
  auto const prompt = [&moveit_visual_tools](auto text) {
    moveit_visual_tools.prompt(text);
  };
  auto const draw_trajectory_tool_path =
      [&moveit_visual_tools,
      jmg = move_group_interface.getRobotModel()->getJointModelGroup(
          "ur_manipulator")](auto const trajectory) {
        moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
      };

  // Set a target Pose
  auto home_pose = createPose(-0.12,0.5,0.6,0.996,0.041,0.009,0.076);
  auto table_pose = createPose(0.372,0.124,0.621,0.999,0.041,0.006,0.004);
  auto standup_pose = createPose(0.1,0.233,0.98,-0.5,0.5,0.5,0.5);

  // Create collision object for the robot to avoid
  auto frame_id = move_group_interface.getPlanningFrame();
  auto collision_object_pose = createPose(-0.4,0.0,0.25,0,0,0,1);
  auto collision_object = createCollisionBox(frame_id,"box1",collision_object_pose,0.1,0.1,0.5);

  auto collision_object_pose2 = createPose(0.6,0.0,0.25,0,0,0,1);
  auto collision_object2 = createCollisionBox(frame_id,"table",collision_object_pose2,1,1,0.2);

  // Add the collision object to the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObject(collision_object);
  planning_scene_interface.applyCollisionObject(collision_object2);

  // ----------------- Plan and Execute -----------------------
  // - Move to Home Pose
  auto success = planAndExecutePose(move_group_interface,home_pose,moveit_visual_tools,logger);
  if (!success) {
    planAndExecutePose(move_group_interface,standup_pose,moveit_visual_tools,logger);
    planAndExecutePose(move_group_interface,home_pose,moveit_visual_tools,logger);
  }
  // - Move to Table Pose
  auto success2 = planAndExecutePose(move_group_interface,table_pose,moveit_visual_tools,logger);
  if (!success2) {
    planAndExecutePose(move_group_interface,standup_pose,moveit_visual_tools,logger);
    planAndExecutePose(move_group_interface,table_pose,moveit_visual_tools,logger);
  }
  // - Move to Home Pose
  auto success3 = planAndExecutePose(move_group_interface,home_pose,moveit_visual_tools,logger);
  if (!success3) {
    planAndExecutePose(move_group_interface,standup_pose,moveit_visual_tools,logger);
    planAndExecutePose(move_group_interface,home_pose,moveit_visual_tools,logger);
  }
  // ----------------------------------------------------


  // Shutdown ROS
  rclcpp::shutdown();  // <--- This will cause the spin function in the thread to return
  spinner.join();  // <--- Join the thread before exiting
  return 0;
}
