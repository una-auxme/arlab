#include "arlab_manipulation_cpp/utils.hpp"

geometry_msgs::msg::Pose createPose(
  double x, double y, double z,
  double ox, double oy, double oz, double ow) {

  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.x = ox;
  pose.orientation.y = oy;
  pose.orientation.z = oz;
  pose.orientation.w = ow;
  
  return pose;
}

moveit_msgs::msg::CollisionObject createCollisionBox(
  const std::string &frame_id,
  std::string object_id,geometry_msgs::msg::Pose pose,
  double size_x, double size_y, double size_z) {

  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = frame_id;
  collision_object.id = object_id;

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = size_x;
  primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = size_y;
  primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = size_z;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(pose);
  collision_object.operation = collision_object.ADD;

  return collision_object;
}


bool planAndExecutePose(
  moveit::planning_interface::MoveGroupInterface &move_group_interface,
  const geometry_msgs::msg::Pose &target_pose,
  const rclcpp::Logger &logger) {

  // Set target pose
  move_group_interface.setPoseTarget(target_pose);

  // Visualization
  auto const text_pose = [] {
    auto pose = Eigen::Isometry3d::Identity();
    pose.translation().z() = 1.0;
    return pose;
  }();

  // Plan trajectory
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(move_group_interface.plan(plan));

  if (success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  return success;
}


