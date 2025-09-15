#include "arlab_manipulation_cpp/utils.hpp"

/**
 * @brief Create a geometry_msgs::msg::Pose.
 *
 * @param x Position along X axis.
 * @param y Position along Y axis.
 * @param z Position along Z axis.
 * @param ox Orientation x component (quaternion).
 * @param oy Orientation y component (quaternion).
 * @param oz Orientation z component (quaternion).
 * @param ow Orientation w component (quaternion).
 * @return geometry_msgs::msg::Pose Constructed pose.
 */
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


/**
 * @brief Create a MoveIt collision box object.
 *
 * @param frame_id Reference frame ID.
 * @param object_id Identifier of the collision object.
 * @param pose Pose of the collision box.
 * @param size_x Box dimension along X axis.
 * @param size_y Box dimension along Y axis.
 * @param size_z Box dimension along Z axis.
 * @return moveit_msgs::msg::CollisionObject Constructed collision box.
 */
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

/**
 * @brief Plan and execute a motion to a target pose.
 *
 * This function sets a target pose for the MoveGroupInterface, plans a motion,
 * and executes it if planning succeeds.
 *
 * @param move_group_interface Reference to MoveGroupInterface controlling the robot.
 * @param target_pose Target pose for the robot end-effector.
 * @param logger ROS 2 logger for error messages.
 * @return execution success.
 */
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


