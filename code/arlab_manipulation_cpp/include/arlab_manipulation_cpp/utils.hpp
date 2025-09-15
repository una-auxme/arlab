#ifndef UTILS_HPP
#define UTILS_HPP

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <rclcpp/logger.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

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
  const rclcpp::Logger &logger
);

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
  double ox, double oy, double oz, double ow
);

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
  const std::string &frame_id, std::string object_id,
  geometry_msgs::msg::Pose pose,
  double size_x, double size_y, double size_z
);


#endif
