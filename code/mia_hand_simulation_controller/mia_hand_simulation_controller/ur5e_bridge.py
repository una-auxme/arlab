import random

import rclpy
from rclpy.node import Node


from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint


class Bridge(Node):
    def __init__(self):
        super().__init__("ur5e_controller")
        self.get_logger().info("Waiting for MoveGroup...")

        self.client = ActionClient(self, MoveGroup, "/move_action")

        self.waiting_for_async_finish = False

        self.get_logger().info("Got MoveGroup")

        self.client.wait_for_server()
        # Execute every 5 seconds
        self.timer = self.create_timer(0.1, self.move_robot)

    def move_robot(self):

        if self.waiting_for_async_finish:
            return

        goal = MoveGroup.Goal()

        # Which MoveIt planning group to use
        goal.request.group_name = "ur_manipulator"

        # Joint goal
        joint_goal = Constraints()

        """joints = {
            "shoulder_pan_joint": 0.0,
            "shoulder_lift_joint": 0.0,
            "elbow_joint": 1.2,
            "wrist_1_joint": -1.0,
            "wrist_2_joint": 1.0,
            "wrist_3_joint": 0.0,
        }"""

        joints = self.random_joint_goal()

        for name, position in joints.items():
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = position
            constraint.tolerance_above = 0.01
            constraint.tolerance_below = 0.01
            constraint.weight = 1.0
            joint_goal.joint_constraints.append(constraint)

        goal.request.goal_constraints.append(joint_goal)

        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.2
        goal.request.max_acceleration_scaling_factor = 0.2

        self.get_logger().info(f"Sending goal {joints}")

        result = self.client.send_goal_async(goal)
        self.waiting_for_async_finish = True

        result.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return

        self.get_logger().info("Goal accepted")

        # Ask for the result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):

        result = future.result()

        # MoveIt error code
        error_code = result.result.error_code.val

        if error_code == 1:
            self.get_logger().info("Motion completed successfully")
        else:
            self.get_logger().error(f"Motion failed. MoveIt error code: {error_code}")

        self.waiting_for_async_finish = False

    def random_joint_goal(self):

        limits = {
            "shoulder_pan_joint": (-3.14, 3.14),
            "shoulder_lift_joint": (-2.0, 2.0),
            "elbow_joint": (-2.5, 2.5),
            "wrist_1_joint": (-3.14, 3.14),
            "wrist_2_joint": (-3.14, 3.14),
            "wrist_3_joint": (-3.14, 3.14),
        }

        return {joint: random.uniform(low, high) for joint, (low, high) in limits.items()}


def main():
    rclpy.init()

    node = Bridge()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
