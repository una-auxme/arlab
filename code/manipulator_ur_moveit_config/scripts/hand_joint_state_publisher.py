#!/usr/bin/env python3
"""Publish fixed joint states for the Mia hand on /joint_states (stop-gap until the real hand driver runs).

Why: manipulator.urdf.xacro contains the Mia hand. Without joint states for its joints,
robot_state_publisher publishes no finger TFs and MoveIt considers the robot state incomplete
(execution is refused). This node publishes the 4 actuated hand joints with a current stamp;
the mimic joints (j_little_fle, j_ring_fle) are derived by robot_state_publisher / MoveIt.

Positions are parameters (read every cycle -> `ros2 param set /hand_joint_state_publisher j_index_fle 0.5`
works at runtime). Defaults = SRDF state "hand_open". Set them to the real hand pose so the
octomap self-filter masks the fingers where they actually are.
Do NOT run this together with a real hand driver / joint_state_broadcaster for the hand.
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINTS = (("j_thumb_fle", 0.0), ("j_thumb_opp", -0.4), ("j_index_fle", 0.0), ("j_mrl_fle", 0.0))


class HandJointStatePublisher(Node):
    def __init__(self):
        super().__init__("hand_joint_state_publisher")
        self.declare_parameter("prefix", "")
        self.declare_parameter("rate", 10.0)
        for name, default in JOINTS:
            self.declare_parameter(name, default)
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.timer = self.create_timer(1.0 / self.get_parameter("rate").value, self.tick)

    def tick(self):
        prefix = self.get_parameter("prefix").value
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [prefix + name for name, _ in JOINTS]
        msg.position = [float(self.get_parameter(name).value) for name, _ in JOINTS]
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = HandJointStatePublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
