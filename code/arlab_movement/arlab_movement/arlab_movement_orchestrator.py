#!/usr/bin/env python3

import rclpy
from arlab_common_interfaces.msg import Destination, MovementCommand
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool, String


class MovementOrchestratorNode(Node):
    def __init__(self):
        super().__init__("MovementOrchestratorNode")

        self.activate_mapping = False
        self.activate_localization = False
        self.has_destination = False
        self.destination = None  # Oder einen Standardwert vom Typ Destination()

        self.publisher_ = self.create_publisher(String, "heartbeat", 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.setup_subscribers()
        self.setup_publishers()

    def setup_subscribers(self):
        self.pose_sub = self.create_subscription(
            msg_type=MovementCommand,
            callback=self.receive_command,
            topic="/MovementCommand",
            qos_profile=1,
        )

    def setup_publishers(self):
        """Setup publishers for all components of a MovementCommand msg"""
        topic_prefix = "/arlab_movement"

        self.activate_mapping_pub = self.create_publisher(
            msg_type=Bool,
            topic=f"{topic_prefix}/activate_mapping",
            qos_profile=1,
        )
        self.activate_localization_pub = self.create_publisher(
            msg_type=Bool,
            topic=f"{topic_prefix}/activate_localization",
            qos_profile=1,
        )
        self.has_destination_pub = self.create_publisher(
            msg_type=Bool,
            topic=f"{topic_prefix}/has_destination",
            qos_profile=1,
        )
        self.destination_pub = self.create_publisher(
            msg_type=Destination,
            topic=f"{topic_prefix}/destination",
            qos_profile=1,
        )

    def timer_callback(self):
        msg = String()
        msg.data = "MovementOrchestratorNode alive"
        self.publisher_.publish(msg)
        self.i += 1

    def receive_command(self, movement_command_msg):
        """Is called, when a MovementCommand gets received.

        Splits the message and republisheds the smaller parts.
        """

        msg_map = Bool()
        msg_map.data = movement_command_msg.activate_mapping
        self.activate_mapping_pub.publish(msg_map)

        msg_loc = Bool()
        msg_loc.data = movement_command_msg.activate_localization
        self.activate_localization_pub.publish(msg_loc)

        msg_has_dest = Bool()
        msg_has_dest.data = movement_command_msg.has_destination
        self.has_destination_pub.publish(msg_has_dest)

        if movement_command_msg.has_destination:
            self.destination_pub.publish(movement_command_msg.destination)


def main(args=None):
    rclpy.init(args=args)

    movement_orchestrator = MovementOrchestratorNode()

    rclpy.spin(movement_orchestrator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    movement_orchestrator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
