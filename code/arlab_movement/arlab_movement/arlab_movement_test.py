#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String


class MovementNode(Node):
    def __init__(self):
        super().__init__("minimal_publisher")
        self.publisher_ = self.create_publisher(String, "topic", 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.setup_subscriber()
        self.setup_publisher()

    def setup_subscriber(self):
        self.pose_sub = self.create_subscription(
            msg_type=PoseStamped,
            callback=self.receive_pose,
            topic="/pose",
            qos_profile=1,
        )

    def setup_publisher(self):
        self.pose_pub = self.create_publisher(
            msg_type=String,  # geometry_msgs/PoseStamped
            topic="/goal_pose",
            qos_profile=1,
        )
        self.j = 0

    def timer_callback(self):
        msg = String()
        msg.data = "Hello World: %d" % self.i
        self.publisher_.publish(msg)
        self.i += 1

    def receive_pose(self, pose_msg):
        point = pose_msg.pose.position.x
        msg = String()
        msg.data = "point"
        self.pose_pub.publish(msg)
        self.get_logger().info('--- : publishing "%f"' % point)
        self.j += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MovementNode()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
