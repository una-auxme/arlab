#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from builtin_interfaces.msg import Time

class TwistRelay(Node):
    def __init__(self):
        super().__init__('twist_to_twiststamped')

        # Subscriber: Nav2 publishes Twist on /cmd_vel_twist
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',  
            self.twist_callback,
            10
        )

        # Publisher: Robot controller expects TwistStamped on /cmd_vel
        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)

    def twist_callback(self, msg: Twist):
        ts_msg = TwistStamped()
        ts_msg.header.stamp = self.get_clock().now().to_msg()
        ts_msg.header.frame_id = 'base_link'  
        ts_msg.twist = msg
        self.publisher.publish(ts_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TwistRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
