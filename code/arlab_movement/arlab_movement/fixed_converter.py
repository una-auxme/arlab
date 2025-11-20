#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class FixedConverter(Node):
    def __init__(self):
        super().__init__('fixed_converter')
        
        # Use absolute topic names with leading slash
        input_topic = '/cmd_vel'
        output_topic = '/cmd_vel'
        
        self.get_logger().info(f"Subscribing to: {input_topic}")
        self.get_logger().info(f"Publishing to: {output_topic}")
        
        self.subscription = self.create_subscription(
            Twist,
            input_topic,
            self.twist_callback,
            10)
        
        self.publisher = self.create_publisher(
            TwistStamped,
            output_topic,
            10)
        
        self.get_logger().info('Fixed converter started successfully!')
        
    def twist_callback(self, msg):
        self.get_logger().info('Received Twist, converting to TwistStamped')
        stamped_msg = TwistStamped()
        stamped_msg.header.stamp = self.get_clock().now().to_msg()
        stamped_msg.header.frame_id = 'base_link'
        stamped_msg.twist = msg
        
        self.publisher.publish(stamped_msg)
        self.get_logger().info('Published TwistStamped')

def main():
    rclpy.init()
    node = FixedConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()