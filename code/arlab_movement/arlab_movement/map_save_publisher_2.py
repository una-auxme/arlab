#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class MapSavePublisher(Node):
    """Node that publishes map save requests at regular intervals"""
    
    def __init__(self):
        super().__init__('map_save_publisher')
        
        # Declare parameters
        self.declare_parameter('save_interval', 5.0)
        self.declare_parameter('initial_delay', 2.0)
        #self.declare_parameter('auto_start', True)
        
        # Get parameters
        self.save_interval = self.get_parameter('save_interval').value
        self.initial_delay = self.get_parameter('initial_delay').value
        #auto_start = self.get_parameter('auto_start').value
        
        # Create publisher
        self.publisher = self.create_publisher(Bool, 'map_save', 10)
        
        self.get_logger().info(
            f"Map Save Publisher initialized with interval: {self.save_interval}s, "
            f"initial delay: {self.initial_delay}"
        )
        
        # If auto_start is true, set up the timer
        if self.save_interval > 0:
            # First, create a one-shot timer for initial delay
            self.delay_timer = self.create_timer(
                self.initial_delay,
                self.start_periodic_publishing,
                oneshot=True
            )
        else:
            self.get_logger().info("Auto-save not enabled")
    
    def start_periodic_publishing(self):
        """Start the periodic publishing after initial delay"""
        self.get_logger().info("Initial delay completed, starting periodic publishing")
        
        # Create the periodic timer
        self.periodic_timer = self.create_timer(
            self.save_interval,
            self.publish_map_save
        )
        
        # Publish immediately
        self.publish_map_save()
    
    def publish_map_save(self):
        """Publish a map save request"""
        self.get_logger().info("Publishing map save request")
        msg = Bool()
        msg.data = True
        self.publisher.publish(msg)
        self.get_logger().info("Map save request published")


def main(args=None):
    rclpy.init(args=args)
    node = MapSavePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down map_save_publisher")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()