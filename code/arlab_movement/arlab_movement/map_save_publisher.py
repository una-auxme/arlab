import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class MapSaveTrigger(Node):
    def __init__(self):
        super().__init__('map_save_trigger')

        # Declare parameters with default values
        self.declare_parameter('save_interval', 5.0)
        self.declare_parameter('initial_delay', 2.0)

        # Read parameters
        self.publish_interval = (
            self.get_parameter('save_interval').get_parameter_value().double_value
        )
        self.initial_delay = (
            self.get_parameter('initial_delay').get_parameter_value().double_value
        )

        self.get_logger().info(
            f"MapSaveTrigger starting with initial_delay={self.initial_delay}s "
            f"and publish_interval={self.publish_interval}s"
        )

        # Publisher
        self.publisher_ = self.create_publisher(Bool, '/map_save', 10)

        # Initial delay timer -> calls start_publishing() once
        self.initial_timer = self.create_timer(self.initial_delay, self.start_publishing)

        self.publish_timer = None

    def start_publishing(self):
        """Start periodic publishing after the initial delay."""
        self.get_logger().info("Initial delay complete — starting periodic publishing.")

        # Cancel initial delay timer
        self.initial_timer.cancel()

        # Start periodic timer
        self.publish_timer = self.create_timer(
            self.publish_interval, self.timer_callback
        )

    def timer_callback(self):
        msg = Bool()
        msg.data = True
        self.publisher_.publish(msg)
        self.get_logger().debug("Published True to /map_save")


def main(args=None):
    rclpy.init(args=args)
    node = MapSaveTrigger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
