import rclpy
from rclpy.node import Node


class MyRos2Node(Node):
    """Cool ros2 template node ;-)"""

    def __init__(self):
        super().__init__(type(self).__name__)


def main(args=None):
    rclpy.init(args=args)

    my_ros2_node = MyRos2Node()

    rclpy.spin(my_ros2_node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
