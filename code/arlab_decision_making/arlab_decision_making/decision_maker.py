import rclpy
from rclpy.node import Node


class DecisionMaker(Node):
    """Global decision maker"""

    def __init__(self):
        super().__init__(type(self).__name__)


def main(args=None):
    rclpy.init(args=args)

    decision_maker_node = DecisionMaker()

    rclpy.spin(decision_maker_node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
