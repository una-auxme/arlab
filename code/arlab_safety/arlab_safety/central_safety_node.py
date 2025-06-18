import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray


class CentralSafetyNode(Node):
    """Cool ros2 template node that publishes stuff to itself ;-)"""

    def __init__(self):
        super().__init__(type(self).__name__)
        self.create_subscription(Int32MultiArray, "/global_heartbeat", self.cool_callback, 10)

    def cool_callback(self, msg: Int32MultiArray):
        """Receives messages from the /cool_topic

        Args:
            msg (String): Received cool message
        """
        print(f"Received message: {msg.data}")
        if msg.data[1] == -1:
            self.register_node()
        if msg.data[1] > 0:
            self.critical_error()

    def register_node(self):
        print("register now")
        pass

    def critical_error(self):
        pass


def main(args=None):
    rclpy.init(args=args)

    my_ros2_node = CentralSafetyNode()
    rclpy.spin(my_ros2_node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
