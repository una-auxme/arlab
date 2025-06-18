import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MyRos2PubSub(Node):
    """Cool ros2 template node that publishes stuff to itself ;-)"""

    def __init__(self):
        super().__init__(type(self).__name__)

        self.cool_pub = self.create_publisher(String, "/cool_topic", 10)
        self.create_subscription(String, "/cool_topic", self.cool_callback, 10)

    def pub_cool_msg(self, msg: str):
        """Publishes a very cool message

        Args:
            msg (str): The message to publish
        """
        ros_msg = String(data=msg)
        self.cool_pub.publish(ros_msg)

    def cool_callback(self, msg: String):
        """Receives messages from the /cool_topic

        Args:
            msg (String): Received cool message
        """
        print(f"Received message: {msg.data}")

        # Publishing more stuff is allowed here, because
        # publishing does not block the ros executor
        self.cool_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    my_ros2_node = MyRos2PubSub()
    my_ros2_node.pub_cool_msg("Coooool")

    rclpy.spin(my_ros2_node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
