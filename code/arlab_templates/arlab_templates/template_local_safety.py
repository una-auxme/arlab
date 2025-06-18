import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import String


class LocalSafety(Node):
    """Template node for local safety nodes inside the system submodules"""

    def __init__(self):
        super().__init__(type(self).__name__)

        # Static table of node ids
        # {int:nodeID: bool:is_alive}
        self.module_node_table = {}
        self.timer = self.create_timer(1.0, self.reset_module_node_table)

        self.pub_global_heartbeat = self.create_publisher(String, "/global_heartbeat", 10)
        self.create_subscription(Int32, "/<local_module>_heartbeat", self.callback, 10)

    def pub_module_heartbeat(self, msg: str):
        """Publishes a global heartbeat to the Central Safety node

        Args:
            msg (str): The message to publish
        """
        ros_msg = String(data=msg)
        self.pub_global_heartbeat.publish(ros_msg)

    def callback(self, msg: Int32):
        """Receives messages from the /<local_module>_heartbeat

        Args:
            msg (Int32): Node ID of the publisher
        """

        print(f"Received message: {msg.data}")
        node_id = msg.data
        self.module_node_table[node_id] = True

        # Publisher for testing logic
        self.pub_global_heartbeat.publish(msg)

    def reset_module_node_table(self):
        """Resets the is_alive state of module nodes
        """
        for key in self.module_node_table:
            self.module_node_table[key] = False

    def local_safety_checks(self):
        """Design your local safety checks here.
        """

def main(args=None):
    """Main function
    """
    rclpy.init(args=args)

    my_ros2_node = LocalSafety()
    my_ros2_node.pub_module_heartbeat("OK")

    rclpy.spin(my_ros2_node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
