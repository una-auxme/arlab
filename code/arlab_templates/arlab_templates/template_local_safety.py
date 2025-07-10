import rclpy
import rospy

from rclpy.node import Node
from std_msgs.msg import Int32


class LocalSafety(Node):
    """Template node for local safety nodes inside the system submodules"""

    def __init__(self):
        super().__init__(type(self).__name__)

        # Static table of node ids
        # Struktur: { node_id: {"last_seen": ..., "is_alive": ..., "heartbeat": ...} }
        self.module_node_table = {}
        self.timer = self.create_timer(0.001, self.reset_module_node_table)
        self.health_state = -1
        self.unhealthy_nodes = []
        self.pub_global_heartbeat = self.create_publisher(
            Int32, "/global_heartbeat", 10
        )

        self.create_subscription(Int32, "/local_module_heartbeat", self.callback, 10)

    def pub_module_heartbeat(self):
        """Publishes a global heartbeat to the Central Safety node
            If heartbeat is not -1 or 0 the Central Safety Node
            will receive an error code.
        Args:
            msg(Int32) = Encoded module health_state_error
        """
        msg = Int32()
        msg.data = str(self.health_state)
        for node in self.unhealthy_nodes:
            msg.data += str(node)
        self.pub_global_heartbeat.publish(msg)
        self.health_state = 0

    def callback(self, msg: Int32):
        """Receives messages from the /local_module_heartbeat

        Args:
            msg (Int32): Node ID of the publisher
        """
        print(f"Received message: {msg.data}")
        node_id = msg.data
        self.module_node_table[node_id] = {
            "last_seen": rospy.time(),  # oder rospy.get_time()
            "is_alive": True,
        }

    def reset_module_node_table(self):
        """Checks timestamps and updates is_alive status based on timeout"""
        for node_id, info in self.module_node_table.items():
            if rospy.get_time() - info["last_seen"] > info["heartbeat"]:
                info["is_alive"] = False
                self.health_state = node_id
                self.unhealthy_nodes.append(node_id)
            else:
                info["is_alive"] = True
        self.pub_module_heartbeat()

    def local_safety_checks(self):
        """Design your local safety checks here.
        Implement heath_state_errors
        """


def main(args=None):
    """Main function
    Publishes module health_state with initial value -1
    to register node in Central Safety Node
    """
    rclpy.init(args=args)

    my_ros2_node = LocalSafety()
    my_ros2_node.pub_module_heartbeat()

    rclpy.spin(my_ros2_node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
