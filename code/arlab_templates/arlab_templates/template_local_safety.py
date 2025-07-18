import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String


class LocalSafety(Node):
    """Template node for local safety nodes inside the system submodules"""

    def __init__(self):
        super().__init__(type(self).__name__)

        # Static table of node ids
        # Struktur: { node_id: {"last_seen": ..., "is_alive": ..., "heartbeat": ...} }
        self.module_node_table = {}

        # 0.5s default heartbeat timeout
        self.default_heartbeat = 0.1

        self.timer = self.create_timer(0.1, self.reset_module_node_table)

        # Definition:
        # -1 : Initial state
        #  0: Working
        # Integer > 0 represents node_id of the first Safety Issue
        self.health_state = -1

        # Represents a list of nodes with unsafe behaviour
        self.unhealthy_nodes = []

        self.pub_global_heartbeat = self.create_publisher(
            String, "/global_heartbeat", 10
        )

        self.create_subscription(Int32, "/local_module_heartbeat", self.callback, 10)

    def pub_module_heartbeat(self):
        """Publishes a global heartbeat to the Central Safety node."""
        msg = String()

        # Build string message like: "-1123" (health_state -1, unhealthy_nodes [1, 2, 3])
        msg_content = str(self.health_state)
        for node in self.unhealthy_nodes:
            msg_content += str(node)

        msg.data = msg_content
        self.get_logger().info(f"Publishing global heartbeat: {msg.data}")

        self.pub_global_heartbeat.publish(msg)

        # Reset state after publishing
        self.health_state = 0
        self.unhealthy_nodes.clear()

    def callback(self, msg: Int32):
        """Receives messages from the /local_module_heartbeat"""
        node_id = msg.data
        self.get_logger().info(f"Received heartbeat from node {node_id}")

        self.module_node_table[node_id] = {
            "last_seen": self.get_clock().now(),
            "is_alive": True,
            "heartbeat": self.default_heartbeat,
        }

    def reset_module_node_table(self):
        """Checks timestamps and updates is_alive status based on timeout"""
        now = self.get_clock().now()
        for node_id, info in self.module_node_table.items():
            elapsed = now - info["last_seen"]
            if elapsed > info["heartbeat"]:
                if info["is_alive"]:  # only print once
                    self.get_logger().warn(f"Node {node_id} timed out!")
                info["is_alive"] = False
                if node_id not in self.unhealthy_nodes:
                    self.unhealthy_nodes.append(node_id)
                    self.health_state = node_id  # first failing node
            else:
                info["is_alive"] = True

        self.pub_module_heartbeat()

    def local_safety_checks(self):
        """Design your local safety checks here.
        Implement custom health_state errors if needed.
        """
        pass


def main(args=None):
    """Main function"""
    rclpy.init(args=args)

    node = LocalSafety()

    # Register node initially with state -1
    node.pub_module_heartbeat()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
