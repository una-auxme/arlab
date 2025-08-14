"""Local safety monitor node for Zirbi system submodules.

Tracks heartbeat messages from local module nodes and sends global heartbeat
messages to the CentralSafetyNode.

TODO:
    - [ ] Implement actual safety checks in `local_safety_checks()`.
    - [ ] Replace magic numbers (-1, 0) with enums
          for readability and precise error tracking.

Maintainers:
    Aleksander Michalak <aleksander.michalak@web.de>
    Daniel Gabler <daniel.gabler@uni-augsburg.de>
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Int32MultiArray


class LocalSafety(Node):
    """ROS2 node for local safety monitoring within a subsystem.

    Attributes:
        module_node_table (dict[int, bool]): Node ID -> alive status.
        module_id (int): ID of the module (e.g., 1=movement, 2=manipulation).
        health_state (int): -1=registering, 0=OK, >0=error code.
        pub_global_heartbeat: Publisher for the `/global_heartbeat` topic.
    """

    def __init__(self) -> None:
        """Initialize node, subscriptions, publishers, and timer."""
        super().__init__(type(self).__name__)

        # Static table: node_id -> is_alive
        self.module_node_table: dict[int, bool] = {}

        # Module 1 = movement, module 2 = manipulation
        self.module_id = 1

        # Timer to reset the node table periodically
        self.timer = self.create_timer(0.1, self.reset_module_node_table)

        self.health_state = -1
        self.pub_global_heartbeat = self.create_publisher(
            Int32MultiArray,
            "/global_heartbeat",
            10,
        )
        self.create_subscription(
            Int32,
            "/local_module_heartbeat",
            self.callback,
            10,
        )

    def pub_module_heartbeat(self) -> None:
        """Publish a global heartbeat to the CentralSafetyNode.

        Notes:
            - If health_state is -1 or 0 → normal operation.
            - Any other value → error code for CentralSafetyNode.
        """
        msg = Int32MultiArray()
        msg.data = [self.module_id, self.health_state]
        self.get_logger().info(f"Publishing heartbeat: {list(msg.data)}")
        self.pub_global_heartbeat.publish(msg)

        # Reset health_state after publishing
        self.health_state = 0

    def callback(self, msg: Int32) -> None:
        """Handle heartbeat messages from local module nodes.

        Args:
            msg: Node ID of the publisher as `std_msgs/Int32`.
        """
        self.get_logger().debug(f"Received local heartbeat: {msg.data}")
        node_id = msg.data
        self.module_node_table[node_id] = True

    def reset_module_node_table(self) -> None:
        """Reset the is_alive state of tracked nodes.

        Notes:
            - Marks non-responsive nodes with their ID in health_state.
            - Sends updated heartbeat to CentralSafetyNode.
        """
        for node_id, is_alive in self.module_node_table.items():
            if not is_alive:
                # Error encoding for CentralSafetyNode
                self.health_state = node_id
            else:
                self.module_node_table[node_id] = False

        self.pub_module_heartbeat()

    def local_safety_checks(self) -> None:
        """Design and implement local safety checks here.

        Notes:
            - This method can set health_state to non-zero values to indicate
              local subsystem errors.
        """


def main(args=None) -> None:
    """Main entry point for LocalSafetyNode."""
    rclpy.init(args=args)

    my_ros2_node = LocalSafety()
    my_ros2_node.pub_module_heartbeat()

    try:
        rclpy.spin(my_ros2_node)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
