"""Local safety monitor node for Zirbi system submodules.

Tracks heartbeat messages from local module nodes and sends global heartbeat
messages to the CentralSafetyNode.

TODO:
    - [ ] Implement actual safety checks in `local_safety_checks()`.

Maintainers:
    Aleksander Michalak <aleksander.michalak@web.de>
    Daniel Gabler <daniel.gabler@uni-a.de>
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Int32MultiArray
from arlab_common_interfaces.msg import SafetyState, SafetyModule


class LocalSafety(Node):
    """ROS2 node for local safety monitoring within a subsystem.

    Attributes:
        module_node_table (dict[int, bool]): Node ID -> alive status, see SafetyNode.msg.
            Nodes are added automatically on their first heartbeat.
        module_id (int): ID of the module from SafetyModule.msg.
        health_state (int): SafetyState.REGISTERING, SafetyState.OK or error code > 0.
        local_heartbeat_topic (str): Topic the module nodes send their heartbeats on.
        pub_global_heartbeat: Publisher for the `/global_heartbeat` topic.
    """

    def __init__(self) -> None:
        """Initialize node, subscriptions, publishers, and timer."""
        super().__init__(type(self).__name__)

        # node_id -> is_alive, nodes register automatically with their first heartbeat
        self.module_node_table: dict[int, bool] = {}

        # Module id defined in arlab_common_interfaces SafetyModule.msg
        self.module_id = SafetyModule.MODULE_ID  # Replace with actual module ID from SafetyModule.msg

        # Local heartbeat topic of the module, e.g., /arlab/movement/heartbeat for movement module
        self.declare_parameter("local_heartbeat_topic", "/arlab/module_name/heartbeat")  # Replace module_name with actual module
        self.local_heartbeat_topic = str(self.get_parameter("local_heartbeat_topic").value)

        # Timer to reset the node table periodically
        self.timer = self.create_timer(0.1, self.reset_module_node_table)

        self.health_state = SafetyState.REGISTERING
        self.pub_global_heartbeat = self.create_publisher(
            Int32MultiArray,
            "/global_heartbeat",
            10,
        )
        self.create_subscription(
            Int32,
            self.local_heartbeat_topic,
            self.callback,
            10,
        )

    def pub_module_heartbeat(self) -> None:
        """Publish a global heartbeat to the CentralSafetyNode.

        Notes:
            - health_state is SafetyState.REGISTERING for initial registration.
            - health_state is SafetyState.OK → normal operation.
            - Any other value → error code for CentralSafetyNode.
            - Constants are defined in arlab_common_interfaces.
        """
        msg = Int32MultiArray()
        msg.data = [self.module_id, self.health_state]
        self.get_logger().info(f"Publishing heartbeat: {list(msg.data)}")
        self.pub_global_heartbeat.publish(msg)

        # Reset health_state after publishing
        self.health_state = SafetyState.OK

    def callback(self, msg: Int32) -> None:
        """Handle heartbeat messages from local module nodes.

        Args:
            msg: Node ID of the publisher from SafetyNode.msg as `std_msgs/Int32`.

        Notes:
            - Unknown node IDs are added to module_node_table.
        """
        self.get_logger().debug(f"Received local heartbeat: {msg.data}")
        node_id = msg.data
        self.module_node_table[node_id] = True

    def reset_module_node_table(self) -> None:
        """Reset the is_alive state of tracked nodes.

        Notes:
            - Marks non-responsive nodes with their ID in health_state.
              The node ID is the error code for a node timeout, see SafetyNode.msg.
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
            - This method can set health_state to error codes from the module's
              range in SafetyState.msg to indicate local subsystem errors.
        """


def main(args=None) -> None:
    """Main entry point for LocalSafety."""
    rclpy.init(args=args)

    my_ros2_node = LocalSafety()
    my_ros2_node.pub_module_heartbeat()

    try:
        rclpy.spin(my_ros2_node)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
