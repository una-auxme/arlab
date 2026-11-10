"""Local safety monitor node for Zirbi system submodule movement (navigation).

This node is based on the template in 'arlab_templates/arlab_templates/template_local_safety.py'.

Tracks heartbeat messages from local module nodes and sends global heartbeat
messages to the CentralSafetyNode.

Maintainers:
    Luca Kahlenberg <luca.kahlenberg@uni-a.de>
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Int32MultiArray
from arlab_common_interfaces.msg import SafetyModule, SafetyState


class MovementLocalSafety(Node):
    """ROS2 node for local safety monitoring within the movement module.

    Attributes:
        module_node_table (dict[int, bool]): Node ID -> alive status, see SafetyNode.msg.
            Nodes are added automatically on their first heartbeat.
        module_id (int): ID of the module from SafetyModule.msg.
        health_state (int): SafetyState.REGISTERING, SafetyState.OK or error code > 0.
        local_heartbeat_topic (str): Topic the module nodes send their heartbeats on.
        local_safety_error_topic (str): Topic the module nodes report SafetyState error codes on.
        reported_error (int): Last reported error code, SafetyState.OK if none is pending.
        pub_global_heartbeat: Publisher for the `/global_heartbeat` topic.
    """

    def __init__(self) -> None:
        """Initialize node, subscriptions, publishers, and timer."""
        super().__init__(type(self).__name__)

        # node_id -> is_alive, nodes register automatically with their first heartbeat
        self.module_node_table: dict[int, bool] = {}

        # Module id defined in arlab_common_interfaces SafetyModule.msg
        self.module_id = SafetyModule.MOVEMENT

        # Local heartbeat topic of the movement module
        self.declare_parameter("local_heartbeat_topic", "/arlab/movement/heartbeat")
        self.local_heartbeat_topic = str(self.get_parameter("local_heartbeat_topic").value)

        # Error codes reported by movement nodes, forwarded in local_safety_checks
        self.declare_parameter("local_safety_error_topic", "/arlab/movement/safety_error")
        self.local_safety_error_topic = str(self.get_parameter("local_safety_error_topic").value)
        self.reported_error = SafetyState.OK

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
        self.create_subscription(
            Int32,
            self.local_safety_error_topic,
            self.safety_error_callback,
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
        self.get_logger().debug(f"Publishing heartbeat: {list(msg.data)}")
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

    def safety_error_callback(self, msg: Int32) -> None:
        """Store an error code reported by a movement node.

        Args:
            msg: Error code from the movement range in SafetyState.msg as `std_msgs/Int32`.

        Notes:
            - Only the last code per heartbeat window is kept. Fine while only
              the orchestrator reports, revisit when more nodes report here.
        """
        self.get_logger().warn(f"Received safety error: {msg.data}")
        self.reported_error = msg.data

    def reset_module_node_table(self) -> None:
        """Reset the is_alive state of tracked nodes.

        Notes:
            - Marks non-responsive nodes with their ID in health_state.
              The node ID is the error code for a node timeout, see SafetyNode.msg.
            - Runs local_safety_checks before sending.
            - Sends updated heartbeat to CentralSafetyNode.
        """
        for node_id, is_alive in self.module_node_table.items():
            if not is_alive:
                self.health_state = node_id
            else:
                self.module_node_table[node_id] = False

        self.local_safety_checks()
        self.pub_module_heartbeat()

    def local_safety_checks(self) -> None:
        """Forward error codes reported on the local safety error topic.

        Notes:
            - Sets health_state to the reported error code from the movement
              range in SafetyState.msg (MOVEMENT_*), sent once with the next heartbeat.
        """
        if self.reported_error != SafetyState.OK:
            self.health_state = self.reported_error
            self.reported_error = SafetyState.OK


def main(args=None) -> None:
    """Main entry point for MovementLocalSafety."""
    rclpy.init(args=args)

    my_ros2_node = MovementLocalSafety()
    my_ros2_node.pub_module_heartbeat()

    try:
        rclpy.spin(my_ros2_node)
    finally:
        my_ros2_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
