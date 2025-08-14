"""Central safety monitor node for ROS2 modules.

This node listens for heartbeat messages from modules on the `/global_heartbeat`
topic, maintains a safety status table, and can trigger system freezes when
critical errors or timeouts occur.

Maintainers:
    Aleksander Michalak <aleksander.michalak@web.de>
    Daniel Gabler <daniel.gabler@uni-augsburg.de>
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray


class CentralSafetyNode(Node):
    """ROS2 node for monitoring and enforcing module safety.

    Attributes:
        module_safety_table (dict[int, dict]): Stores module safety status, with:
            - error_state (int): -1=registering, 0=OK, >0=critical.
            - last_seen (float): Time in seconds when last seen.
            - heartbeat (float): Max allowed time between heartbeats.
        timer: ROS timer for periodic timeout checks.
    """

    def __init__(self) -> None:
        """Initialize node, subscriptions, and timer."""
        super().__init__(type(self).__name__)
        self.module_safety_table: dict[int, dict] = {}

        # Timer to periodically check heartbeat timeouts.
        self.timer = self.create_timer(0.1, self.reset_module_safety_table)

        # Subscription to heartbeat messages from all modules.
        self.create_subscription(
            Int32MultiArray,
            "/global_heartbeat",
            self.callback,
            10,
        )

    def now(self) -> float:
        """Return current time in seconds.

        Returns:
            float: Current ROS clock time in seconds.
        """
        return self.get_clock().now().nanoseconds / 1e9

    def callback(self, msg: Int32MultiArray) -> None:
        """Process incoming heartbeat messages.

        Message format: [module_id, module_state]

        Args:
            msg: `std_msgs/Int32MultiArray` containing heartbeat info.

        Notes:
            - module_state == -1 → registering state.
            - module_state == 0 → working/OK.
            - module_state > 0 → critical error code.
        """
        self.get_logger().debug(f"Received heartbeat: {list(msg.data)}")

        if len(msg.data) < 2:
            self.get_logger().warn("Invalid heartbeat message. Skipping.")
            return

        module_id = msg.data[0]
        module_state = msg.data[1]

        if module_state == -1:
            self.register_module(module_id)
        elif module_state == 0:
            self.module_working(module_id)
        elif module_state > 0:
            self.critical_error(module_id, module_state)

    def register_module(self, module_id: int) -> None:
        """Register a new module in the safety table.

        Args:
            module_id: Unique ID of the module to register.

        Notes:
            Initializes the module with error_state=-1 (registering). If
            already present in registering state, escalates to critical error.
        """
        if module_id not in self.module_safety_table:
            self.get_logger().info(f"Registering module {module_id}.")
            self.module_safety_table[module_id] = {
                "error_state": -1,
                "last_seen": self.now(),
                "heartbeat": 0.5,
            }
        else:
            self.get_logger().warn(f"Module {module_id} stuck in registering state.")
            self.critical_error(module_id, -1)

    def module_working(self, module_id: int) -> None:
        """Mark a module as working and update last seen time.

        Args:
            module_id: Unique ID of the module.
        """
        if module_id in self.module_safety_table:
            self.module_safety_table[module_id]["error_state"] = 0
            self.module_safety_table[module_id]["last_seen"] = self.now()
        else:
            self.get_logger().warn(
                f"Received 'working' from unknown module {module_id}. Registering now."
            )
            self.register_module(module_id)

    def critical_error(self, module_id: int, module_state: int) -> None:
        """Handle critical error states for modules.

        Args:
            module_id: Unique ID of the module.
            module_state: Positive integer error code (>0 = critical error).

        Notes:
            Updates the module's error_state in the table. Triggers a system
            freeze if the error state is not 0.
        """
        self.get_logger().error(
            f"Module {module_id} reports critical error: {module_state}"
        )

        if module_id not in self.module_safety_table:
            self.register_module(module_id)

        self.module_safety_table[module_id]["error_state"] = module_state

        if module_state != 0:
            self.system_freeze()

    def reset_module_safety_table(self) -> None:
        """Check heartbeat timeouts and escalate to error if necessary.

        Notes:
            Called periodically by the internal ROS timer. Marks modules as
            critical error (-1) if last seen exceeds heartbeat interval.
        """
        now = self.now()
        for module_id, info in self.module_safety_table.items():
            if now - info["last_seen"] > info["heartbeat"]:
                self.get_logger().warn(f"Module {module_id} heartbeat timeout.")
                self.critical_error(module_id, -1)

    def system_freeze(self) -> None:
        """Trigger a system freeze due to unrecoverable critical errors.

        Notes:
            Intended to be the final safety action (e.g., emergency stop).
        """
        self.get_logger().fatal("System freeze triggered due to critical error!")


def main(args=None) -> None:
    """Entry point for the CentralSafetyNode.

    Args:
        args: Optional CLI args forwarded to rclpy.init.
    """
    rclpy.init(args=args)
    node = CentralSafetyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
