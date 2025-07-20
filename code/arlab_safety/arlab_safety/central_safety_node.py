import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray


class CentralSafetyNode(Node):
    """Cool ros2 template node that publishes stuff to itself ;-)"""

    def __init__(self):
        super().__init__(type(self).__name__)

        # module safety table
        # {
        #   int:module_id:
        #       {
        #           "error_state": INT,
        #           "last_seen": FLOAT (timestamp),
        #           "heartbeat": FLOAT
        #       }
        # }
        self.module_safety_table = {}

        self.timer = self.create_timer(0.1, self.reset_module_safety_table)
        self.create_subscription(
            Int32MultiArray, "/global_heartbeat", self.callback, 10
        )

    def now(self):
        """Returns current time in seconds (float)"""
        return self.get_clock().now().nanoseconds / 1e9

    def callback(self, msg: Int32MultiArray):
        """Receives messages from the /global_heartbeat"""
        print(f"Received message: {msg.data}")
        if len(msg.data) < 2:
            self.get_logger().warn("Invalid message received. Skipping.")
            return

        module_id = msg.data[0]
        module_state = msg.data[1]

        if module_state == -1:
            self.register_module(module_id)
        elif module_state == 0:
            self.module_working(module_id)
        elif module_state > 0:
            self.critical_error(module_id, module_state)

    def register_module(self, module_id):
        """Adds module to its safety list."""
        if module_id not in self.module_safety_table:
            self.get_logger().info(f"Registering module {module_id}.")
            self.module_safety_table[module_id] = {
                "error_state": -1,
                "last_seen": self.now(),
                "heartbeat": 0.5,
            }
        else:
            self.get_logger().warn(f"Module {module_id} is stuck in registering state!")
            self.critical_error(module_id, -1)

    def module_working(self, module_id):
        """Updates working state in the module table."""
        if module_id in self.module_safety_table:
            self.module_safety_table[module_id]["error_state"] = 0
            self.module_safety_table[module_id]["last_seen"] = self.now()
        else:
            self.get_logger().warn(
                f"Received 'working' message from unknown module {module_id}. Registering now."
            )
            self.register_module(module_id)

    def critical_error(self, module_id, module_state):
        """Handles the safety guidelines for specific error states."""
        self.get_logger().error(
            f"Module {module_id} reports critical error state: {module_state}"
        )
        if module_id in self.module_safety_table:
            self.module_safety_table[module_id]["error_state"] = module_state
        else:
            self.register_module(module_id)
            self.module_safety_table[module_id]["error_state"] = module_state

        # Here you could add actions such as triggering alerts or system freeze
        if module_state != -1:
            self.system_freeze()

    def reset_module_safety_table(self):
        """Checks if modules have timed out and triggers error if necessary."""
        now = self.now()
        for module_id, info in self.module_safety_table.items():
            if now - info["last_seen"] > info["heartbeat"]:
                self.get_logger().warn(f"Module {module_id} heartbeat timeout.")
                self.critical_error(module_id, -1)

    def system_freeze(self):
        """In a worst case scenario this function will initialize a system freeze."""
        self.get_logger().fatal("System freeze triggered due to critical error!")


def main(args=None):
    rclpy.init(args=args)

    node = CentralSafetyNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
