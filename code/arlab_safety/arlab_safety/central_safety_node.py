import rclpy
import rospy

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
        #           error_state: INT,
        #           "last_seen": Timestamp,
        #           "heartbeat": FLOAT
        #       }
        # }
        self.module_safety_table = {}

        self.timer = self.create_timer(0.001, self.reset_module_safety_table)
        self.create_subscription(Int32MultiArray, "/global_heartbeat", self.callback, 10)

    def callback(self, msg: Int32MultiArray):
        """Receives messages from the /global_heartbeat

        Args:
            msg (Int32MultiArray): Received an integer array
                with module_id and error_state inside.
        """
        print(f"Received message: {msg.data}")
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
        if not self.module_safety_table.keys().__contains__(module_id):
            print(f"Registering module {module_id} now.")
            self.module_safety_table[module_id]["error_state"] = -1
            self.module_safety_table[module_id]["last_seen"] = rospy.get_time()
            self.module_safety_table[module_id]["heartbeat"] = 0.1
        else:
            print(f"Modul {module_id} is stuck in registering to global health node!")
            self.critical_error(module_id, -1)

    def module_working(self, module_id):
        """Updates working state in the module table."""
        self.module_safety_table[module_id] = 0
        self.module_safety_table[module_id]["last_seen"] = rospy.get_time()

    def critical_error(self, module_id, module_state):
        """Handles the safety guidelines
        for specifc errors states.
        """
        self.module_safety_table[module_id]["error_state"] = module_state
        if module_state == -1:
            pass  # Handle module does not share health state
        else:
            ...
        # Check if system freeze is neccessary, else do nothing

    def reset_module_safety_table(self):
        """Resets the error_state of modules"""
        for module_id, (
            _,
            last_seen,
            heartbeat,
        ) in self.module_safety_table.items():
            if rospy.get_time() - last_seen > heartbeat:
                self.critical_error(module_id, -1)

    def system_freeze(self):
        """In a worst case scenario this function
        will initialize a system freeze.
        """


def main(args=None):
    """Main function"""
    rclpy.init(args=args)

    my_ros2_node = CentralSafetyNode()
    rclpy.spin(my_ros2_node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
