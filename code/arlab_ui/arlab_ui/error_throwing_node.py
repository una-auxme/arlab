"""error_throwing_node.py

This node emits an error log message for a specified duration.
It is used for debugging and testing the UI.

Author: Jonas Platzer
"""

import time

import rclpy
from rclpy.node import Node


class ErrorTest(Node):
    def __init__(self):
        super().__init__("error_test")
        self.timer = self.create_timer(0.5, self._tick)
        self.count = 0

    def _tick(self):
        self.count += 1
        self.get_logger().error(f"TEST ERROR {self.count}: hello from error_test")


def main():
    rclpy.init()
    node = ErrorTest()

    t_end = time.time() + 5.0
    while rclpy.ok() and time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
