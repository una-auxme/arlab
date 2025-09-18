#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import subprocess
import signal
import os


class NavigationStackManager(Node):
    def __init__(self):
        super().__init__('navigation_stack_manager')

        # Subscribers
        self.create_subscription(Bool, 'localization_bool', self.localization_cb, 10)
        self.create_subscription(Bool, 'mapping_bool', self.mapping_cb, 10)

        # Handles for subprocesses
        self.amcl_process = None
        self.slam_process = None

        self.get_logger().info("Navigation Stack Manager (subprocess mode) started.")

    def localization_cb(self, msg: Bool):
        if msg.data:  # Start AMCL
            if self.amcl_process is None:
                self.get_logger().info("Starting AMCL...")
                self.amcl_process = subprocess.Popen(
                    ["ros2", "run", "nav2_amcl", "amcl"]
                )
            else:
                self.get_logger().info("AMCL already running.")
        else:  # Stop AMCL
            if self.amcl_process is not None:
                self.get_logger().info("Stopping AMCL...")
                self.amcl_process.send_signal(signal.SIGINT)
                self.amcl_process = None
            else:
                self.get_logger().info("AMCL not running.")

    def mapping_cb(self, msg: Bool):
        if msg.data:  # Start SLAM
            if self.slam_process is None:
                self.get_logger().info("Starting SLAM Toolbox...")
                self.slam_process = subprocess.Popen(
                    ["ros2", "run", "slam_toolbox", "sync_slam_toolbox_node"]
                )
            else:
                self.get_logger().info("SLAM already running.")
        else:  # Stop SLAM
            if self.slam_process is not None:
                self.get_logger().info("Stopping SLAM Toolbox...")
                self.slam_process.send_signal(signal.SIGINT)
                self.slam_process = None
            else:
                self.get_logger().info("SLAM not running.")

    def destroy_node(self):
        # Cleanup processes if still running
        if self.amcl_process is not None:
            self.get_logger().info("Cleaning up AMCL process...")
            self.amcl_process.send_signal(signal.SIGINT)
        if self.slam_process is not None:
            self.get_logger().info("Cleaning up SLAM process...")
            self.slam_process.send_signal(signal.SIGINT)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NavigationStackManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
