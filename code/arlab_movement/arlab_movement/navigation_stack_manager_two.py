#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import subprocess
import signal
import os


class NavigationStackManagerTwo(Node):
    def __init__(self):
        super().__init__('navigation_stack_manager_two')

        # Subscribers
        self.create_subscription(Bool, 'localization_bool', self.localization_cb, 10)
        self.create_subscription(Bool, 'mapping_bool', self.mapping_cb, 10)

        # Handles for subprocesses
        self.amcl_process = None
        self.slam_process = None

        self.get_logger().info("Navigation Stack Manager 2 (subprocess mode) started.")

    def start_process(self, cmd, name):
        """Start a process in its own process group."""
        self.get_logger().info(f"Starting {name} with command: {' '.join(cmd)}")
        return subprocess.Popen(cmd, preexec_fn=os.setsid)

    def stop_process(self, process, name):
        """Stop a whole process group cleanly."""
        if process is not None:
            self.get_logger().info(f"Stopping {name}...")
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGINT)
            except ProcessLookupError:
                self.get_logger().warn(f"{name} was not running anymore.")
        else:
            self.get_logger().info(f"{name} not running.")
        return None

    def localization_cb(self, msg: Bool):
        if msg.data:  # Start AMCL
            if self.amcl_process is None:
                self.amcl_process = self.start_process(
                    ["ros2", "run", "nav2_amcl", "amcl"], "AMCL"
                )
            else:
                self.get_logger().info("AMCL already running.")
        else:  # Stop AMCL
            self.amcl_process = self.stop_process(self.amcl_process, "AMCL")

    def mapping_cb(self, msg: Bool):
        if msg.data:  # Start SLAM
            if self.slam_process is None:
                # Using launch file is better if you have configs
                self.slam_process = self.start_process(
                    ["ros2", "run", "slam_toolbox", "sync_slam_toolbox_node"], "SLAM Toolbox"
                )
            else:
                self.get_logger().info("SLAM already running.")
        else:  # Stop SLAM
            self.slam_process = self.stop_process(self.slam_process, "SLAM Toolbox")

    def destroy_node(self):
        # Cleanup processes if still running
        self.amcl_process = self.stop_process(self.amcl_process, "AMCL")
        self.slam_process = self.stop_process(self.slam_process, "SLAM Toolbox")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NavigationStackManagerTwo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
