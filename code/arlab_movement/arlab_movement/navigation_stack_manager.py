#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import subprocess
import signal
import os


class NavigationStackManager(Node):
    """This Node starts and stops the navigation nodes for localization and mapping according to the movement orchestrator"""

    def __init__(self):
        """Subscribes to the topics that the orchestrator publishes to and creates handles for the subprocesses
        """
        super().__init__('navigation_stack_manager')

        self.create_subscription(Bool, 'localization_bool', self.localization_callback, 10)
        self.create_subscription(Bool, 'mapping_bool', self.mapping_callback, 10)

        self.amcl_process = None
        self.slam_process = None

        self.get_logger().info("Navigation Stack Manager 2 (subprocess mode) started.")

    def start_process(self, cmd, name):
        """Starts a subprocess through a given command

        Args:
            cmd (list): a list of strings
            name (str): subprocess label for logging

        Returns:
            Popen: handle that represents the subprocess
        """
        self.get_logger().info(f"Starting {name} with command: {' '.join(cmd)}")
        env = os.environ.copy()
        process = subprocess.Popen(
            cmd,
            preexec_fn=os.setsid,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )

        # Read a bit of output after a short delay
        #rclpy.get_default_context().create_timer(1.0, lambda: self.read_process_logs(process, name))
        return process

    def read_process_logs(self, process, name):
        if process.poll() is not None:  # process exited
            out, err = process.communicate()
            if out:
                self.get_logger().info(f"{name} stdout:\n{out}")
            if err:
                self.get_logger().error(f"{name} stderr:\n{err}")

    def stop_process(self, process, name):
        """Stops a subprocess

        Args:
            process (Popen): subprocess handle
            name (str): subprocess label for logging
        """
        if process is not None:
            self.get_logger().info(f"Stopping {name}...")
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGINT)
            except ProcessLookupError:
                self.get_logger().warn(f"{name} was not running anymore.")
        else:
            self.get_logger().info(f"{name} not running.")
        return None

    def localization_callback(self, msg):
        """Gets called when a bool message arrives on topic localization_bool. Starts localization if true, otherwise shuts localization down

        Args:
            msg (Bool): message from topic
        """
        if msg.data:
            if self.amcl_process is None:
                self.amcl_process = self.start_process(
                    ["ros2", "run", "nav2_amcl", "amcl"], "AMCL"
                )
            else:
                self.get_logger().info("AMCL already running.")
        else:
            self.amcl_process = self.stop_process(self.amcl_process, "AMCL")

    def mapping_callback(self, msg):
        """Gets called when a bool message arrives on topic mapping_bool. Starts mapping if true, otherwise shuts mapping down

        Args:
            msg (Bool): message from topic
        """
        if msg.data:
            if self.slam_process is None or self.slam_process.poll() is not None:
                self.slam_process = self.start_process(
                    #["ros2", "run", "slam_toolbox", "sync_slam_toolbox_node"], "SLAM Toolbox" # old start command without using launchfile
                    ["ros2", "launch", "arlab_movement", "slam_launch.py"], "SLAM Toolbox"
                )
            else:
                self.get_logger().info("SLAM already running.")
        else:
            self.slam_process = self.stop_process(self.slam_process, "SLAM Toolbox")

    def destroy_node(self):
        """cleanup to properly stop all subprocesses
        """
        self.amcl_process = self.stop_process(self.amcl_process, "AMCL")
        self.slam_process = self.stop_process(self.slam_process, "SLAM Toolbox")
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
