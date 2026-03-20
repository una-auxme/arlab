#!/usr/bin/env python3
import os
import signal
import subprocess
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class NavigationStackManager(Node):
    """This Node starts and stops the navigation nodes for localization and mapping according to the movement orchestrator"""

    def __init__(self):
        """Subscribes to the topics that the orchestrator publishes to and creates handles for the subprocesses"""
        super().__init__("navigation_stack_manager")

        self.declare_parameter("map_path", "/workspace/src/arlab/code/arlab_movement/map/my_map")
        self.declare_parameter("use_timestamp", False)

        self.map_path = self.get_parameter("map_path").value
        self.use_timestamp = self.get_parameter("use_timestamp").value

        self.create_subscription(Bool, "localization_bool", self.localization_callback, 10)
        self.create_subscription(Bool, "mapping_bool", self.mapping_callback, 10)
        self.create_subscription(Bool, "nav_bool", self.nav_callback, 10)
        self.create_subscription(Bool, "map_save", self.map_save_callback, 10)

        self.amcl_process = None
        self.slam_process = None
        self.nav_process = None

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
        workspace_setup = "/workspace/install/setup.bash"
        sourced_cmd = ["bash", "-c", f"source {workspace_setup} && {' '.join(cmd)}"]
        process = subprocess.Popen(
            sourced_cmd,
            preexec_fn=os.setsid,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )

        # Read a bit of output after a short delay
        # rclpy.get_default_context().create_timer(1.0, lambda: self.read_process_logs(process, name))
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
                self.amcl_process = self.start_process(["ros2", "run", "nav2_amcl", "amcl"], "AMCL")
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
                    # ["ros2", "run", "slam_toolbox", "sync_slam_toolbox_node"], "SLAM Toolbox" # old start command without using launchfile
                    # ["ros2", "launch", "arlab_movement", "slam_launch.py"], "SLAM Toolbox"
                    # ["ros2", "launch", "slam_toolbox", "online_sync_launch.py", "--ros-args", "-p", "use_pose_graph_slam:=true"], "SLAM Toolbox"
                    ["ros2", "launch", "slam_toolbox", "online_async_launch.py"],
                    "SLAM Toolbox",
                )
            else:
                self.get_logger().info("SLAM already running.")
        else:
            self.slam_process = self.stop_process(self.slam_process, "SLAM Toolbox")

    def nav_callback(self, msg):
        """Starts/stops the Nav2 navigation stack based on /nav_bool messages."""
        if msg.data:
            if self.nav_process is None or self.nav_process.poll() is not None:
                self.nav_process = self.start_process(
                    # ["ros2", "launch", "arlab_movement", "nav_stack_launch.py"], "Nav2 Stack"
                    ["ros2", "run", "nav2_bringup", "navigation_launch.py"],
                    "Nav2 Stack",
                )
            else:
                self.get_logger().info("Nav2 already running.")
        else:
            self.nav_process = self.stop_process(self.nav_process, "Nav2 Stack")

    def map_save_callback(self, msg):
        if msg.data:
            self.get_logger().info("Received map save request")
            self.save_map()
        else:
            self.get_logger().debug("Map save request with False value, ignoring")

    def save_map(self):
        try:
            map_path = self.map_path

            if self.use_timestamp:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                map_path = f"{self.map_path}_{timestamp}"
                self.get_logger().info(f"Using timestamped map name: {map_path}")
            else:
                self.get_logger().info(f"Using fixed map name: {map_path}")

            self.get_logger().info(f"Saving map to: {map_path}")

            cmd = ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", map_path]

            env = os.environ.copy()
            workspace_setup = "/workspace/install/setup.bash"
            sourced_cmd = ["bash", "-c", f"source {workspace_setup} && {' '.join(cmd)}"]

            result = subprocess.run(
                sourced_cmd,
                capture_output=True,
                text=True,
                env=env,
            )

            if result.returncode == 0:
                self.get_logger().info(f"Map saved successfully to {map_path}")
                if result.stdout:
                    self.get_logger().info(f"Map saver output: {result.stdout}")
            else:
                self.get_logger().error(f"Failed to save map. Error: {result.stderr}")

        except Exception as e:
            self.get_logger().error(f"Exception while saving map: {str(e)}")

    def destroy_node(self):
        """cleanup to properly stop all subprocesses"""
        self.amcl_process = self.stop_process(self.amcl_process, "AMCL")
        self.slam_process = self.stop_process(self.slam_process, "SLAM Toolbox")
        self.nav_process = self.stop_process(self.nav_process, "Nav2 Stack")
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


if __name__ == "__main__":
    main()
