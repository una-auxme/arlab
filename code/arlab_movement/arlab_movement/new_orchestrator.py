#!/usr/bin/env python3
"""movement_orchestrator.py

This node implements an action-based orchestrator for managing navigation-related processes.
It provides functionality to start/stop localization (AMCL) [currently not used], mapping (SLAM Toolbox),
and navigation (Nav2), as well as saving the current map to file and optionally to a knowledge database.

Author: Jonas Platzer

"""

import os
import signal
import subprocess
import threading
from datetime import datetime
import time
from typing import Optional, Tuple

import rclpy
import rclpy.executors
from arlab_common_interfaces.action import MovementAction
from arlab_knowledge_interfaces.srv import AddMap
from nav_msgs.msg import OccupancyGrid
from rclpy.action.server import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import Bool


class NavErr:
    OK = 1
    UNDEFINED = 0
    BAD_COMMAND = -10
    PROCESS_START_FAILED = -20
    PROCESS_STOP_FAILED = -21
    MAP_SAVE_FAILED = -30
    DB_SERVICE_UNAVAILABLE = -31
    DB_SAVE_FAILED = -32


class NavigationOrchestrator(Node):
    """Action-based orchestrator for navigation stack processes and map saving."""

    def __init__(self):
        super().__init__("navigation_orchestrator")

        # Match manipulation orchestrator pattern: isolate services
        self.service_group = MutuallyExclusiveCallbackGroup()
        self.action_group = ReentrantCallbackGroup()

        # Initialize parameters set in ../params/arlab_navigation_params.yaml
        self.declare_parameter(
            "map_path", "/workspace/src/arlab/code/arlab_movement/map/my_map"
        )
        self.declare_parameter("use_timestamp", False)
        self.declare_parameter("save_to_database", False)
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("database_service", "/arlab/knowledge/add_map")
        self.declare_parameter("database_timeout", 10.0)

        # old topic API
        self.declare_parameter("enable_legacy_topics", False)

        self.map_path = str(self.get_parameter("map_path").value)
        self.use_timestamp = bool(self.get_parameter("use_timestamp").value)
        self.save_to_database = bool(self.get_parameter("save_to_database").value)
        self.map_topic = str(self.get_parameter("map_topic").value)
        self.database_service = str(self.get_parameter("database_service").value)
        self.database_timeout = float(self.get_parameter("database_timeout").value)
        self.enable_legacy_topics = bool(
            self.get_parameter("enable_legacy_topics").value
        )

        # initialize states
        self.current_map: Optional[OccupancyGrid] = None
        self.amcl_process: Optional[subprocess.Popen] = None
        self.slam_process: Optional[subprocess.Popen] = None
        self.nav_process: Optional[subprocess.Popen] = None

        # subscribe to the map topic to cache the latest map for saving
        self.map_subscription = self.create_subscription(
            OccupancyGrid, self.map_topic, self.map_callback, 10
        )

        # initialize database client
        self.database_client = None
        if self.save_to_database:
            self.database_client = self.create_client(
                AddMap, self.database_service, callback_group=self.service_group
            )
            self.get_logger().info(f"Database client created: {self.database_service}")

        # action server for movement commands
        self._action_server = ActionServer(
            self,
            MovementAction,
            "/movement/action",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.action_group,
        )

        # legacy topic interface (subscription based control)
        if self.enable_legacy_topics:
            self.create_subscription(
                Bool, "localization_bool", self.legacy_localization_callback, 10
            )
            self.create_subscription(
                Bool, "mapping_bool", self.legacy_mapping_callback, 10
            )
            self.create_subscription(Bool, "nav_bool", self.legacy_nav_callback, 10)
            self.create_subscription(
                Bool, "map_save", self.legacy_map_save_callback, 10
            )

        self.get_logger().info("NavigationOrchestrator ActionServer started.")
        self.get_logger().info("Action name: /movement/action")

    # Map subscription
    def map_callback(self, msg: OccupancyGrid):
        """
        Cache the latest occupancy grid map.

        This callback stores the most recent /map message so that it can be
        saved to file or sent to the knowledge database on a map_save command.

        Args:
            msg (OccupancyGrid): The current map published by SLAM or Nav2.
        """
        self.current_map = msg
        self.get_logger().debug("Map updated.")

    # Action callbacks
    def goal_callback(self, goal_request: MovementAction.Goal):
        """
        Validate and accept incoming MovementAction goals.

        This method is called when a new action goal is received.
        Currently, all goals are accepted and validated later during execution.

        Args:
            goal_request (MovementAction.Goal): The requested action goal.

        Returns:
            GoalResponse: ACCEPT to allow execution of the goal.
        """
        self.get_logger().info(
            f"Goal: cmd='{goal_request.cmd}', enable={goal_request.enable}"
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """
        Handle cancellation requests for an active action goal.

        When a cancel request is received, all navigation-related processes
        (AMCL, SLAM Toolbox, Nav2) are stopped as a safety measure.

        Args:
            goal_handle: The active action goal handle.

        Returns:
            CancelResponse: ACCEPT to confirm cancellation.
        """
        self.get_logger().warning(
            "Cancel requested; stopping all navigation processes."
        )
        self.stop_all()
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """
        Execute a MovementAction goal.

        Sends the requested command to the appropriate handler:
          - start/stop localization, mapping, or navigation
          - save the map
          - stop all processes

        Publishes status feedback during execution and returns a structured
        result with an error code, message, and success flag.

        Args:
            goal_handle: The active action goal handle.

        Returns:
            MovementAction.Result: Result containing error_code, message, and success.
        """
        cmd = (goal_handle.request.cmd or "").strip().lower()
        enable = bool(goal_handle.request.enable)

        feedback = MovementAction.Feedback()

        def publish_status(text: str):
            feedback.status = text
            goal_handle.publish_feedback(feedback)

        publish_status(f"Executing cmd='{cmd}' enable={enable}")

        err = NavErr.UNDEFINED
        msg = "UNDEFINED"

        try:
            if cmd == "localization":
                err, msg = self.set_localization(enable, publish_status)
            elif cmd == "mapping":
                err, msg = self.set_mapping(enable, publish_status)
            elif cmd == "nav":
                err, msg = self.set_nav(enable, publish_status)
            elif cmd == "map_save":
                err, msg = self.save_map(publish_status)
            elif cmd == "stop_all":
                publish_status("Stopping all navigation processes")
                self.stop_all()
                err, msg = NavErr.OK, "Stopped all navigation processes"
            else:
                err, msg = NavErr.BAD_COMMAND, f"Unknown cmd '{cmd}'."
        except Exception as e:
            err, msg = NavErr.UNDEFINED, f"Exception while executing '{cmd}': {e}"

        result = MovementAction.Result()
        result.error_code = int(err)
        result.message = str(msg)
        result.success = bool(err == NavErr.OK)

        publish_status(f"Done: success={result.success} err={result.error_code}")

        if goal_handle.is_active:
            goal_handle.succeed()

        return result

    # Process helpers
    def _start_process(
        self, cmd_list, name: str
    ) -> Tuple[Optional[subprocess.Popen], int, str]:
        """
        Start a navigation-related subprocess.

        Executes the given command in a new process group.

        Args:
            cmd_list (list[str]): Command and arguments to execute.
            name (str): Human-readable process name for logging.

        Returns:
            Tuple:
                - subprocess.Popen or None: Handle to the started process
                - int: Error code (NavErr.OK on success)
                - str: Status or error message
        """
        self.get_logger().info(f"Starting {name}: {' '.join(cmd_list)}")
        try:
            env = os.environ.copy()
            workspace_setup = "/workspace/install/setup.bash"
            sourced_cmd = [
                "bash",
                "-c",
                f"source {workspace_setup} && {' '.join(cmd_list)}",
            ]

            proc = subprocess.Popen(
                sourced_cmd,
                preexec_fn=os.setsid,
                env=env,
                stdout=None,  # no stdout, otherwise processes might hang if buffer fills up
                stderr=None,
                text=True,
            )

            # giving the process time to start and check if it exited immediately
            time.sleep(2.0)

            if proc.poll() is not None:
                return None, NavErr.PROCESS_START_FAILED, f"{name} exited immediately"

            return proc, NavErr.OK, f"{name} started"
        except Exception as e:
            return None, NavErr.PROCESS_START_FAILED, f"Failed to start {name}: {e}"

    def _stop_process(
        self, proc: Optional[subprocess.Popen], name: str
    ) -> Tuple[Optional[subprocess.Popen], int, str]:
        """
        Stop a running navigation-related subprocess.

        Sends SIGINT to the process group of the subprocess to allow
        clean shutdown.

        Args:
            proc (subprocess.Popen or None): The process to stop.
            name (str): Human-readable process name for logging.

        Returns:
            Tuple:
                - None or subprocess.Popen: None if stopped, original handle if failed
                - int: Error code (NavErr.OK on success)
                - str: Status or error message
        """
        if proc is None:
            return None, NavErr.OK, f"{name} not running"
        self.get_logger().info(f"Stopping {name}...")
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            return None, NavErr.OK, f"{name} stopped"
        except Exception as e:
            return proc, NavErr.PROCESS_STOP_FAILED, f"Failed to stop {name}: {e}"

    # Commands
    def set_localization(self, enable: bool, publish_status) -> Tuple[int, str]:
        """
        CURRENTLY NOT USED. AMCL NODE GETS STARTED BUT CANT BE UTILIZED BY THE TURTLEBOT.

        Start or stop localization (AMCL).

        Args:
            enable (bool): True to start AMCL, False to stop it.
            publish_status (Callable[[str], None]): Callback to publish feedback.

        Returns:
            Tuple:
                - int: Error code (NavErr.OK on success)
                - str: Status or error message
        """
        if enable:
            if self.amcl_process is None or self.amcl_process.poll() is not None:
                publish_status("Starting localization (AMCL)")
                self.amcl_process, err, msg = self._start_process(
                    ["ros2", "run", "nav2_amcl", "amcl"], "AMCL"
                )
                return err, msg
            return NavErr.OK, "AMCL already running"
        publish_status("Stopping localization (AMCL)")
        self.amcl_process, err, msg = self._stop_process(self.amcl_process, "AMCL")
        return err, msg

    def set_mapping(self, enable: bool, publish_status) -> Tuple[int, str]:
        """
        Start or stop mapping using SLAM Toolbox.

        Args:
            enable (bool): True to start SLAM Toolbox, False to stop it.
            publish_status (Callable[[str], None]): Callback to publish feedback.

        Returns:
            Tuple:
                - int: Error code (NavErr.OK on success)
                - str: Status or error message
        """
        if enable:
            if self.slam_process is None or self.slam_process.poll() is not None:
                publish_status("Starting mapping (SLAM Toolbox)")
                self.slam_process, err, msg = self._start_process(
                    # ---------- EXERCISE C ----------
                    #
                    #
                    #
                    #
                    #
                )
                return err, msg
            return NavErr.OK, "SLAM already running"
        publish_status("Stopping mapping (SLAM Toolbox)")
        self.slam_process, err, msg = self._stop_process(
            self.slam_process, "SLAM Toolbox"
        )
        return err, msg

    def set_nav(self, enable: bool, publish_status) -> Tuple[int, str]:
        """
        Start or stop the Nav2 navigation stack.

        Args:
            enable (bool): True to start Nav2, False to stop it.
            publish_status (Callable[[str], None]): Callback to publish feedback.

        Returns:
            Tuple:
                - int: Error code (NavErr.OK on success)
                - str: Status or error message
        """
        if enable:
            if self.nav_process is None or self.nav_process.poll() is not None:
                publish_status("Starting navigation (Nav2)")
                self.nav_process, err, msg = self._start_process(
                    ["ros2", "launch", "arlab_movement", "nav2.launch.py"],
                    # ["ros2", "launch", "nav2_bringup", "navigation_launch.py"],
                    "Nav2",
                )
                return err, msg
            return NavErr.OK, "Nav2 already running"
        publish_status("Stopping navigation (Nav2)")
        self.nav_process, err, msg = self._stop_process(self.nav_process, "Nav2")
        return err, msg

    def stop_all(self):
        """
        Stop all navigation-related subprocesses.

        This shuts down AMCL, SLAM Toolbox, and Nav2 if they are running.
        Used both for explicit "stop_all" commands and action cancellation.
        """
        self.amcl_process, _, _ = self._stop_process(self.amcl_process, "AMCL")
        self.slam_process, _, _ = self._stop_process(self.slam_process, "SLAM Toolbox")
        self.nav_process, _, _ = self._stop_process(self.nav_process, "Nav2")

    # Map saving
    def save_map(self, publish_status) -> Tuple[int, str]:
        """
        Save the current map.

        Saves the map to a file using nav2_map_server's map_saver_cli.
        Optionally saves the map to the knowledge base.

        Args:
            publish_status (Callable[[str], None]): Callback to publish feedback.

        Returns:
            Tuple:
                - int: Error code (NavErr.OK on success)
                - str: Status or error message
        """
        publish_status("Saving map to file")
        err, msg = self._save_map_to_file()
        if err != NavErr.OK:
            return err, msg

        if self.save_to_database:
            publish_status("Saving map to database")
            return self._save_map_to_database()

        return NavErr.OK, msg

    def _save_map_to_file(self) -> Tuple[int, str]:
        """
        Save the current map to a file.

        Uses nav2_map_server's map_saver_cli to write a YAML + image file.
        Optionally appends a timestamp to the map name.

        Returns:
            Tuple:
                - int: Error code (NavErr.OK on success)
                - str: Status or error message
        """
        try:
            map_path = self.map_path
            if self.use_timestamp:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                map_path = f"{self.map_path}_{timestamp}"

            cmd = ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", map_path]
            env = os.environ.copy()
            workspace_setup = "/workspace/install/setup.bash"
            sourced_cmd = ["bash", "-c", f"source {workspace_setup} && {' '.join(cmd)}"]

            result = subprocess.run(
                sourced_cmd, capture_output=True, text=True, env=env
            )

            if result.returncode == 0:
                return NavErr.OK, f"Map saved to file: {map_path}"
            return NavErr.MAP_SAVE_FAILED, f"Map save failed: {result.stderr}"

        except Exception as e:
            return NavErr.MAP_SAVE_FAILED, f"Exception saving map: {e}"

    def _save_map_to_database(self) -> Tuple[int, str]:
        """
        Save the current map to the knowledge database.

        Sends the cached OccupancyGrid to the AddMap service.

        Returns:
            Tuple:
                - int: Error code (NavErr.OK on success)
                - str: Status or error message
        """
        if self.database_client is None:
            return NavErr.DB_SAVE_FAILED, "Database client not initialized"

        if self.current_map is None:
            return NavErr.DB_SAVE_FAILED, "No map received to save"

        if not self.database_client.wait_for_service(timeout_sec=self.database_timeout):
            return NavErr.DB_SERVICE_UNAVAILABLE, (
                f"Service {self.database_service} not available"
            )

        req = AddMap.Request()
        req.grid = self.current_map

        future = self.database_client.call_async(req)

        done = threading.Event()

        def _cb(_):
            done.set()

        future.add_done_callback(_cb)

        if not done.wait(timeout=self.database_timeout):
            return NavErr.DB_SAVE_FAILED, "Timeout waiting for database response"

        try:
            resp = future.result()
        except Exception as e:
            return NavErr.DB_SAVE_FAILED, f"Database call failed: {e}"

        for ok_field in ("success", "result"):
            if hasattr(resp, ok_field) and bool(getattr(resp, ok_field)):
                return NavErr.OK, "Map saved to database"

        for msg_field in ("message", "error_message", "error"):
            if hasattr(resp, msg_field):
                return (
                    NavErr.DB_SAVE_FAILED,
                    f"Database save failed: {getattr(resp, msg_field)}",
                )

        return NavErr.DB_SAVE_FAILED, "Database save failed"

    # ----------------------
    # Legacy topic callbacks
    # ----------------------
    def legacy_localization_callback(self, msg: Bool):
        """
        Legacy topic-based interface for localization control.

        Starts or stops AMCL based on the received Bool message.
        Provided for backward compatibility during migration to actions.

        Args:
            msg (Bool): True to start localization, False to stop it.
        """
        self.set_localization(bool(msg.data), lambda _: None)

    def legacy_mapping_callback(self, msg: Bool):
        """
        Legacy topic-based interface for mapping control.

        Starts or stops SLAM Toolbox based on the received Bool message.
        Provided for backward compatibility during migration to actions.

        Args:
            msg (Bool): True to start mapping, False to stop it.
        """
        self.set_mapping(bool(msg.data), lambda _: None)

    def legacy_nav_callback(self, msg: Bool):
        """
        Legacy topic-based interface for navigation control.

        Starts or stops Nav2 based on the received Bool message.
        Provided for backward compatibility during migration to actions.

        Args:
            msg (Bool): True to start navigation, False to stop it.
        """
        self.set_nav(bool(msg.data), lambda _: None)

    def legacy_map_save_callback(self, msg: Bool):
        """
        Legacy topic-based interface for saving the map.

        Triggers a map save when a True message is received.
        Provided for backward compatibility during migration to actions.

        Args:
            msg (Bool): True to save the map.
        """
        if msg.data:
            self.save_map(lambda _: None)

    def destroy_node(self):
        """
        Shut down the orchestrator node.

        Stops all navigation-related subprocesses before destroying the node.
        """
        self.stop_all()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
    node = NavigationOrchestrator()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
