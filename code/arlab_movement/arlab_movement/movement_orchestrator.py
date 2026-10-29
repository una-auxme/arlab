#!/usr/bin/env python3
"""movement_orchestrator.py

Action server that starts and stops the robot's navigation stack on demand.

Clients (e.g. the behavior trees in arlab_decision_making) send a MovementAction
goal with a `cmd` and an `enable` flag. The orchestrator runs SLAM Toolbox and Nav2
as launch subprocesses, saves the current map to file and optionally to the
knowledge base (arlab_knowledge), and can run auto-annotation: triggering CV
snapshots while the robot drives, so detected objects are added to the map.
Commands: mapping, nav, auto_annotate, map_save, stop_all.

Maintainers:
    Jonas Platzer
    Luca Kahlenberg <luca.kahlenberg@uni-a.de>
"""

import math
import os
import signal
import subprocess
import threading
from datetime import datetime
import time
from typing import Optional, Tuple

import rclpy
import rclpy.executors
from arlab_common_interfaces.action import MovementAction, VisionSnapshotAction
from arlab_common_interfaces.msg import VisionSnapshotResponse
from arlab_knowledge_interfaces.srv import AddMap
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.action.server import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import Bool, Int32
from std_srvs.srv import Trigger
from arlab_knowledge_interfaces.msg import Result
from arlab_movement.movement_safety_ids import MovementSafetyIDs


class NavErr:
    """
    Error codes reported in MovementAction.Result.error_code.

    Grouped by area: process handling (-2x), map saving (-3x), auto-annotation (-4x).
    """

    OK = 1
    UNDEFINED = 0
    BAD_COMMAND = -10
    PROCESS_START_FAILED = -20
    PROCESS_STOP_FAILED = -21
    MAP_SAVE_FAILED = -30
    DB_SERVICE_UNAVAILABLE = -31
    DB_SAVE_FAILED = -32
    ANNOTATE_SERVER_UNAVAILABLE = -40


class NavigationOrchestrator(Node):
    """
    Node owning the navigation subprocesses, map saving and auto-annotation.
    """

    def __init__(self):
        super().__init__("navigation_orchestrator")

        # services/subscriptions mutually exclusive, actions reentrant so a running goal does not block other callbacks
        self.service_group = MutuallyExclusiveCallbackGroup()
        self.action_group = ReentrantCallbackGroup()

        # defaults, overridden by ../params/arlab_navigation_params.yaml
        self.declare_parameter("map_path", "/workspace/src/arlab/code/arlab_movement/map/my_map")
        self.declare_parameter("use_timestamp", False)
        self.declare_parameter("save_to_database", False)
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("database_service", "/arlab/knowledge/add_map")
        self.declare_parameter("database_timeout", 10.0)

        # heartbeat to the local movement_local_safety node
        self.declare_parameter("heartbeat_period", 0.05) # seconds between heartbeat messages

        # auto-annotation: thresholds deciding when a CV snapshot is taken
        self.declare_parameter("snapshot_action_name", "/vision/snapshot")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("annotate_tick_period", 1.0)  # seconds between checks for snapshot conditions
        self.declare_parameter("annotate_min_dist", 0.75)  # m moved since last snapshot
        self.declare_parameter("annotate_min_yaw", 0.5)  # rad turned since last snapshot
        self.declare_parameter("annotate_min_interval", 6.0)  # min interval in seconds between snapshots
        self.declare_parameter("annotate_max_speed", 0.10)  # only fire when slower than this
        self.declare_parameter("annotate_max_yaw_rate", 0.20)  # only fire when turning slower than this
        self.declare_parameter("annotate_server_timeout", 10.0)

        self.map_path = str(self.get_parameter("map_path").value)
        self.use_timestamp = bool(self.get_parameter("use_timestamp").value)
        self.save_to_database = bool(self.get_parameter("save_to_database").value)
        self.map_topic = str(self.get_parameter("map_topic").value)
        self.database_service = str(self.get_parameter("database_service").value)
        self.database_timeout = float(self.get_parameter("database_timeout").value)

        self.current_map: Optional[OccupancyGrid] = None

        # Popen handles of the launched subprocesses, None when not running
        self.slam_process: Optional[subprocess.Popen] = None
        self.nav_process: Optional[subprocess.Popen] = None

        # auto-annotation state, reset on every start
        self.annotate_active = False
        self._snapshot_in_flight = False
        self._last_snap_pose: Optional[Tuple[float, float, float]] = None
        self._last_snap_time = 0.0
        self._latest_odom: Optional[Odometry] = None
        self._annotate_timer = None
        self._odom_sub = None
        self._snap_lock = threading.Lock()

        # cache latest /map so map_save can send it to the knowledge base
        self.map_subscription = self.create_subscription(OccupancyGrid, self.map_topic, self.map_callback, 10)

        # AddMap client, only created if maps are stored in the knowledge base
        self.database_client = None
        if self.save_to_database:
            self.database_client = self.create_client(AddMap, self.database_service, callback_group=self.service_group)
            self.get_logger().info(f"Database client created: {self.database_service}")

        # main interface, one goal per command, see execute_callback
        self._action_server = ActionServer(
            self,
            MovementAction,
            "/movement/action",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.action_group,
        )

        # auto-annotation: client for the CV snapshot action + topic telling the operator to hold position while a snapshot runs
        self.snapshot_action_name = str(self.get_parameter("snapshot_action_name").value)
        self.snapshot_client = ActionClient(self, VisionSnapshotAction, self.snapshot_action_name, callback_group=self.action_group)
        self.annotating_pub = self.create_publisher(Bool, "/arlab/movement/annotating", 10)

        # heartbeat
        self.heartbeat_period = float(self.get_parameter("heartbeat_period").value)

        self.heartbeat_group = MutuallyExclusiveCallbackGroup()
        self.heartbeat_pub = self.create_publisher(Int32, "/arlab/movement/heartbeat", 10)
        self.heartbeat_timer = self.create_timer(self.heartbeat_period, self.heartbeat_callback, callback_group=self.heartbeat_group)

        # monitor subprocesses (nav2, slam)
        self.monitor_group = MutuallyExclusiveCallbackGroup()
        self.nav2_active_client = self.create_client(Trigger, "/lifecycle_manager_navigation/is_active", callback_group=self.monitor_group)
        self.slam_state_client = self.create_client(GetState, "/slam_toolbox/get_state", callback_group=self.monitor_group)
        self.monitor_timer = self.create_timer(1.0, self._monitor_stack, callback_group=self.monitor_group)

        self.get_logger().info("NavigationOrchestrator ActionServer started.")
        self.get_logger().info("Action name: /movement/action")

    # map subscription
    def map_callback(self, msg: OccupancyGrid):
        """
        Cache the latest occupancy grid map.

        Only used when saving to the knowledge base. The file save uses
        map_saver_cli, which subscribes to /map itself.

        Args:
            msg (OccupancyGrid): The current map published by SLAM or Nav2.
        """
        self.current_map = msg
        self.get_logger().debug("Map updated.")

    # action callbacks
    def goal_callback(self, goal_request: MovementAction.Goal):
        """
        Accept every incoming MovementAction goal.

        The cmd is not checked here. Unknown cmds are still accepted and fail in
        `execute_callback` with error_code BAD_COMMAND.


        Args:
            goal_request (MovementAction.Goal): The requested action goal.

        Returns:
            GoalResponse: ACCEPT to allow execution of the goal.
        """
        self.get_logger().info(f"Goal: cmd='{goal_request.cmd}', enable={goal_request.enable}")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """
        Stop all subprocesses and auto-annotation on cancel.

        Cancelling any goal works as a stop for the whole navigation stack,
        not only for the cancelled command.

        Args:
            goal_handle: The active action goal handle.

        Returns:
            CancelResponse: ACCEPT to confirm cancellation.
        """
        self.get_logger().warning("Cancel requested; stopping all navigation processes.")
        self.stop_all()
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """
        Execute a MovementAction goal.

        Commands:
          - mapping: start/stop SLAM Toolbox
          - nav: start/stop Nav2
          - auto_annotate: start/stop CV snapshot triggering
          - map_save: save the map to file and optionally the knowledge base, enable is ignored
          - stop_all: stop everything above, enable is ignored

        Publishes status feedback during execution. The goal always ends as
        succeeded, the outcome is reported in result.success and result.error_code.

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
            if cmd == "mapping":
                err, msg = self.set_mapping(enable, publish_status)
            elif cmd == "nav":
                err, msg = self.set_nav(enable, publish_status)
            elif cmd == "auto_annotate":
                err, msg = self.set_auto_annotate(enable, publish_status)
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

    # heartbeat callback
    def heartbeat_callback(self):
        """
        Publish a heartbeat message to the local movement_local_safety node.

        The message contains the node ID of this orchestrator, which is
        defined in movement_safety_ids.py.
        """
        self.heartbeat_pub.publish(Int32(data=MovementSafetyIDs.MOVEMENT_ORCHESTRATOR))

    # monitor subprocesses
    def _monitor_stack(self):
        """" Check if SLAM and NAV2 are running and healthy when excpected to."""
        if self.nav_process is not None :
            if self.nav_process.poll() is not None:
                self._report_stack_failure("NAV2", "launch process exited")
            elif self._nav2_check_pending:
                # previous request still pending, likely failure?
                self._report_stack_failure("NAV2", "lifecycle manager not responding")
            elif not self.nav2_active_client.service_is_ready():
                self._report_stack_failure("NAV2", "is_active service not available")
            else:
                self._nav2_check_pending = True
                self.nav2_active_client.call_async(Trigger.Request()).add_done_callback(self._on_nav2_active)

        if self.slam_process is not None:
            if self.slam_process.poll() is not None:
                self._report_stack_failure("SLAM", "launch process exited")
            elif self._slam_check_pending:
                # previous request unanswered for a whole tick
                self._report_stack_failure("SLAM Toolbox", "slam_toolbox not responding")
            elif not self.slam_state_client.service_is_ready():
                self._report_stack_failure("SLAM Toolbox", "get_state service unavailable")
            else:
                self._slam_check_pending = True
                self.slam_state_client.call_async(GetState.Request()).add_done_callback(self._on_slam_state)


    def _report_stack_failure(self, stack_name: str, reason: str):
        pass

    def _on_nav2_active(self, future):
        """Handle the response from the Nav2 active check service."""
        self._nav2_check_pending = False
        response = future.result()
        if not response.success:
            self._report_stack_failure("NAV2", f"lifecycle manager reports not active: {response.message}")

    def _on_slam_state(self, future):
        """Handle the response from the SLAM Toolbox state service."""
        self._slam_check_pending = False
        response = future.result()
        if response.current_state.id != 3:  # 3 is ACTIVE
            self._report_stack_failure("SLAM", f"state is not active: {response.current_state.label}")

    # process helpers
    def _start_process(self, cmd_list, name: str) -> Tuple[Optional[subprocess.Popen], int, str]:
        """
        Launch a command as a subprocess in its own process group.

        The command runs in bash with the workspace sourced, so `ros2 launch`
        finds our packages. The own process group lets `_stop_process` reach the
        launch file and every node it started. Waits 2 s to catch commands that
        exit immediately, e.g. a missing package.

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

            # give the process time to start, catch immediate exits
            time.sleep(2.0)

            if proc.poll() is not None:
                return None, NavErr.PROCESS_START_FAILED, f"{name} exited immediately"

            return proc, NavErr.OK, f"{name} started"
        except Exception as e:
            return None, NavErr.PROCESS_START_FAILED, f"Failed to start {name}: {e}"

    def _stop_process(self, proc: Optional[subprocess.Popen], name: str) -> Tuple[Optional[subprocess.Popen], int, str]:
        """
        Stop a subprocess started by `_start_process`.

        Sends SIGINT to its process group, same as Ctrl+C on the launch.
        Does not wait for the process to exit.

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

    # commands
    def set_mapping(self, enable: bool, publish_status) -> Tuple[int, str]:
        """
        Start or stop SLAM Toolbox (async) via arlab_movement/slam_async.launch.py.

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
                    ["ros2", "launch", "arlab_movement", "slam_async.launch.py"],
                    # ["ros2", "launch", "slam_toolbox", "online_async_launch.py"],
                    "SLAM Toolbox",
                )
                return err, msg
            return NavErr.OK, "SLAM already running"
        publish_status("Stopping mapping (SLAM Toolbox)")
        self.slam_process, err, msg = self._stop_process(self.slam_process, "SLAM Toolbox")
        return err, msg

    def set_nav(self, enable: bool, publish_status) -> Tuple[int, str]:
        """
        Start or stop Nav2 via arlab_movement/nav2.launch.py.

        This node only manages the stack, navigation goals are sent to Nav2 directly.

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

    # auto-annotation
    def set_auto_annotate(self, enable: bool, publish_status) -> Tuple[int, str]:
        """
        Start or stop periodic CV snapshots that auto-annotate the map.

        Used during mapping drives to fill the knowledge base with detected
        objects. When enabled, subscribes to odometry and starts a timer that
        evaluates the snapshot condition checks on each tick, see `_annotate_tick`.
        Warns if SLAM is not running, since object poses need the map frame.

        Args:
            enable (bool): True to start auto-annotation, False to stop it.
            publish_status (Callable[[str], None]): Callback to publish feedback.

        Returns:
            Tuple:
                - int: Error code (NavErr.OK on success)
                - str: Status or error message
        """
        if enable:
            if self.annotate_active:
                return NavErr.OK, "auto-annotation already running"
            if self.slam_process is None or self.slam_process.poll() is not None:
                self.get_logger().warning("auto_annotate started without SLAM running. map frame may be unavailable.")
            timeout = float(self.get_parameter("annotate_server_timeout").value)
            if not self.snapshot_client.wait_for_server(timeout_sec=timeout):
                return NavErr.ANNOTATE_SERVER_UNAVAILABLE, (f"Snapshot action '{self.snapshot_action_name}' not available")
            publish_status("Starting auto-annotation (CV snapshots)")

            self._last_snap_pose = None
            self._last_snap_time = 0.0
            self._latest_odom = None

            self._odom_sub = self.create_subscription(
                Odometry,
                str(self.get_parameter("odom_topic").value),
                self._odom_callback,
                10,
                callback_group=self.service_group,
            )

            period = float(self.get_parameter("annotate_tick_period").value)
            self._annotate_timer = self.create_timer(period, self._annotate_tick, callback_group=self.action_group)
            self.annotate_active = True
            return NavErr.OK, "auto-annotation started"

        publish_status("Stopping auto-annotation")
        self._teardown_annotation()
        return NavErr.OK, "auto-annotation stopped"

    def _teardown_annotation(self):
        """Cancel the annotation timer and remove the odom subscription if active."""
        self.annotate_active = False
        if self._annotate_timer is not None:
            self._annotate_timer.cancel()
            self._annotate_timer = None
        if self._odom_sub is not None:
            self.destroy_subscription(self._odom_sub)
            self._odom_sub = None

    def _odom_callback(self, msg: Odometry):
        """
        Cache the latest odometry data for speed and turn rate checks.

        Args:
            msg (Odometry): The latest odometry message.
        """
        self._latest_odom = msg

    def _annotate_tick(self):
        """
        Evaluate the snapshot condition checks and fire a snapshot when all pass.

        Gates / Checks:
        - Auto-annotation is active, odometry was received and no snapshot is in flight
        - The robot is moving slowly (linear speed and turn rate below threshold) for a stable frame
        - A minimum time interval has elapsed since the last snapshot
        - The robot has moved or turned enough since the last snapshot, to avoid duplicate snapshots of the same view
        """
        if not self.annotate_active or self._latest_odom is None:
            return
        with self._snap_lock:
            if self._snapshot_in_flight:
                return

        x, y, yaw, speed, yaw_rate = self._pose_from_odom(self._latest_odom)
        now = self.get_clock().now().nanoseconds * 1e-9

        if speed > float(self.get_parameter("annotate_max_speed").value):
            return
        if yaw_rate > float(self.get_parameter("annotate_max_yaw_rate").value):
            return

        if now - self._last_snap_time < float(self.get_parameter("annotate_min_interval").value):
            return

        if self._last_snap_pose is not None:
            moved = math.hypot(x - self._last_snap_pose[0], y - self._last_snap_pose[1])
            turned = abs(self._angle_diff(yaw, self._last_snap_pose[2]))
            if moved < float(self.get_parameter("annotate_min_dist").value) and turned < float(
                self.get_parameter("annotate_min_yaw").value
            ):
                return

        self._fire_snapshot(x, y, yaw, now)

    def _fire_snapshot(self, x: float, y: float, yaw: float, now: float):
        """
        Send one /vision/snapshot goal and publish to /arlab/movement/annotating.

        Records the snapshot pose/time up front so the gates advance even while
        the goal is in flight, marks the snapshot as in flight to prevent
        overlap.
        Entities are written to the knowledge base by the vision node itself.

        Args:
            x (float): X position of the robot in m.
            y (float): Y position of the robot in m.
            yaw (float): Yaw of the robot in rad.
            now (float): Current time in s.
        """
        goal = VisionSnapshotAction.Goal()
        goal.command.clear_database = False  # accumulate annotations across the run
        goal.command.mask_hand = False
        # extra_models left empty to use general model for annotation

        with self._snap_lock:
            self._snapshot_in_flight = True
        self._last_snap_pose = (x, y, yaw)
        self._last_snap_time = now

        # signal operator for running snapshot
        self.annotating_pub.publish(Bool(data=True))
        self.get_logger().info(f"auto-annotate: SNAPSHOT IN PROGRESS at ({x:.2f}, {y:.2f}, {yaw:.2f} rad) - hold position.")

        send_future = self.snapshot_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_snapshot_goal)

    def _on_snapshot_goal(self, future):
        """
        Handle the goal-accepted response and chain to the result future.

        Args:
            future (Future): Future of the snapshot goal request.
        """
        try:
            handle = future.result()
        except Exception as e:
            self._clear_in_flight()
            self.get_logger().warning(f"Snapshot goal send failed: {e}")
            return
        if not handle.accepted:
            self._clear_in_flight()
            self.get_logger().warning("Snapshot goal rejected by server.")
            return
        handle.get_result_async().add_done_callback(self._on_snapshot_result)

    def _on_snapshot_result(self, future):
        """
        Log the snapshot outcome and clear the in-flight flag/operator signal.

        Args:
            future (Future): Future of the snapshot goal result.
        """
        self._clear_in_flight()
        try:
            response = future.result().result.response
        except Exception as e:
            self.get_logger().warning(f"Snapshot result failed: {e}")
            return
        if response.result != VisionSnapshotResponse.SUCCESS:
            self.get_logger().warning(f"Snapshot failed: {response.error_msg}")

    def _clear_in_flight(self):
        """Mark no snapshot in flight and publish to /arlab/movement/annotating that snapshot finished."""
        with self._snap_lock:
            self._snapshot_in_flight = False
        self.annotating_pub.publish(Bool(data=False))

    def _pose_from_odom(self, msg: Odometry) -> Tuple[float, float, float, float, float]:
        """
        Extract planar (x, y, yaw, linear_speed, abs_yaw_rate) from Odometry.

        Args:
            msg (Odometry): The odometry to extract from.

        Returns:
            Tuple:
                - float: x
                - float: y
                - float: yaw
                - float: linear speed
                - float: absolute yaw rate
        """
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        v = msg.twist.twist.linear
        speed = math.hypot(v.x, v.y)
        yaw_rate = abs(msg.twist.twist.angular.z)
        return p.x, p.y, yaw, speed, yaw_rate

    @staticmethod
    def _angle_diff(a: float, b: float) -> float:
        """
        Shortest signed difference between two angles (rad).

        Args:
            a (float): First angle in rad.
            b (float): Second angle in rad.

        Returns:
            float: Signed difference a - b in rad, in [-pi, pi].
        """
        return math.atan2(math.sin(a - b), math.cos(a - b))

    def stop_all(self):
        """
        Stop all navigation-related subprocesses.

        This shuts down SLAM Toolbox and Nav2 if they are running, and
        also stops auto-annotation if active. Used both for explicit "stop_all"
        commands and action cancellation.
        """
        self._teardown_annotation()
        self.slam_process, _, _ = self._stop_process(self.slam_process, "SLAM Toolbox")
        self.nav_process, _, _ = self._stop_process(self.nav_process, "Nav2")

    # map saving
    def save_map(self, publish_status) -> Tuple[int, str]:
        """
        Save the current map.

        Saves the map to a file using nav2_map_server's map_saver_cli.
        Optionally saves the map to the knowledge base.
        Needs SLAM or a map server publishing /map.

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

            result = subprocess.run(sourced_cmd, capture_output=True, text=True, env=env)

            if result.returncode == 0:
                return NavErr.OK, f"Map saved to file: {map_path}"
            return NavErr.MAP_SAVE_FAILED, f"Map save failed: {result.stderr}"

        except Exception as e:
            return NavErr.MAP_SAVE_FAILED, f"Exception saving map: {e}"

    def _save_map_to_database(self) -> Tuple[int, str]:
        """
        Save the current map to the knowledge database.

        Sends the cached OccupancyGrid to the AddMap service. Blocks up to
        database_timeout.

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
            return NavErr.DB_SERVICE_UNAVAILABLE, (f"Service {self.database_service} not available")

        req = AddMap.Request()
        req.grid = self.current_map

        future = self.database_client.call_async(req)

        # the executor already spins this node, so wait on an event instead of spin_until_future_complete
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
            if hasattr(resp, ok_field) and resp.result.result_type == Result.SUCCESS:
                self.get_logger().info("Map added with id: " + str(resp.mapid))
                return NavErr.OK, "Map saved to database"

        for msg_field in ("message", "error_message", "error"):
            if hasattr(resp, msg_field):
                return (
                    NavErr.DB_SAVE_FAILED,
                    f"Database save failed: {getattr(resp, msg_field)}",
                )

        return NavErr.DB_SAVE_FAILED, "Database save failed"

    def destroy_node(self):
        """
        Shut down the orchestrator node.

        Stops all navigation-related subprocesses first, so no SLAM or Nav2
        processes outlive the node.
        """
        self.stop_all()
        super().destroy_node()


def main(args=None):
    """
    Start the navigation orchestrator node with a multi-threaded executor.

    Two threads, so a running action goal does not block the snapshot and service callbacks.
    """
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=3)
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
