#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid
import subprocess
import signal
import os
from datetime import datetime
import threading
from arlab_knowledge_interfaces.msg import EntityPickable, EntityType, StatusType
from arlab_knowledge_interfaces.srv import (
    AddEntity,
    AddMap,
    AddStatusEvent,
    DelEntities,
    GetDescription,
    GetEntities,
    GetMap,
    GetPose,
    GetReference,
    GetShape,
    GetStatusEvents,
    UpdEntity,
    UpdPose,
    UpdReference,
    UpdShape,
)


class NavigationStackManager(Node):
    """This Node starts and stops the navigation nodes for localization and mapping according to the movement orchestrator"""

    def __init__(self):
        """Subscribes to the topics that the orchestrator publishes to and creates handles for the subprocesses
        """
        super().__init__('navigation_stack_manager')

        self.declare_parameter('map_path', '/workspace/src/arlab/code/arlab_movement/map/my_map')
        self.declare_parameter('use_timestamp', False)
        self.declare_parameter('save_to_database', False)
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('database_service', '/arlab/knowledge/add_map')
        self.declare_parameter('database_timeout', 10.0)

        self.map_path = self.get_parameter('map_path').value
        self.use_timestamp = self.get_parameter('use_timestamp').value
        self.save_to_database = self.get_parameter('save_to_database').value
        self.map_topic = self.get_parameter('map_topic').value
        self.database_service = self.get_parameter('database_service').value
        self.database_timeout = self.get_parameter('database_timeout').value

        self.create_subscription(Bool, 'localization_bool', self.localization_callback, 10)
        self.create_subscription(Bool, 'mapping_bool', self.mapping_callback, 10)
        self.create_subscription(Bool, 'nav_bool', self.nav_callback, 10)
        self.create_subscription(Bool, 'map_save', self.map_save_callback, 10)

        # Subscription to get current map
        self.current_map = None
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            10
        )

        # Service client for database
        self.database_client = None
        if self.save_to_database:
            self.database_client = self.create_client(AddMap, self.database_service)
            self.get_logger().info(f"Created database service client for {self.database_service}")

        self.amcl_process = None
        self.slam_process = None
        self.nav_process = None

        self.get_logger().info("Navigation Stack Manager 2 (subprocess mode) started.")
        self.get_logger().info(f"Map path: {self.map_path}")
        self.get_logger().info(f"Use timestamp: {self.use_timestamp}")
        self.get_logger().info(f"Save to database: {self.save_to_database}")
        self.get_logger().info(f"Map topic: {self.map_topic}")

    def map_callback(self, msg):
        """Callback for map topic - stores the latest map"""
        self.current_map = msg
        self.get_logger().debug(f"Received map update: {msg.header.frame_id}")

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
                    ["ros2", "launch", "slam_toolbox", "online_async_launch.py"], "SLAM Toolbox"
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
                    ["ros2", "run", "nav2_bringup", "navigation_launch.py"], "Nav2 Stack"
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
        """Save map to either file or database based on configuration"""
        # Save to file first for backup
        self.save_map_to_file()
        
        # Save to db if enabled
        if self.save_to_database:
            self.save_map_to_database()

    def save_map_to_file(self):
        """Save map to file"""
        try:
            map_path = self.map_path

            if self.use_timestamp:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                map_path = f"{self.map_path}_{timestamp}"
                self.get_logger().info(f"Using timestamped map name: {map_path}")
            else:
                self.get_logger().info(f"Using fixed map name: {map_path}")

            self.get_logger().info(f"Saving map to file: {map_path}")

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
                self.get_logger().info(f"Map saved successfully to file: {map_path}")
                if result.stdout:
                    self.get_logger().debug(f"Map saver output: {result.stdout}")
            else:
                self.get_logger().error(f"Failed to save map to file. Error: {result.stderr}")

        except Exception as e:
            self.get_logger().error(f"Exception while saving map to file: {str(e)}")

    def save_map_to_database(self):
        """Save map to database via service call"""
        try:
            if self.current_map is None:
                self.get_logger().error('No map available to save. Make sure map topic is publishing correctly.')
                self.get_logger().info('Attempting to get current map...')
                self.get_map_once()
                if self.current_map is None:
                    self.get_logger().error('Failed to get map. Cannot save to database.')
                    return
            
            self.get_logger().info("Saving map to database...")

            # Wait for service to be available
            if not self.database_client.wait_for_service(timeout_sec=self.database_timeout):
                self.get_logger().error(f"Service {self.database_service} not available after {self.database_timeout}s")
                return
            
            request = AddMap.Request()
            request.grid = self.current_map
            request.grid.header.stamp = self.get_clock().now().to_msg()
            
            self.get_logger().info(f"Sending map to database service: {self.database_service}")
            self.get_logger().info(f"  Frame ID: {request.grid.header.frame_id}")
            self.get_logger().info(f"  Resolution: {request.grid.info.resolution:.3f}m")
            self.get_logger().info(f"  Dimensions: {request.grid.info.width}x{request.grid.info.height}")
            self.get_logger().info(f"  Origin: [{request.grid.info.origin.position.x:.2f}, {request.grid.info.origin.position.y:.2f}]")
            
            future = self.database_client.call_async(request)
            future.add_done_callback(self.database_service_callback)

        except Exception as e:
            self.get_logger().error(f"Exception while saving map to database: {str(e)}")

    def database_service_callback(self, future):
        """Callback for database service response"""
        try:
            response = future.result()
            
            if hasattr(response, 'success') and response.success:
                self.get_logger().info(f"✅ Map successfully saved to database.")
                if hasattr(response, 'map_id'):
                    self.get_logger().info(f"Map ID: {response.map_id}")
                elif hasattr(response, 'mapid'):
                    self.get_logger().info(f"Map ID: {response.mapid}")
            elif hasattr(response, 'result') and response.result:
                self.get_logger().info(f"✅ Map successfully saved to database.")
                if hasattr(response, 'mapid'):
                    self.get_logger().info(f"Map ID: {response.mapid}")
            else:
                error_msg = "Unknown error"
                if hasattr(response, 'error_message'):
                    error_msg = response.error_message
                elif hasattr(response, 'message'):
                    error_msg = response.message
                elif hasattr(response, 'error'):
                    error_msg = response.error
                self.get_logger().error(f"❌ Failed to save map to database: {error_msg}")
                
        except Exception as e:
            self.get_logger().error(f"❌ Service call failed: {str(e)}")

    def get_map_once(self):
        """Try to get the current map once"""
        try:
            received_map = None
            event = threading.Event()

            def temp_callback(msg):
                nonlocal received_map
                received_map = msg
                event.set()

            temp_sub = self.create_subscription(OccupancyGrid, self.map_topic, temp_callback, 1)

            if event.wait(timeout=2.0):
                self.current_map = received_map
                self.get_logger().info("Successfully retrieved current map")
            else:
                self.get_logger().warning("Timeout waiting for map message")
            
            self.destroy_subscription(temp_sub)

        except Exception as e:
            self.get_logger().error(f"Error getting map: {str(e)}")

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
        node.get_logger().info("Shutting down navigation stack manager...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()