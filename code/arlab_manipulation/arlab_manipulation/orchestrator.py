#!/usr/bin/env python3
"""
Orchestrator Node for Robotic Manipulation (ROS2).

This node subscribes to manipulation commands via Action, queries gripping
parameters, computes gripping poses, and publishes orchestrator data for
execution by MoveIt or other downstream nodes.

Maintainer:
    Sofia Öttl <sofia.oettl@uni-a.de>
"""

from threading import Event
from typing import cast

import rclpy
import rclpy.executors
import tf2_ros
from arlab_common_interfaces.action import ManipulationAction, OrchestratorAction
from arlab_common_interfaces.msg import ManipulationResponse
from arlab_common_interfaces.srv import GrippingParameter
from arlab_knowledge_interfaces.srv import GetEntity, GetShape
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.msg import PlanningScene

# from octomap_msgs.msg import Octomap, OctomapWithPose
from rclpy.action.client import ActionClient
from rclpy.action.server import ActionServer, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from std_msgs.msg import Float64, String, Bool

from .utils.octomap_utils import find_placing_area
from .utils.transform_utils import (
    transform_pose,  # transform_bBox, transform_pointCloud,
)


class orchestrator(Node):
    """ROS2 Node for orchestrating robotic manipulation actions.

    Responsibilities:
        - Accept manipulation commands via ManipulationAction.
        - Query object properties (GetEntity, GetShape) from the knowledge base.
        - Request gripping parameters via GrippingParameter service.
        - Compute pick/place poses considering octomap occupancy.
        - Send orchestrator goals to downstream MoveIt Action server.

    Attributes:
        action_done_event: Threading event to synchronize asynchronous callbacks.
        tf_buffer / tf_listener: TF2 utilities for frame transforms.
        object_name, object_group: Semantic object info.
        pickable: Flag if object is manipulable.
        pose / gripping / placing points: Computed target poses.
        octomap: Latest PlanningScene octomap data.
        force / grip modes: Parameters from GrippingParameter service.
        err / msg: Status code and message for manipulation responses.
    """

    def __init__(self):
        """Initialize ROS2 node, Action servers/clients, service clients, and subscriptions.

        Side Effects:
            - Registers multiple asynchronous callbacks.
            - Initializes TF2 buffer and listener.
            - Subscribes to /monitored_planning_scene to receive octomap updates.
        """
        super().__init__("orchestrator")
        self.service_group = MutuallyExclusiveCallbackGroup()

        # Outgoing action client (to MoveIt / C++ node)
        self._orchestrator_client = ActionClient(
            self,
            OrchestratorAction,
            "/orchestrator/action",
            callback_group=self.service_group,
        )

        # Incoming action server (from decision maker)
        self._action_server = ActionServer(
            self,
            ManipulationAction,
            "/manipulation/action",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
        )

        # Threading event used to block execute_callback until async chain ends
        self.action_done_event = Event()

        prefix = "/arlab/knowledge"

        # Knowledge base service clients
        self.client_get_entity = self.create_client(GetEntity, f"{prefix}/get_entity", callback_group=self.service_group)
        self.client_get_shape = self.create_client(GetShape, f"{prefix}/get_shape", callback_group=self.service_group)

        # Gripping parameter service client
        self.client_gripping_parameter = self.create_client(GrippingParameter, "GetGrippingParameter", callback_group=self.service_group)

        # Octomap subscription (MoveIt planning scene)
        self.subscription = self.create_subscription(PlanningScene, "/monitored_planning_scene", self.octomap_callback, 10)

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # publisher for object placed
        self.obj_placed_pub = self.create_publisher(Bool, "/object_placed", 10)

        # Default state initialization
        self.entity_id = None
        self.command_type = "home"
        self.object_name = "default"
        self.object_group = "default"
        self.pickable = False
        self.ref_frame = "camera_link"
        self.force = 5.0
        self.gripping_point_pos = Point()
        self.gripping_point_orient = Quaternion(w=1.0)
        self.placing_point_pos = Point()
        self.placing_point_orient = Quaternion(w=1.0)
        self.octomap = None
        self.action_result = None
        self.err = ManipulationResponse.UNDEFINED
        self.msg = ""

    def octomap_callback(self, msg: PlanningScene):
        """Callback to receive the octomap from the MoveIt planning scene.

        Updates self.octomap for later placement computations.

        Args:
            msg: PlanningScene message containing octomap.
        """
        octomap_with_pose = msg.world.octomap
        octomap_msg = octomap_with_pose.octomap
        self.octomap = octomap_msg.data
        if not self.octomap:
            # Lines below to treat an empty octomap as an error (uncomment for testing without working octomap):
            # self.get_logger().warn("Octomap is empty")
            # self.err = -40
            # self.msg = "Octomap is empty"
            # self.finish_action()
            return
        self.get_logger().info("Octomap received")

    def goal_callback(self, goal_request):
        """Accept incoming ManipulationAction goals.

        Logs command and entity information for debugging.

        Args:
            goal_request: Action goal request containing manipulation command.

        Returns:
            GoalResponse.ACCEPT to accept all incoming goals.
        """
        self.get_logger().info(
            f"Data from Decision Making: cmd={goal_request.command.command_type}, entity_id={goal_request.command.target_entityid}"
        )
        return GoalResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Execute the manipulation command asynchronously.

        Sequence:
            1. Query GetEntity service for object info.
            2. Transform pose to reference frame.
            3. Query GetShape service for object geometry.
            4. Request GrippingParameter if pickable.
            5. Compute pick/place pose.
            6. Send orchestrator goal to MoveIt.

        Side Effects:
            Sets self.err / self.msg for ManipulationResponse.
            Uses threading event to wait for async service responses.
        """

        self.action_result = ManipulationAction.Result()
        self.action_result.response = ManipulationResponse()

        goal_command = goal_handle.request.command
        self.command_type = goal_command.command_type
        self.entity_id = goal_command.target_entityid
        self.target_pose = getattr(goal_command, "target_pose", None)

        if self.command_type in ["pick", "place"]:
            # Start async service chain: GetEntity → GetShape → GrippingParameter
            self.req_get_entity = GetEntity.Request()
            self.req_get_entity.entityid = self.entity_id
            future_entity = self.client_get_entity.call_async(self.req_get_entity)
            future_entity.add_done_callback(self.handle_get_entity_response)
        else:
            # For non-pick/place commands (e.g. "home"), skip knowledge queries
            self.send_goal()

        # Block until the async chain completes (set via finish_action)
        self.action_done_event.wait()
        self.action_done_event.clear()

        # Give feedback if object was correctly placed
        if self.command_type == "place":
            placed = Bool()
            placed.data = self.err == ManipulationResponse.SUCCESS  # ← Erfolg-Wert bestätigen
            self.obj_placed_pub.publish(placed)
            self.get_logger().info(f"Publishing {placed.data} on /object_placed")  # DEBUG

        if not goal_handle.is_active:
            return

        self.get_logger().info("Action response send to desicion maker")
        self.action_result.response.error_code = self.err
        self.action_result.response.message = self.msg

        goal_handle.succeed()

        return self.action_result

    def handle_get_entity_response(self, future):
        """Handle asynchronous response from GetEntity service.

        Extracts object information (pickable, name, category, pose) and
        transforms the pose to the reference frame using TF2.

        Side Effects:
            - Updates self.pickable, self.object_name, self.object_group, self.pose
            - Requests GetShape service if pickable
            - Updates self.err / self.msg if service fails or transform fails

        Args:
            future: Future object from async GetEntity service call.
        """
        try:
            response = future.result()
            entity = response.data
            self.get_logger().info((f"Entity info received: name={entity.pickable.object_name},\npose={entity.pose}"))

            if entity.entity_type.PICKABLE == 2:
                self.pickable = True
                self.object_name = entity.pickable.object_name
                self.object_group = entity.pickable.object_category
                self.pose = entity.pose
            else:
                self.pickable = False
                self.object_name = "noName"
                self.object_group = "default"

            self.ref_frame = entity.pose_reference_frame
            self.stamp = entity.stamp

            # Transform pose from the entity's reference frame to base_link
            self.pose, err, msg = transform_pose(self.tf_buffer, cast(Pose, self.pose), self.stamp, self.ref_frame)

            if err == 1:
                # Continue chain: request shape data for grasp/place planning
                self.req_get_shape = GetShape.Request()
                self.req_get_shape.entityid = self.entity_id
                future_shape = self.client_get_shape.call_async(self.req_get_shape)
                future_shape.add_done_callback(self.handle_get_shape_response)
            else:
                self.err = err
                self.msg = msg
                self.finish_action()

        except Exception as e:
            self.get_logger().error(f"GetEntity failed: {e}")
            self.err = -41
            self.msg = "GetEntity from Knowledgebase failed"
            self.finish_action()

    def handle_get_shape_response(self, future):
        """Handle asynchronous response from GetShape service.

        Extracts point cloud or bounding box geometry and initiates
        GrippingParameter request if object is pickable.

        Side Effects:
            - Updates self.point_cloud, self.bounding_box
            - Requests GrippingParameter service if pickable
            - Updates self.err / self.msg if service fails

        Args:
            future: Future object from async GetShape service call.
        """
        try:
            response = future.result()
            shape = response.shape
            self.point_cloud = shape.pointcloud if shape.has_pointcloud else None
            self.bounding_box = shape.boundingbox2d if shape.has_boundingbox2d else None

            # TODO: TF transform for point cloud and bounding box is experimental
            # self.point_cloud, err, msg = transform_pointCloud(
            #     self.tf_buffer, self.point_cloud, self.stamp, self.ref_frame
            # )

            # self.bounding_box, err, msg = transform_bBox(
            #     self.tf_buffer, self.bounding_box, self.stamp, self.ref_frame
            # )

            if self.pickable:
                # Continue chain: fetch gripping parameters for this object group
                self.req_gripping_parameter = GrippingParameter.Request()
                self.req_gripping_parameter.objectgroup = self.object_group
                future_param = self.client_gripping_parameter.call_async(self.req_gripping_parameter)
                future_param.add_done_callback(self.handle_gripping_parameter_response)
            else:
                # self.err = err
                # self.msg = msg
                self.finish_action()

        except Exception as e:
            self.get_logger().error(f"GetShape failed: {e}")
            self.err = -42
            self.msg = "GetShape from Knowledgebase failed"
            self.finish_action()

    def handle_gripping_parameter_response(self, future):
        """Handle response from GrippingParameter service.

        Updates gripping force and grip modes, then computes pick/place pose.

        Side Effects:
            - Updates self.force, self.grip_pos_mode, self.grip_orient_mode
            - Calls compute_goal_pose
            - Updates self.err / self.msg if service fails

        Args:
            future: Future object from async GrippingParameter service call.
        """
        try:
            response = future.result()
            self.force = response.gripforce
            self.grip_pos_mode = response.grippos_mode
            self.grip_orient_mode = response.griporient_mode
            self.compute_goal_pose()
        except Exception as e:
            self.get_logger().error(f"GetGrippingParameter failed: {e}")
            self.err = -43
            self.msg = "GetGrippingParameter from ParameterService failed"
            self.finish_action()

    def compute_goal_pose(self):
        """Compute the target pose for pick or place actions.

        For 'pick':
            - Applies static offsets to the object pose
            - Sets gripping orientation
        For 'place':
            - Computes placement above shelf using octomap
            - Sets placing orientation to match gripping orientation

        Side Effects:
            - Updates self.gripping_point_pos / self.gripping_point_orient
            - Updates self.placing_point_pos / self.placing_point_orient
            - Calls send_goal
            - Updates self.err / self.msg if pose calculation fails
        """
        if self.command_type == "pick":
            if self.pose is not None:
                self.gripping_point_pos = self.pose.position
                # Static offsets to align the gripper with the object centre.
                # TODO: Replace with a proper grasp planner.
                self.gripping_point_pos.x += -0.085
                self.gripping_point_pos.y += 0.01
                self.gripping_point_pos.z = 0.19

                # Fixed grasp orientation (pre-calibrated for the Zirbi gripper)
                self.gripping_point_orient = Quaternion(x=0.243005, y=0.808244, z=-0.0517573, w=0.533864)
                self.send_goal()
            else:
                self.err = -52
                self.msg = "No pose calculated"
                self.finish_action()

        elif self.command_type == "place":
            if self.octomap:
                pose, err, msg = find_placing_area(
                    octo_data=self.octomap,
                    bbox=self.bounding_box,
                    margin=0.02,  # Extra safety clearance around the object [m]
                    lift=0.01,  # Vertical lift above shelf floor [m]
                    offset_x=0.05,  # Additional clearance along gripper X-axis [m]
                    offset_y=0.05,  # Additional clearance along gripper Y-axis [m]
                )
                if err == 1:
                    self.placing_point_pos = Point(x=pose.position.x, y=pose.position.y, z=pose.position.z)
                    # Reuse the grasp orientation for placing
                    self.placing_point_orient = self.gripping_point_orient
                    self.send_goal()
                else:
                    self.err = err
                    self.msg = self.msg
                    self.finish_action()

    def send_goal(self):
        """Send computed pick/place pose as OrchestratorAction goal.

        Side Effects:
            - Waits for OrchestratorAction server
            - Sends goal with pose, grip force, and command type
            - Registers callback for response
            - Updates self.err / self.msg if server unavailable
        """
        # Select the appropriate target pose for the command
        if self.command_type == "pick":
            goal_pose = Pose()
            goal_pose.position = self.gripping_point_pos
            goal_pose.orientation = self.gripping_point_orient
        elif self.command_type == "place":
            goal_pose = Pose()
            goal_pose.position = self.placing_point_pos
            goal_pose.orientation = self.placing_point_orient
        else:
            # For commands like "home", use a provided target pose or identity
            goal_pose = self.target_pose or Pose()

        msg = OrchestratorAction.Goal()
        msg.data.pose = goal_pose
        msg.data.grip_force = Float64(data=self.force)
        msg.data.cmd = String(data=self.command_type)

        if not self._orchestrator_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("OrchestratorAction server is not available")
            self.err = -44
            self.msg = "Orchestrator Actionserver is not available"
            self.finish_action()
            return

        send_future = self._orchestrator_client.send_goal_async(msg)
        send_future.add_done_callback(self.handle_orchestrator_response)

    def handle_orchestrator_response(self, future):
        """Handle goal acceptance from OrchestratorAction server.

        Registers callback for result retrieval or sets error if rejected.

        Side Effects:
            - Updates self.err / self.msg if goal rejected or failed
        """
        try:
            self.goal_handle = future.result()
            if not self.goal_handle.accepted:
                self.get_logger().error("Orchestrator goal rejected")
                self.err = -45
                self.msg = "Orchestrator Actiongoal from MoveIt rejected"
                self.finish_action()

            self.get_logger().info("Orchestrator goal accepted")
            self.goal_handle.get_result_async().add_done_callback(self.handle_orchestrator_result)

        except Exception as e:
            self.get_logger().error(f"Failed to receive orchestrator goal: {e}")
            self.err = -46
            self.msg = "Failed to receive orchestrator goal"
            self.finish_action()

    def handle_orchestrator_result(self, future):
        """Handle result from OrchestratorAction execution.

        Updates status code and message based on result from downstream execution.

        Side Effects:
            - Updates self.err / self.msg
            - Signals action completion via finish_action()
        """
        try:
            result = future.result().result.response
            self.msg = result.message
            self.get_logger().info(f"Orchestrator action completed: {self.msg}")
            self.err = result.error_code
            self.finish_action()

        except Exception as e:
            self.get_logger().error(f"Handle orchestrator action result failed: {e}")
            self.err = -48
            self.msg = "Handle orchestrator action result failed"
            self.finish_action()

    def finish_action(self):
        """Signal that the current manipulation action is complete.

        Side Effects:
            - Sets self.action_done_event to unblock waiting execute_callback()
        """
        self.action_done_event.set()


def main(args=None):
    """Start the orchestrator node with a multi-threaded executor.

    Uses two threads so that the blocking ``execute_callback`` and the async
    service callbacks can run concurrently without deadlocking.

    Args:
        args: Optional command-line arguments forwarded to ``rclpy.init``.
    """
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)

    try:
        node = orchestrator()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
