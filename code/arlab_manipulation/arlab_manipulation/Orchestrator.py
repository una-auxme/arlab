#!/usr/bin/env python3
"""
Orchestrator.py
---------------

ROS2 Node 'Orchestrator' that subscribes to manipulation commands via Action,
queries the gripping force service, computes gripping poses, and publishes orchestrator
data.

Author: Sofia Öttl
Date: 2025-10-22
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.action import ActionServer
import rclpy.duration

from geometry_msgs.msg import Pose, PointStamped
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float64, String

import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import tf2_ros

from arlab_knowledge_interfaces.srv import GetEntity, GetShape
from arlab_common_interfaces.srv import GrippingParameter
from arlab_common_interfaces.msg import OrchestratorData, ActionResponse
from arlab_common_interfaces.action import ManipulationAction


class Orchestrator(Node):
    """ROS2 Node for orchestrating robotic manipulation."""

    def __init__(self):
        super().__init__("Orchestrator")

        # Publisher for OrchestratorData
        self.data_publisher = self.create_publisher(
            OrchestratorData, "/orchestrator_data", 10
        )

        # Action Server for ManipulationAction
        self._action_server = ActionServer(
            self,
            ManipulationAction,
            '/manipulation_action',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        # Service clients
        self.service_group = MutuallyExclusiveCallbackGroup()
        prefix = "/arlab/knowledge"

        self.client_get_entity = self.create_client(
            GetEntity, f"{prefix}/get_entity", callback_group=self.service_group
        )
        self.client_get_shape = self.create_client(
            GetShape, f"{prefix}/get_shape", callback_group=self.service_group
        )
        self.client_gripping_parameter = self.create_client(
            GrippingParameter, "GetGrippingParameter", callback_group=self.service_group
        )

        # Wait for services
        for client, name in [
            (self.client_get_entity, "GetEntity"),
            (self.client_get_shape, "GetShape"),
            (self.client_gripping_parameter, "GrippingParameter")
        ]:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f"Waiting for {name} service...")

        # TF2 for transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Requests
        self.req_get_entity = GetEntity.Request()
        self.req_get_shape = GetShape.Request()
        self.req_gripping_parameter = GrippingParameter.Request()

        # Internal state
        self.entity_id = None
        self.command_type = "home"
        self.objectname = "default"
        self.pickable = False
        self.ref_frame = "camera_link"
        self.shape_pointcloud = None
        self.shape_boundingbox = None
        self.force = 5.0
        self.gripping_point_pos = [0.0, 0.0, 0.0]
        self.gripping_point_orient = [0.0, 0.0, 0.0, 1.0]

    # Action Goal Callback
    def goal_callback(self, goal_request):
        self.get_logger().info(
            (
            f"Data from Decision Making: cmd={goal_request.command.command_type}, "
            f"entity_id={goal_request.command.target_entityid}"
            )
        )
        return rclpy.action.GoalResponse.ACCEPT

    # Action Execute Callback
    def execute_callback(self, goal_handle):
        """Execute the manipulation command action."""
        goal_command = goal_handle.request.command
        self.command_type = goal_command.command_type
        self.entity_id = goal_command.target_entityid
        self.target_pose = getattr(goal_command, 'target_pose', None)

        # Call GetShape
        if self.command_type != "move":
            self.req_get_entity.entityid = self.entity_id
            future_entity = self.client_get_entity.call_async(self.req_get_entity)
            future_entity.add_done_callback(self.handle_get_entity_response)
        else:
            self.publish_goal()

        # Prepare Action Result
        result = ManipulationAction.Result()
        result.response = ActionResponse()
        result.response.message = "Goal accepted and processing"

        goal_handle.succeed()
        return result

    # GetEntity Response
    def handle_get_entity_response(self, future):
        try:
            response = future.result()
            entity = response.data
            self.get_logger().info(
            (
                f"Entity info received from knowledgebase: "
                f"name={entity.pickable.picking_tag}, "
                # f"group={entity.pickable.category}, "
                f"pose={entity.pose}"
            )
)

            if 2 == entity.entity_type.PICKABLE: #entity.entity_type.id instead of 2
                self.pickable = True
                self.objectname = "bottle" # entity.pickable.picking_tag
                self.objectgroup = "fruits" # entity.pickable.category
                self.pose = entity.pose
            else:
                self.pickable = False
                self.objectname = "noName"
                self.objectgroup = "default"

            self.ref_frame = entity.pose_reference_frame

            # Call GetShape
            self.req_get_shape.entityid = self.entity_id
            future_shape = self.client_get_shape.call_async(self.req_get_shape)
            future_shape.add_done_callback(self.handle_get_shape_response)

        except Exception as e:
            self.get_logger().error(f"GetEntity failed: {e}")
            self.pickable = False

    # GetShape Response
    def handle_get_shape_response(self, future):
        try:
            response = future.result()
            shape = response.shape
            if shape.has_pointcloud:
                self.pointcloud = shape.pointcloud
                self.get_logger().info("Pointcloud received from knowledgebase.")

            else:
                self.pointcloud = None
                self.get_logger().warn("No pointcloud received from knowledgebase.")

            if shape.has_boundingbox2d:
                self.boundingbox = shape.boundingbox2d
                self.get_logger().info("Boundingbox received from knowledgebase.")
            else:
                self.boundingbox = None
                self.get_logger().warn("No boundingbox received from knowledgebase.")

            # Call GrippingForce
            if self.pickable:
                self.req_gripping_parameter.objectgroup = self.objectgroup
                client = self.client_gripping_parameter
                future_parameter = client.call_async(self.req_gripping_parameter)
                future_parameter.add_done_callback(self.handle_gripping_parameter_response)
            else:
                self.get_logger().warn(
                    (
                        f"Entity '{self.objectname}' not pickable. "
                        "Skipping gripping force."
                    )
                )
                self.force = 0.0
                self.compute_gripping_pose()

        except Exception as e:
            self.get_logger().error(f"GetShape failed: {e}")

    # GrippingForce Response
    def handle_gripping_parameter_response(self, future):
        try:
            response = future.result()
            self.gripforce = response.gripforce
            self.grippos_mode = response.grippos_mode
            self.griporient_mode = response.griporient_mode
            self.get_logger().info(
                (
                    f"Gripping parameter received for '{self.objectgroup}': "
                    f"Gripping force = {self.gripforce}N, "
                    f"Gripping position mode = {self.grippos_mode}, "
                    f"Gripping orientation mode = {self.griporient_mode}"
                )
            )
        except Exception as e:
            self.get_logger().error(f"GetGrippingParameter failed: {e}")
            self.force = 5.0

        # Call Compute Pose
        self.compute_gripping_pose()

    # Compute Gripping Pose
    def compute_gripping_pose(self):

        # Compute position
        if self.grippos_mode == 0: # gripping in the middle of the object
            self.gripping_point_pos = self.pose.position

        if self.grippos_mode == 1: # thickest/thinnest place or center of gravity
            # TODO: implement logic in future
            pass

        # Compute orientation
        if self.griporient_mode == 0: # dont calculate orientation
            self.gripping_point_orient = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        elif self.griporient_mode == 1: # calculate orientation from bounding box
            if self.boundingbox:
                self.gripping_point_orient = self.compute_orientation(self.boundingbox)
            else:
                self.gripping_point_orient = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        elif self.griporient_mode == 2: # calculate orientation from pointcloud (future)
            # TODO: implement logic in future
            pass

        self.publish_goal()

    # Determine angle from bounding box dimensions
    def compute_orientation(self, bbox):
        if bbox.size_x >= bbox.size_y:
            angle = 0.0             # horicontal bbox
        else:
            angle = np.pi / 2       # vertical bbox

        qz = np.sin(angle / 2.0)
        qw = np.cos(angle / 2.0)
        return Quaternion(x=0.0, y=0.0, z=qz, w=qw)

    # Publish OrchestratorData
    def publish_goal(self):
        if self.command_type != "move":
            gripping_pose = Pose()
            gripping_pose.position = self.gripping_point_pos
            gripping_pose.orientation = self.gripping_point_orient
        else:
            gripping_pose = self.target_pose

        msg = OrchestratorData()
        msg.pose = gripping_pose
        msg.grip_force = Float64()
        msg.grip_force.data = self.force
        msg.cmd = String()
        msg.cmd.data = self.command_type

        self.data_publisher.publish(msg)

        self.get_logger().info(
            f"Published gripper goal: Pos=({gripping_pose.position}) "
            f"Orient=({gripping_pose.orientation}) "
            f"Force={self.force:.1f}N Cmd={self.command_type}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = Orchestrator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


