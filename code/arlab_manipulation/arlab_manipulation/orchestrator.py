#!/usr/bin/env python3
"""
orchestrator.py
---------------

ROS2 Node 'orchestrator' that subscribes to manipulation commands via Action,
queries the gripping force service, computes gripping poses, and publishes orchestrator
data.

Author: Sofia Öttl
Date: 2025-10-22
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.action.server import ActionServer, GoalResponse
import rclpy.duration

from geometry_msgs.msg import Pose, Point
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float64, String























































import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import tf2_ros
import matplotlib.pyplot as plt

from arlab_knowledge_interfaces.srv import GetEntity, GetShape
from arlab_common_interfaces.srv import GrippingParameter
from arlab_common_interfaces.msg import OrchestratorData, ActionResponse
from arlab_common_interfaces.action import ManipulationAction


class orchestrator(Node):
    """ROS2 Node for orchestrating robotic manipulation."""

    def __init__(self):
        super().__init__("orchestrator")

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
        self.object_name = "default"
        self.object_group = "default"
        self.pickable = False
        self.ref_frame = "camera_link"
        self.shape_pointcloud = None
        self.shape_boundingbox = None
        self.force = 5.0
        self.gripping_point_pos = Point(x=0.0, y=0.0, z=0.0)
        self.gripping_point_orient = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.placing_point_pos = Point(x=0.0, y=0.0, z=0.0)
        self.placing_point_orient = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.voxel_map = None

    # Action Goal Callback
    def goal_callback(self, goal_request):
        self.get_logger().info(
            (
            f"Data from Decision Making: cmd={goal_request.command.command_type}, "
            f"entity_id={goal_request.command.target_entityid}"
            )
        )
        return GoalResponse.ACCEPT

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
                self.object_name = "bottle" # entity.pickable.picking_tag
                self.object_group = "fruits" # entity.pickable.category
                self.pose = entity.pose
            else:
                self.pickable = False
                self.object_name = "noName"
                self.object_group = "default"

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
            self.point_cloud = shape.pointcloud if shape.has_pointcloud else None
            self.get_logger().info(
                "Pointcloud received." if self.point_cloud is not None
                else "No pointcloud received."
            )

            self.bounding_box = shape.boundingbox2d if shape.has_boundingbox2d else None
            self.get_logger().info(
                "Boundingbox received." if self.bounding_box is not None
                else "No boundingbox received."
            )

            # Create VoxelMap
            if self.command_type == "place":
                voxel_map_result = self.pointcloud_to_voxel_map(self.point_cloud)
                if voxel_map_result is not None:
                    self.get_logger().warn("Voxel map creation successful.")
                    self.voxel_map, self.voxel_orig, self.voxel_size = voxel_map_result
                    self.visualize_voxel_map(self.voxel_map)
                else:
                    self.get_logger().warn("Voxel map creation failed.")


            # Call GrippingForce
            if self.pickable:
                self.req_gripping_parameter.objectgroup = self.object_group
                client = self.client_gripping_parameter
                future_parameter = client.call_async(self.req_gripping_parameter)
                future_parameter.add_done_callback(self.handle_gripping_parameter_response)
            else:
                self.get_logger().warn(
                    (
                        f"Entity '{self.object_name}' not pickable. "
                        "Skipping gripping force."
                    )
                )
                self.force = 0.0
                self.compute_goal_pose()

        except Exception as e:
            self.get_logger().error(f"GetShape failed: {e}")

    # GrippingForce Response
    def handle_gripping_parameter_response(self, future):
        try:
            response = future.result()
            self.grip_force = response.gripforce
            self.grip_pos_mode = response.grippos_mode
            self.grip_orient_mode = response.griporient_mode
            self.get_logger().info(
                (
                    f"Gripping parameter received for '{self.object_group}': "
                    f"Gripping force = {self.grip_force}N, "
                    f"Gripping position mode = {self.grip_pos_mode}, "
                    f"Gripping orientation mode = {self.grip_orient_mode}"
                )
            )
        except Exception as e:
            self.get_logger().error(f"GetGrippingParameter failed: {e}")
            self.force = 5.0

        # Call Compute Goal Pose
        self.compute_goal_pose()

    # Compute Goal Pose
    def compute_goal_pose(self):

        if self.command_type == "pick":
            # Compute position
            if self.grip_pos_mode == 0: # gripping in the middle of the object
                self.gripping_point_pos = self.pose.position

            if self.grip_pos_mode == 1: # thickest/thinnest place or center of gravity
                # TODO: implement logic in future
                pass

            # Compute orientation
            if self.grip_orient_mode == 0: # dont calculate orientation
                self.gripping_point_orient = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            elif self.grip_orient_mode == 1: # calculate orientation from bounding box
                if self.bounding_box:
                    self.gripping_point_orient = self.compute_orientation(self.bounding_box)
                else:
                    self.gripping_point_orient = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            elif self.grip_orient_mode == 2: # calculate orientation from pointcloud
                # TODO: implement logic in future
                pass

            self.publish_goal()

        elif self.command_type == "place":
            if self.voxel_map is not None:
                pos_list = self.find_placing_point(self.voxel_map)
                self.placing_point_pos = Point(x=pos_list[0], y=pos_list[1], z=pos_list[2])
            else:
                self.get_logger().warn("Voxel map not available, using default.")
                self.placing_point_pos = Point(x=0.0, y=0.0, z=0.0)

            self.placing_point_orient = self.gripping_point_orient
            self.publish_goal()

    # Determine angle from bounding box dimensions
    def compute_orientation(self, bbox):
        if bbox.size_x >= bbox.size_y:
            angle = 0.0             # horizontal bbox
        else:
            angle = np.pi / 2       # vertical bbox

        qz = np.sin(angle / 2.0)
        qw = np.cos(angle / 2.0)
        return Quaternion(x=0.0, y=0.0, z=qz, w=qw)

    # Transfer pointcloud into voxel map
    def pointcloud_to_voxel_map(self, pointcloud, voxel_size=0.01):

        if pointcloud is None or len(pointcloud) == 0:
            self.get_logger().warn("Empty pointcloud, cannot create voxel map.")
            return None

        # Move Pointcloud in positive Area
        min_coords = np.min(pointcloud, axis=0)
        shifted = pointcloud - min_coords

        # Dimension of Voxelmap
        dims = np.ceil(np.max(shifted, axis=0) / voxel_size).astype(int)
        voxel_map = np.zeros(dims, dtype=np.uint8)

        # Calculate Indices
        indices = np.floor(shifted / voxel_size).astype(int)
        voxel_map[indices[:,0], indices[:,1], indices[:,2]] = 1

        return voxel_map, min_coords, voxel_size

    # Find a placing point using the voxel map
    def find_placing_point(self, voxel_map):
        """
        Find a free voxel suitable for placing the current object.
        Returns the voxel coordinates (as list) or [0,0,0] if none available.
        """
        free_indices = np.argwhere(voxel_map == 0)
        if free_indices.size == 0:
            self.get_logger().warn("No free voxels found, defaulting to origin")
            return [0.0, 0.0, 0.0]

        # Choose the highest free level (Z) at first
        highest_voxel = free_indices[np.argmax(free_indices[:, 2])]
        return highest_voxel.tolist()

    # Show Voxel Map
    def visualize_voxel_map(self, voxel_map):
        """
        Visualizes the voxel map using matplotlib 3D scatter plot.
        """
        filled = np.argwhere(voxel_map == 1)
        if filled.size == 0:
            print("Voxelmap is empty.")
            return

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(filled[:,0], filled[:,1], filled[:,2], c='blue', marker='s', s=10)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        plt.show()

    # Publish OrchestratorData
    def publish_goal(self):
        if self.command_type != "move":
            if self.command_type == "pick":
                goal_pose = Pose()
                goal_pose.position = self.gripping_point_pos
                goal_pose.orientation = self.gripping_point_orient
            elif self.command_type == "place":
                goal_pose = Pose()
                goal_pose.position = self.placing_point_pos
                goal_pose.orientation = self.placing_point_orient
        else:
            goal_pose = self.target_pose or Pose()

        msg = OrchestratorData()
        msg.pose = goal_pose
        msg.grip_force = Float64()
        msg.grip_force.data = self.force
        msg.cmd = String()
        msg.cmd.data = self.command_type

        self.data_publisher.publish(msg)

        self.get_logger().info(
            f"Published gripper goal: Pos=({goal_pose.position}) "
            f"Orient=({goal_pose.orientation}) "
            f"Force={self.force:.1f}N Cmd={self.command_type}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = orchestrator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


