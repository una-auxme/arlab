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
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.action.server import ActionServer, GoalResponse
from rclpy.action.client import ActionClient
import rclpy.executors

from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Float64, String

import numpy as np
import tf2_ros

from arlab_knowledge_interfaces.srv import GetEntity, GetShape
from arlab_common_interfaces.srv import GrippingParameter
from arlab_common_interfaces.action import ManipulationAction, OrchestratorAction

from .utils.transform_utils import transform_pose, transform_pointCloud, transform_bBox
from .utils.voxel_utils import find_placing_area, visualize_voxel_map

from time import sleep
from threading import Event


class orchestrator(Node):
    """ROS2 Node for orchestrating robotic manipulation."""

    def __init__(self):
        super().__init__("orchestrator")
        self.service_group = MutuallyExclusiveCallbackGroup()

        self._orchestrator_client = ActionClient(self, OrchestratorAction,
                                                 '/orchestrator/action', callback_group=self.service_group)

        self._action_server = ActionServer(
            self,
            ManipulationAction,
            '/manipulation/action',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
        )

        self.action_done_event = Event()

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
        self.client_create_voxelmap = self.create_client(
            CreateVoxelmal, "CreateVoxelmap", callback_group=self.service_group
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

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
        self.voxel_map = None
        self.action_result = None

    # Accept Goal from ManipulationAction
    def goal_callback(self, goal_request):
        self.get_logger().info(
            f"Data from Decision Making: cmd={goal_request.command.command_type}, "
            f"entity_id={goal_request.command.target_entityid}"
        )
        return GoalResponse.ACCEPT

    # Get data from ManipulationAction
    def execute_callback(self, goal_handle):
        """Execute the manipulation command action."""

        self.action_result = ManipulationAction.Result()

        goal_command = goal_handle.request.command
        self.command_type = goal_command.command_type
        self.entity_id = goal_command.target_entityid
        self.target_pose = getattr(goal_command, "target_pose", None)

        if self.command_type in ["pick", "place"]:
            self.req_get_entity = GetEntity.Request()
            self.req_get_entity.entityid = self.entity_id
            future_entity = self.client_get_entity.call_async(self.req_get_entity)
            future_entity.add_done_callback(self.handle_get_entity_response)
        else:
            self.send_goal()

        self.action_done_event.wait()

        if not goal_handle.is_active:
            return

        self.get_logger().info("Action response send to desicion maker")
        if self.msg == "done":
            self.action_result.response.message = "SUCCESS"
        else:
            self.action_result.response.message = "ERROR"

        goal_handle.succeed()

        return self.action_result

    # GetEntity Response
    def handle_get_entity_response(self, future):
        try:
            response = future.result()
            entity = response.data
            self.get_logger().info(
                (
                    f"Entity info received: name={entity.pickable.picking_tag},\n"
                    f"pose={entity.pose}"
                )
            )

            if entity.entity_type.PICKABLE == 2:
                self.pickable = True
                self.object_name = entity.pickable.picking_tag
                self.object_group = entity.pickable.category
                self.pose = entity.pose
            else:
                self.pickable = False
                self.object_name = "noName"
                self.object_group = "default"

            self.ref_frame = entity.pose_reference_frame
            self.stamp = entity.stamp

            self.pose = transform_pose(self.tf_buffer, self.pose, self.stamp,
                                       self.ref_frame)

            self.req_get_shape = GetShape.Request()
            self.req_get_shape.entityid = self.entity_id
            future_shape = self.client_get_shape.call_async(self.req_get_shape)
            future_shape.add_done_callback(self.handle_get_shape_response)

        except Exception as e:
            self.get_logger().error(f"GetEntity failed: {e}")
            self.pickable = False
            self.send_goal()

    # GetShape Response
    def handle_get_shape_response(self, future):
        try:
            response = future.result()
            shape = response.shape
            self.point_cloud = shape.pointcloud if shape.has_pointcloud else None
            self.bounding_box = shape.boundingbox2d if shape.has_boundingbox2d else None

            if self.point_cloud:
                self.point_cloud = transform_pointCloud(self.tf_buffer,
                                                        self.point_cloud,
                                                        self.stamp,
                                                        self.ref_frame)
            if self.bounding_box:
                self.bounding_box = transform_bBox(self.tf_buffer,
                                                   self.bounding_box,
                                                   self.stamp,
                                                   self.ref_frame)

            if self.command_type == "place" and self.point_cloud:
                voxel_map_result = pointcloud_to_voxel_map(self.tf_buffer,
                                                           self.point_cloud)
                if voxel_map_result:
                    self.voxel_map, self.voxel_orig, self.voxel_size = voxel_map_result
                    visualize_voxel_map(self.tf_buffer, self.voxel_map)

            if self.pickable:
                self.req_gripping_parameter = GrippingParameter.Request()
                self.req_gripping_parameter.objectgroup = self.object_group
                future_param = self.client_gripping_parameter.call_async(
                    self.req_gripping_parameter)
                future_param.add_done_callback(self.handle_gripping_parameter_response)
            else:
                self.force = 0.0
                self.compute_goal_pose()

        except Exception as e:
            self.get_logger().error(f"GetShape failed: {e}")
            self.compute_goal_pose()

    # GetGrippingParameter Response
    def handle_gripping_parameter_response(self, future):
        try:
            response = future.result()
            self.force = response.gripforce
            self.grip_pos_mode = response.grippos_mode
            self.grip_orient_mode = response.griporient_mode
        except Exception as e:
            self.get_logger().error(f"GetGrippingParameter failed: {e}")
            self.force = 5.0
            self.grip_pos_mode = self.grip_orient_mode = 0

        future_param = self.client_gripping_parameter.call_async(
            self.req_gripping_parameter)
        future_param.add_done_callback(self.handle_gripping_parameter_response)

        self.compute_goal_pose()

    # CreateVoxelmap Response
    def handle_create_voxelmap_response(self, future):
        try:
            response = future.result()
            self.force = response.gripforce
            self.grip_pos_mode = response.grippos_mode
            self.grip_orient_mode = response.griporient_mode
        except Exception as e:
            self.get_logger().error(f"CreateVoxelmap failed: {e}")
            self.force = 5.0
            self.grip_pos_mode = self.grip_orient_mode = 0

        self.compute_goal_pose()

    # Compute Goal Pose for MoveIt Node
    def compute_goal_pose(self):
        if self.command_type == "pick":
            self.gripping_point_pos = self.pose.position
            self.gripping_point_orient = Quaternion(w=1.0)
        elif self.command_type == "place":
            if self.voxel_map:
                pos_list = find_placing_area(self.tf_buffer,
                                             self.voxel_map,
                                             self.bounding_box)
                self.placing_point_pos = Point(x=pos_list[0],
                                               y=pos_list[1],
                                               z=pos_list[2])
            self.placing_point_orient = self.gripping_point_orient

        self.send_goal()

    # Publish Goal Pose with ActionClient
    def send_goal(self):
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

        msg = OrchestratorAction.Goal()
        msg.data.pose = goal_pose
        msg.data.grip_force = Float64(data=self.force)
        msg.data.cmd = String(data=self.command_type)

        if not self._orchestrator_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("OrchestratorAction server not available")
            self.msg = "error"
            self.finish_action()
            return

        send_future = self._orchestrator_client.send_goal_async(msg)
        send_future.add_done_callback(self.handle_orchestrator_response)

    # OrchestratorAction Response
    def handle_orchestrator_response(self, future):
        try:
            self.goal_handle = future.result()
            if not self.goal_handle.accepted:
                self.get_logger().error("Orchestrator goal rejected")
                self.msg = "error"
                self.finish_action()

            self.get_logger().info("Orchestrator goal accepted")
            self.goal_handle.get_result_async().add_done_callback(self.handle_orchestrator_result)

        except Exception as e:
            self.get_logger().error(f"Failed to send orchestrator goal: {e}")
            self.msg = "error"
            self.finish_action()

    # OrchestratorAction Result
    def handle_orchestrator_result(self, future):
        try:
            self.msg = future.result().result.response.message
            self.get_logger().info(f"Orchestrator action completed: {self.msg}")
            self.finish_action()

        except Exception as e:
            self.get_logger().error(f"Orchestrator action failed: {e}")
            self.msg = "error"
            self.finish_action()

    # Finish ManipulationAction --> send Result
    def finish_action(self):
        self.action_done_event.set()


def main(args=None):
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
