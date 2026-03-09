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
from std_msgs.msg import Float64, String

from .utils.octomap_utils import find_placing_area
from .utils.transform_utils import (
    transform_pose,  # transform_bBox, transform_pointCloud,
)


class orchestrator(Node):
    """ROS2 Node for orchestrating robotic manipulation."""

    def __init__(self):
        super().__init__("orchestrator")
        self.service_group = MutuallyExclusiveCallbackGroup()

        self._orchestrator_client = ActionClient(
            self,
            OrchestratorAction,
            "/orchestrator/action",
            callback_group=self.service_group,
        )

        self._action_server = ActionServer(
            self,
            ManipulationAction,
            "/manipulation/action",
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

        self.subscription = self.create_subscription(
            PlanningScene, "/monitored_planning_scene", self.octomap_callback, 10
        )

        # Inits
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
        self.octomap = None
        self.action_result = None
        self.err = ManipulationResponse.UNDEFINED
        self.msg = ""

    # Get Octomap from MoveIt Node
    def octomap_callback(self, msg: PlanningScene):
        octomap_with_pose = msg.world.octomap
        octomap_msg = octomap_with_pose.octomap
        self.octomap = octomap_msg.data
        if not self.octomap:
            # self.get_logger().warn("Octomap is empty")
            # self.err = -40
            # self.msg = "Octomap is empty"
            # self.finish_action()
            return
        self.get_logger().info("Octomap received")

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
        self.action_result.response = ManipulationResponse()

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
        self.action_done_event.clear()

        if not goal_handle.is_active:
            return

        self.get_logger().info("Action response send to desicion maker")
        self.action_result.response.error_code = self.err
        self.action_result.response.message = self.msg

        goal_handle.succeed()

        return self.action_result

    # GetEntity Response
    def handle_get_entity_response(self, future):
        try:
            response = future.result()
            entity = response.data
            self.get_logger().info(
                (
                    f"Entity info received: name={entity.pickable.object_name},\n"
                    f"pose={entity.pose}"
                )
            )

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

            self.pose, err, msg = transform_pose(
                self.tf_buffer, cast(Pose, self.pose), self.stamp, self.ref_frame
            )

            if err == 1:
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

    # GetShape Response
    def handle_get_shape_response(self, future):
        try:
            response = future.result()
            shape = response.shape
            self.point_cloud = shape.pointcloud if shape.has_pointcloud else None
            self.bounding_box = shape.boundingbox2d if shape.has_boundingbox2d else None

            # TODO: FIX
            # self.point_cloud, err, msg = transform_pointCloud(
            #     self.tf_buffer, self.point_cloud, self.stamp, self.ref_frame
            # )

            # self.bounding_box, err, msg = transform_bBox(
            #     self.tf_buffer, self.bounding_box, self.stamp, self.ref_frame
            # )

            if self.pickable:
                self.req_gripping_parameter = GrippingParameter.Request()
                self.req_gripping_parameter.objectgroup = self.object_group
                future_param = self.client_gripping_parameter.call_async(
                    self.req_gripping_parameter
                )
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

    # GetGrippingParameter Response
    def handle_gripping_parameter_response(self, future):
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

    # Compute Goal Pose for MoveIt Node
    def compute_goal_pose(self):
        if self.command_type == "pick":
            if self.pose is not None:
                self.gripping_point_pos = self.pose.position
                # Just static offsets + orientation for now
                self.gripping_point_pos.x += 0.03
                self.gripping_point_pos.y -= 0.10
                self.gripping_point_pos.z = 0.185
                self.gripping_point_orient = Quaternion(
                    x=0.623141, y=-0.57889, z=-0.458192, w=-0.258152
                )
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
                    margin=0.02,  # safety offset
                    lift=0.01,  # offset shelf
                    offset_x=0.05,  # offset gripper side
                    offset_y=0.05,  # offset gripper front
                )
                if err == 1:
                    self.placing_point_pos = Point(
                        x=pose.position.x, y=pose.position.y, z=pose.position.z
                    )
                    self.placing_point_orient = self.gripping_point_orient
                    self.send_goal()
                else:
                    self.err = err
                    self.msg = self.msg
                    self.finish_action()

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
            self.get_logger().error("OrchestratorAction server is not available")
            self.err = -44
            self.msg = "Orchestrator Actionserver is not available"
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
                self.err = -45
                self.msg = "Orchestrator Actiongoal from MoveIt rejected"
                self.finish_action()

            self.get_logger().info("Orchestrator goal accepted")
            self.goal_handle.get_result_async().add_done_callback(
                self.handle_orchestrator_result
            )

        except Exception as e:
            self.get_logger().error(f"Failed to receive orchestrator goal: {e}")
            self.err = -46
            self.msg = "Failed to receive orchestrator goal"
            self.finish_action()

    # OrchestratorAction Result
    def handle_orchestrator_result(self, future):
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
