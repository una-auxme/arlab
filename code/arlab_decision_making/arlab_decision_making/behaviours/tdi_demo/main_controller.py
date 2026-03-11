import math
import random
from copy import deepcopy
from typing import List, Optional, Tuple

import py_trees
from arlab_knowledge_interfaces.msg import (
    Entity,
    EntityType,
    Result,
)
from arlab_knowledge_interfaces.srv import GetEntities, GetEntity
from geometry_msgs.msg import Point, Pose
from rclpy.node import Node

from ..common.speech import queue_speech


def point_distance(p0: Point, p1: Point) -> float:
    return math.sqrt((p1.x - p0.x) ** 2 + (p1.y - p0.y) ** 2 + (p1.z - p0.z) ** 2)


class DatabaseBehaviour(py_trees.behaviour.Behaviour):
    def setup(self, **kwargs):
        try:
            self.node: Node = kwargs["node"]
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs"
            raise KeyError(error_message) from e  # 'direct cause' traceability

        self._get_entities_cli = self.node.create_client(
            GetEntities, "/arlab/knowledge/get_entities"
        )
        self._get_entity_cli = self.node.create_client(
            GetEntity, "/arlab/knowledge/get_entity"
        )

        for client in self.node.clients:
            while not client.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().warn(
                    f"Client {client.service_name} not available"
                )

        return super().setup(**kwargs)


class ChoosePickable(DatabaseBehaviour):
    def __init__(self, name: str, id_output_key: str):
        super().__init__(name)
        self.id_output_key = id_output_key

        self.blackboard = self.attach_blackboard_client(name)
        self.blackboard.register_key(
            key="id_output",
            access=py_trees.common.Access.WRITE,
            # make sure to namespace it if not already
            remap_to=py_trees.blackboard.Blackboard.absolute_name(
                "/", self.id_output_key
            ),
        )

    def update(self) -> py_trees.common.Status:
        # Get all pickable entities
        get_req = GetEntities.Request()
        get_req.entity_type.id = EntityType.PICKABLE
        response: GetEntities.Response = self._get_entities_cli.call(get_req)

        if response.result.result_type != Result.SUCCESS:
            self.node.get_logger().error(
                f"Failed to get entities: {response.result.error}"
            )
            return py_trees.common.Status.FAILURE
        entity_ids = response.entities
        found_entities: List[Tuple[int, Entity]] = []
        for entity_id in entity_ids:
            get_ent_req = GetEntity.Request()
            get_ent_req.entityid = entity_id
            response_ent: GetEntity.Response = self._get_entity_cli.call(get_ent_req)
            if response_ent.result.result_type != Result.SUCCESS:
                self.node.get_logger().warn(
                    f"Failed to get entity data: {response_ent.result.error}"
                )
                continue

            entity = response_ent.data
            if "mango" in entity.description.lower():
                # mangos are meh (also tables)
                continue
            found_entities.append((entity_id, entity))

        desc_str = ""
        chosen_entity = None
        for i, entity in enumerate(found_entities):
            desc_str += f"{entity[1].description}"
            if i < len(found_entities) - 2:
                desc_str += ", "
            elif i < len(found_entities) - 1:
                desc_str += " and "

            if i == 0:
                chosen_entity = entity

        if chosen_entity is None:
            queue_speech(
                self.blackboard,
                "I did not find any fruit. Please place fruit in front of me.",
            )
            return py_trees.common.Status.FAILURE

        queue_speech(
            self.blackboard,
            f"I have found: {desc_str}. "
            f"I choose to pick the {chosen_entity[1].description}.",
        )

        self.blackboard.id_output = chosen_entity[0]

        return py_trees.common.Status.SUCCESS


class ChoosePlacingPos(DatabaseBehaviour):
    def __init__(
        self,
        name: str,
        place_approach_pose_key: str,
        place_pose_key: str,
        blacklisted_positions_key: str,
        max_occupied_distance: float,
        place_poses: List[Pose],
        approach_z_offset: float,
        place_poses_names: Optional[List[str]] = None,
    ):
        super().__init__(name)
        self.place_approach_pose_key = place_approach_pose_key
        self.place_pose_key = place_pose_key
        self.blacklisted_positions_key = blacklisted_positions_key
        self.max_occupied_distance = max_occupied_distance
        self.place_poses = place_poses
        self.place_poses_names = place_poses_names
        self.approach_z_offset = approach_z_offset

        self.blackboard = self.attach_blackboard_client(name)
        self.blackboard.register_key(
            key="place_pose",
            access=py_trees.common.Access.WRITE,
            # make sure to namespace it if not already
            remap_to=py_trees.blackboard.Blackboard.absolute_name(
                "/", self.place_pose_key
            ),
        )
        self.blackboard.register_key(
            key="place_approach_pose",
            access=py_trees.common.Access.WRITE,
            # make sure to namespace it if not already
            remap_to=py_trees.blackboard.Blackboard.absolute_name(
                "/", self.place_approach_pose_key
            ),
        )
        self.blackboard.register_key(
            key="blacklisted_positions",
            access=py_trees.common.Access.READ,
            # make sure to namespace it if not already
            remap_to=py_trees.blackboard.Blackboard.absolute_name(
                "/", self.blacklisted_positions_key
            ),
        )

    def update(self) -> py_trees.common.Status:
        blacklisted_positions: List[int] = self.blackboard.blacklisted_positions
        shuffled_poses = random.sample(
            list(enumerate(self.place_poses)), len(self.place_poses)
        )

        # Get all pickable entities
        get_req = GetEntities.Request()
        get_req.entity_type.id = EntityType.PICKABLE
        response: GetEntities.Response = self._get_entities_cli.call(get_req)

        if response.result.result_type != Result.SUCCESS:
            self.node.get_logger().error(
                f"Failed to get entities: {response.result.error}"
            )
            return py_trees.common.Status.FAILURE
        entity_ids = response.entities
        found_entities: List[Tuple[int, Entity]] = []
        for entity_id in entity_ids:
            get_ent_req = GetEntity.Request()
            get_ent_req.entityid = entity_id
            response_ent: GetEntity.Response = self._get_entity_cli.call(get_ent_req)
            if response_ent.result.result_type != Result.SUCCESS:
                self.node.get_logger().warn(
                    f"Failed to get entity data: {response_ent.result.error}"
                )
                continue

            entity = response_ent.data
            if "mango" in entity.description.lower():
                # mangos are meh (also tables)
                continue
            found_entities.append((entity_id, entity))

        occupied_count = 0
        chosen_pose = None
        for i, pose in shuffled_poses:
            if i in blacklisted_positions:
                continue

            is_occupied = False
            for _, entity in found_entities:
                if (
                    point_distance(pose.position, entity.pose.position)
                    <= self.max_occupied_distance
                ):
                    is_occupied = True

            if is_occupied:
                occupied_count += 1
            else:
                chosen_pose = (i, pose)

        if chosen_pose is None:
            queue_speech(
                self.blackboard,
                "All positions seem to be occupied. "
                "Please remove the fruit from at least one position.",
            )
            return py_trees.common.Status.FAILURE

        queue_speech(
            self.blackboard,
            f"{occupied_count} out of {len(shuffled_poses)} positions are occupied.",
        )

        if self.place_poses_names is not None:
            pose_name = self.place_poses_names[chosen_pose[0]]
        else:
            pose_name = str(chosen_pose[0])
        queue_speech(
            self.blackboard,
            f"I choose to place the fruit in the {pose_name} position.",
        )

        place_pose = chosen_pose[1]
        self.blackboard.place_pose = place_pose
        place_approach_pose = deepcopy(place_pose)
        place_approach_pose.position.z += self.approach_z_offset
        self.blackboard.place_approach_pose = place_approach_pose

        return py_trees.common.Status.SUCCESS
