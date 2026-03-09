from typing import Any, List, Tuple

import py_trees
import rclpy.task
from arlab_knowledge_interfaces.msg import (
    Entity,
    EntityPickable,
    EntityType,
    Result,
    Shape,
)
from arlab_knowledge_interfaces.srv import GetEntities, GetEntity
from py_trees.behaviour import Behaviour
from py_trees.composites import Sequence
from py_trees.timers import Timer
from py_trees_ros.action_clients import FromBlackboard
from rclpy.node import Node


class MainController(py_trees.behaviour.Behaviour):
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
                desc_str += "and"

            if i == 0:
                chosen_entity = entity

        if chosen_entity is None:
            return py_trees.common.Status.FAILURE

        self.blackboard.id_output = chosen_entity[0]

        return py_trees.common.Status.SUCCESS
