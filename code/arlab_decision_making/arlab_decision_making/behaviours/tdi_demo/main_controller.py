from typing import Any

import py_trees
import rclpy.task
from arlab_knowledge_interfaces.msg import Result
from arlab_knowledge_interfaces.srv import GetEntities, GetEntity
from py_trees.behaviour import Behaviour
from py_trees.composites import Sequence
from py_trees.timers import Timer
from py_trees_ros.action_clients import FromBlackboard
from rclpy.node import Node


def get_tree() -> Behaviour:
    return Sequence(
        "GrabSequence",
        memory=True,
        children=[
            Timer("InitialWait", 2.0),
            DemoController(
                name="Demo_picker",
            ),
            Timer("FinalWait", 2.0),
        ],
    )


class DemoController(py_trees.behaviour.Behaviour):
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
        return py_trees.common.Status.SUCCESS
