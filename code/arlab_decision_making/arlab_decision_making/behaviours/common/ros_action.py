from typing import Any
from abc import ABC, abstractmethod

from rclpy.node import Node

from py_trees.behaviour import Behaviour
from py_trees.common import Status


# TODO: https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html#getting-feedback
class RosActionBehaviour(Behaviour, ABC):
    def __init__(self, name: str, node: Node) -> None:
        super().__init__(name)
        self._node = node

    def setup(self, **kwargs: Any) -> None:
        pass

    def initialise(self) -> None:
        pass

    def update(self) -> Status:
        return Status.SUCCESS

    def terminate(self, new_status: Status) -> None:
        pass

    @abstractmethod
    def ros_init_request(self) -> Any:
        """Creates the request this ros action starts with

        Returns:
            Any: The request this ros action starts with
        """
        pass

    @abstractmethod
    async def feedback_callback(self, feedback: Any):
        pass
