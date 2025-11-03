from typing import Any

import py_trees
import rclpy.task
from py_trees_ros.action_clients import FromBlackboard


# https://py-trees-ros.readthedocs.io/en/release-2.3.x/modules.html#py_trees_ros.action_clients.FromBlackboard
class RosActionBehaviour(FromBlackboard):
    def __init__(
        self,
        name: str,
        action_type: Any,
        action_name: str,
        key: str,
        result_output_key: str,
    ):
        super().__init__(name, action_type, action_name, key)
        self.result_output_key = result_output_key

        self.blackboard.register_key(
            key="result_output",
            access=py_trees.common.Access.WRITE,
            # make sure to namespace it if not already
            remap_to=py_trees.blackboard.Blackboard.absolute_name(
                "/", result_output_key
            ),
        )
        self.blackboard.result_output = None

    def get_result_callback(self, future: rclpy.task.Future):
        super().get_result_callback(future)

        self.blackboard.result_output = self.result_message
