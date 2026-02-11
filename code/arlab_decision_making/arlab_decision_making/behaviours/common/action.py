"""Base behaviour for ROS2 action clients integrated with py_trees.

Provides a wrapper around 'py_tree_ros.action_clients.FromBlackboard' to:
    - configure a ROS2 action client for a given action type and name
    - write the action result on the py_tree blackboard under the key
        'result_output_key'

Maintainers:
    Peter Viechter <peter.viechter@uni-augsburg.de>
    Daniel Gabler <daniel.gabler@uni-augsburg.de>
"""

from typing import Any

import py_trees
import rclpy.task
from py_trees_ros.action_clients import FromBlackboard


# https://py-trees-ros.readthedocs.io/en/release-2.3.x/modules.html#py_trees_ros.action_clients.FromBlackboard
class RosActionBehaviour(FromBlackboard):
    """Generic ROS2 action behaviour integrated with py_trees using the blackboard
    interface.

    This class extends the `FromBlackboard` action client behaviour from
    `py_trees_ros` to additionally write the action result back to the blackboard
    under a specified key.
    """

    def __init__(
        self,
        name: str,
        action_type: Any,
        action_name: str,
        key: str,
        result_output_key: str,
    ):
        """Initialise the ROS2 action behaviour and blackboard key.

        Args:
            name (str): Name of the behaviour node.
            action_type (Any): ROS2 action type used by the client.
            action_name (str): Name/path of the action server.
            key (str): Blackboard key for the action goal.
            result_output_key (str): Blackboard key for storing the action result.
        """
        super().__init__(name, action_type, action_name, key)
        self.result_output_key = result_output_key
        # Register blackboard key for action result
        # if more than one action uses the same key, they override each other!
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
        """Handle action result and write it to the blackboard.
        Args:
            future (rclpy.task.Future): Future object containing the action result.
        """
        super().get_result_callback(future)

        self.blackboard.result_output = self.result_message.result
