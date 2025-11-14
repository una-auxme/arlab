"""Generic movement action wrapper for robot motion commands.

Provides a reusable ROS2 action behavior that sends `MovementAction` goals
(base motion) via a common action server path.

This class is typically used inside behavior trees or higher-level task
coordination nodes to trigger motion primitives in a uniform way.

Maintainers:
    Peter Viechter <peter.viechter@uni-augsburg.de>
    Daniel Gabler <daniel.gabler@uni-augsburg.de>
"""

from arlab_common_interfaces.action import MovementAction

from ...utils.constants import MOVEMENT_ACTION_PATH
from ..common.action import RosActionBehaviour


class GenericMovement(RosActionBehaviour):
    """Generic ROS2 action behaviour for motion-related tasks.

    This class wraps the `MovementAction` action server located at
    `MOVEMENT_ACTION_PATH` and is intended to be plugged into a behavior
    tree framework.

    Args:
        name (str): Human-readable name of this behavior (for logs/BT tools).
        key (str): Blackboard key used to read the goal input for the action.
        result_output_key (str): Blackboard key under which the action result
            will be stored after completion.

    Attributes:
        action_type: The ROS2 action type used (`MovementAction`).
        action_name (str): The resolved action server name/path.
        key (str): Blackboard key for the action goal.
        result_output_key (str): Blackboard key for the action result.

    Notes:
        - The parent class `RosActionBehaviour` is responsible for:
            * creating the action client,
            * sending goals and handling feedback,
            * writing the result back to the blackboard.
        - `GenericMovement` only configures the base class with the correct
          action type and server name.
    """

    def __init__(self, name: str, key: str, result_output_key: str) -> None:
        """Initialize the generic movement action behaviour.

        Args:
            name: Name of the behavior node (for debugging/visualization).
            key: Blackboard key used as input for the `MovementAction` goal.
            result_output_key: Blackboard key under which the result will be
                written once the action has finished.
        """
        super().__init__(
            name,
            action_type=MovementAction,
            action_name=MOVEMENT_ACTION_PATH,
            key=key,
            result_output_key=result_output_key,
        )