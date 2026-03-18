"""Pick manipulation behaviours for py_trees with ROS2 action integration.

Provides a behaviour tree subtree to:
    - configure a pick manipulation goal on the py_trees blackboard
    - use either a blackboard input key or a fixed target entity ID
    - execute the generic pick manipulation action
    - validate the returned manipulation result

Maintainers:
    Peter Viechter <peter.viechter@uni-augsburg.de>
    Daniel Gabler <daniel.gabler@uni-augsburg.de>
"""

from typing import Optional

import py_trees
from arlab_common_interfaces.action import ManipulationAction
from arlab_common_interfaces.msg import ManipulationCommand, ManipulationResponse
from py_trees.behaviour import Behaviour
from py_trees.common import Access, Status
from py_trees.composites import Sequence

from .generic_manipulation import GenericManipulation


def get_tree(id_input_key: Optional[str] = None, fixed_id: Optional[int] = None) -> Behaviour:
    """Create the behaviour tree subtree for a pick manipulation action.

    The subtree prepares the pick command goal, executes the generic
    manipulation action, and checks whether the returned result indicates
    success.

    The target entity ID can either be read from the blackboard or provided
    as a fixed value.

    Args:
        id_input_key (Optional[str], optional): Blackboard key from which the
            target entity ID is read. Defaults to None.
        fixed_id (Optional[int], optional): Fixed target entity ID for the
            pick command. Defaults to None.

    Returns:
        Behaviour: Root behaviour of the configured pick manipulation
            subtree.
    """
    return Sequence(
        name="ManipulationPick",
        memory=True,
        children=[
            SetHandPick(id_input_key=id_input_key, fixed_id=fixed_id),
            GenericManipulation("Pick", "/manipulation/pick_goal", "/manipulation/pick_result"),
            CheckHandPick(),
        ],
    )


class SetHandPick(Behaviour):
    """Behaviour that writes a pick manipulation goal to the blackboard.

    This behaviour prepares the goal message for the pick manipulation
    action and stores it under the blackboard key
    `/manipulation/pick_goal`.

    The target entity ID is taken either from a configured blackboard key
    or from a fixed ID passed during initialisation.
    """
    def __init__(self, id_input_key: Optional[str] = None, fixed_id: Optional[int] = None):
        """Initialise the pick manipulation goal setup behaviour.

        Args:
            id_input_key (Optional[str], optional): Blackboard key from which
                the target entity ID is read. Defaults to None.
            fixed_id (Optional[int], optional): Fixed target entity ID for the
                pick command. Defaults to None.
        """
        super().__init__(name=type(self).__name__)
        self.fixed_id = fixed_id
        self.id_input_key = id_input_key

        self.blackboard = self.attach_blackboard_client(name=self.name)
        if self.id_input_key is not None:
            self.blackboard.register_key(
                key="id_input",
                access=Access.READ,
                # make sure to namespace it if not already
                remap_to=py_trees.blackboard.Blackboard.absolute_name("/", self.id_input_key),
            )
        self.blackboard.register_key(key="/manipulation/pick_goal", access=Access.WRITE)

    def update(self):
        """Create and store the configured pick manipulation goal.

        Returns:
            Status: `Status.SUCCESS` after the goal has been written to the
                blackboard, or `Status.FAILURE` if no target entity ID is
                available.
        """
        goal = ManipulationAction.Goal()
        goal.command.command_type = ManipulationCommand.COMMAND_PICK
        if self.id_input_key is not None:
            goal.command.target_entityid = self.blackboard.id_input
        elif self.fixed_id is not None:
            goal.command.target_entityid = self.fixed_id
        else:
            return Status.FAILURE
        self.blackboard.manipulation.pick_goal = goal
        return Status.SUCCESS


class CheckHandPick(Behaviour):
    """Behaviour that validates the result of the pick manipulation action.

    This behaviour reads the manipulation result from the blackboard and
    returns success only if the action completed successfully.
    """
    def __init__(self):
        """Initialise the pick manipulation result checking behaviour."""
        super().__init__(name=type(self).__name__)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="/manipulation/pick_result", access=Access.READ)

    def update(self):
        """Check the pick manipulation result stored on the blackboard.

        Returns:
            Status: `Status.SUCCESS` if the pick manipulation action
                succeeded, otherwise `Status.FAILURE`.
        """
        result: ManipulationAction.Result = self.blackboard.manipulation.pick_result
        print(f"pick message: {result.response.message}")
        if result.response.error_code != ManipulationResponse.SUCCESS:
            return Status.FAILURE
        return Status.SUCCESS
