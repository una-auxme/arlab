"""Close manipulation behaviours for py_trees with ROS2 action integration.

Provides a behaviour tree subtree to:
    - configure a close manipulation goal on the py_trees blackboard
    - execute the generic close manipulation action
    - validate the returned manipulation result

Maintainers:
    Peter Viechter <peter.viechter@uni-augsburg.de>
    Daniel Gabler <daniel.gabler@uni-augsburg.de>
"""

from arlab_common_interfaces.action import ManipulationAction
from arlab_common_interfaces.msg import ManipulationCommand, ManipulationResponse
from py_trees.behaviour import Behaviour
from py_trees.common import Access, Status
from py_trees.composites import Sequence

from .generic_manipulation import GenericManipulation


def get_tree() -> Behaviour:
    """Create the behaviour tree subtree for a close manipulation action.

    The subtree prepares the close command goal, executes the generic
    manipulation action, and checks whether the returned result indicates
    success.

    Returns:
        Behaviour: Root behaviour of the configured close manipulation
            subtree.
    """
    return Sequence(
        name="ManipulationClose",
        memory=True,
        children=[
            SetHandClose(),
            GenericManipulation("Close", "/manipulation/close_goal", "/manipulation/close_result"),
            CheckHandClose(),
        ],
    )


class SetHandClose(Behaviour):
    """Behaviour that writes a close manipulation goal to the blackboard.

    This behaviour prepares the goal message for the close manipulation action
    and stores it under the blackboard key `/manipulation/close_goal`.
    """
    def __init__(self):
        """Initialise the close manipulation goal setup behaviour."""
        super().__init__(name=type(self).__name__)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="/manipulation/close_goal", access=Access.WRITE)

    def update(self):
        """Create and store the configured close manipulation goal.

        Returns:
            Status: `Status.SUCCESS` after the goal has been written to the
                blackboard.
        """
        goal = ManipulationAction.Goal()
        goal.command.command_type = ManipulationCommand.COMMAND_CLOSE
        self.blackboard.manipulation.close_goal = goal
        return Status.SUCCESS


class CheckHandClose(Behaviour):
    """Behaviour that validates the result of the close manipulation action.

    This behaviour reads the close manipulation result from the blackboard and
    returns success only if the action completed without an error message.
    """
    def __init__(self):
        """Initialise the close manipulation result checking behaviour."""
        super().__init__(name=type(self).__name__)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="/manipulation/close_result", access=Access.READ)

    def update(self):
        """Check the close manipulation result stored on the blackboard.

        Returns:
            Status: `Status.SUCCESS` if the close manipulation action
                succeeded, otherwise `Status.FAILURE`.
        """
        result: ManipulationAction.Result = self.blackboard.manipulation.close_result
        print(f"close message: {result.response.message}")
        if result.response.error_code != ManipulationResponse.SUCCESS:
            return Status.FAILURE
        return Status.SUCCESS
