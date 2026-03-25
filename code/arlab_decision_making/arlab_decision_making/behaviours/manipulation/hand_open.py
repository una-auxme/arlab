"""Open manipulation behaviours for py_trees with ROS2 action integration.

Provides a behaviour tree subtree to:
    - configure an open manipulation goal on the py_trees blackboard
    - execute the generic open manipulation action
    - validate the returned manipulation result

Maintainers:
    Peter Viechter <peter.viechter@uni-a.de>
    Daniel Gabler <daniel.gabler@uni-a.de>
"""

from arlab_common_interfaces.action import ManipulationAction
from arlab_common_interfaces.msg import ManipulationCommand, ManipulationResponse
from py_trees.behaviour import Behaviour
from py_trees.common import Access, Status
from py_trees.composites import Sequence

from .generic_manipulation import GenericManipulation


def get_tree() -> Behaviour:
    """Create the behaviour tree subtree for an open manipulation action.

    The subtree prepares the open command goal, executes the generic
    manipulation action, and checks whether the returned result indicates
    success.

    Returns:
        Behaviour: Root behaviour of the configured open manipulation
            subtree.
    """
    return Sequence(
        name="ManipulatorOpen",
        memory=True,
        children=[
            SetHandOpen(),
            GenericManipulation("Open", "/manipulation/open_goal", "/manipulation/open_result"),
            CheckHandOpen(),
        ],
    )


class SetHandOpen(Behaviour):
    """Behaviour that writes an open manipulation goal to the blackboard.

    This behaviour prepares the goal message for the open manipulation
    action and stores it under the blackboard key
    `/manipulation/open_goal`.
    """

    def __init__(self):
        """Initialise the open manipulation goal setup behaviour."""
        super().__init__(name=type(self).__name__)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="/manipulation/open_goal", access=Access.WRITE)

    def update(self):
        """Create and store the configured open manipulation goal.

        Returns:
            Status: `Status.SUCCESS` after the goal has been written to the
                blackboard.
        """
        goal = ManipulationAction.Goal()
        goal.command.command_type = ManipulationCommand.COMMAND_OPEN
        self.blackboard.manipulation.open_goal = goal
        return Status.SUCCESS


class CheckHandOpen(Behaviour):
    """Behaviour that validates the result of the open manipulation action.

    This behaviour reads the manipulation result from the blackboard and
    returns success only if the action completed successfully.
    """

    def __init__(self):
        """Initialise the open manipulation result checking behaviour."""
        super().__init__(name=type(self).__name__)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="/manipulation/open_result", access=Access.READ)

    def update(self):
        """Check the open manipulation result stored on the blackboard.

        Returns:
            Status: `Status.SUCCESS` if the open manipulation action
                succeeded, otherwise `Status.FAILURE`.
        """
        result: ManipulationAction.Result = self.blackboard.manipulation.open_result
        print(f"open message: {result.response.message}")
        if result.response.error_code != ManipulationResponse.SUCCESS:
            return Status.FAILURE
        return Status.SUCCESS
