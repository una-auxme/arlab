"""Home manipulation behaviours for py_trees with ROS2 action integration.

Provides a behaviour tree subtree to:
    - configure a home manipulation goal on the py_trees blackboard
    - execute the generic home manipulation action
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
    """Create the behaviour tree subtree for a home manipulation action.

    The subtree prepares the home command goal, executes the generic
    manipulation action, and checks whether the returned result indicates
    success.

    Returns:
        Behaviour: Root behaviour of the configured home manipulation
            subtree.
    """
    return Sequence(
        name="ManipulationHome",
        memory=True,
        children=[
            SetupManipulationHome(),
            GenericManipulation("Home", "/manipulation/home_goal", "/manipulation/home_result"),
            CheckManipulationHome(),
        ],
    )


class SetupManipulationHome(Behaviour):
    """Behaviour that writes a home manipulation goal to the blackboard.

    This behaviour prepares the goal message for the home manipulation
    action and stores it under the blackboard key
    `/manipulation/home_goal`.
    """

    def __init__(self):
        """Initialise the home manipulation goal setup behaviour."""
        super().__init__(name=type(self).__name__)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="/manipulation/home_goal", access=Access.WRITE)

    def update(self):
        """Create and store the configured home manipulation goal.

        Returns:
            Status: `Status.SUCCESS` after the goal has been written to the
                blackboard.
        """
        goal = ManipulationAction.Goal()
        goal.command.command_type = ManipulationCommand.COMMAND_HOME
        goal.command.target_entityid = 1
        self.blackboard.manipulation.home_goal = goal
        return Status.SUCCESS


class CheckManipulationHome(Behaviour):
    """Behaviour that validates the result of the home manipulation action.

    This behaviour reads the home manipulation result from the blackboard and
    returns success only if the action completed without an error message.
    """

    def __init__(self):
        """Initialise the home manipulation result checking behaviour."""
        super().__init__(name=type(self).__name__)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="/manipulation/home_result", access=Access.READ)

    def update(self):
        """Check the home manipulation result stored on the blackboard.

        Returns:
            Status: `Status.SUCCESS` if the home manipulation action
                succeeded, otherwise `Status.FAILURE`.
        """
        result: ManipulationAction.Result = self.blackboard.manipulation.home_result
        print(f"Home message: {result.response.message}")
        if result.response.error_code != ManipulationResponse.SUCCESS:
            return Status.FAILURE
        return Status.SUCCESS
