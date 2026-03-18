"""Move manipulation behaviours for py_trees with ROS2 action integration.

Provides behaviour tree subtrees to:
    - configure a move manipulation goal on the py_trees blackboard
    - use either a fixed target pose or a pose read from the blackboard
    - execute the generic move manipulation action
    - validate the returned manipulation result

Maintainers:
    Peter Viechter <peter.viechter@uni-augsburg.de>
    Daniel Gabler <daniel.gabler@uni-augsburg.de>
"""

import py_trees
from arlab_common_interfaces.action import ManipulationAction
from arlab_common_interfaces.msg import ManipulationCommand, ManipulationResponse
from py_trees.behaviour import Behaviour
from py_trees.common import Access, Status
from py_trees.composites import Sequence

from .generic_manipulation import GenericManipulation


def get_tree(x: float, y: float, z: float, ox: float, oy: float, oz: float, ow) -> Behaviour:
    """Create the behaviour tree subtree for a move manipulation action.

    The subtree prepares a move command goal from fixed position and
    orientation values, executes the generic manipulation action, and checks
    whether the returned result indicates success.

    Args:
        x (float): Target x position.
        y (float): Target y position.
        z (float): Target z position.
        ox (float): Target orientation x component.
        oy (float): Target orientation y component.
        oz (float): Target orientation z component.
        ow: Target orientation w component.

    Returns:
        Behaviour: Root behaviour of the configured move manipulation
            subtree.
    """
    return Sequence(
        name="ManipulationMove",
        memory=True,
        children=[
            SetupManipulationMove(x=x, y=y, z=z, ox=ox, oy=oy, oz=oz, ow=ow),
            GenericManipulation("Move", "/manipulation/move_goal", "/manipulation/move_result"),
            CheckManipulationMove(),
        ],
    )


def get_tree_blackboard(pose_input_key: str) -> Behaviour:
    """Create the behaviour tree subtree for a move action using blackboard input.

    The subtree reads the target pose from the blackboard, executes the
    generic manipulation action, and checks whether the returned result
    indicates success.

    Args:
        pose_input_key (str): Blackboard key from which the target pose is
            read.

    Returns:
        Behaviour: Root behaviour of the configured move manipulation
            subtree.
    """
    return Sequence(
        name="ManipulationMove",
        memory=True,
        children=[
            SetupManipulationMoveBlackboard(pose_input_key=pose_input_key),
            GenericManipulation("Move", "/manipulation/move_goal", "/manipulation/move_result"),
            CheckManipulationMove(),
        ],
    )


class SetupManipulationMove(Behaviour):
    """Behaviour that writes a fixed move manipulation goal to the blackboard.

    This behaviour prepares the goal message for the move manipulation
    action from fixed pose values and stores it under the blackboard key
    `/manipulation/move_goal`.
    """
    def __init__(self, x: float, y: float, z: float, ox: float, oy: float, oz: float, ow):
        """Initialise the fixed move goal setup behaviour.

        Args:
            x (float): Target x position.
            y (float): Target y position.
            z (float): Target z position.
            ox (float): Target orientation x component.
            oy (float): Target orientation y component.
            oz (float): Target orientation z component.
            ow: Target orientation w component.
        """
        super().__init__(name=type(self).__name__)
        self.x = x
        self.y = y
        self.z = z
        self.ox = ox
        self.oy = oy
        self.oz = oz
        self.ow = ow
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="/manipulation/move_goal", access=Access.WRITE)

    def update(self):
        """Create and store the configured fixed move manipulation goal.

        Returns:
            Status: `Status.SUCCESS` after the goal has been written to the
                blackboard.
        """
        goal = ManipulationAction.Goal()
        goal.command.command_type = ManipulationCommand.COMMAND_MOVE
        goal.command.target_pose.position.x = self.x
        goal.command.target_pose.position.y = self.y
        goal.command.target_pose.position.z = self.z
        quat = goal.command.target_pose.orientation
        (quat.x, quat.y, quat.z, quat.w) = (self.ox, self.oy, self.oz, self.ow)
        self.blackboard.manipulation.move_goal = goal
        return Status.SUCCESS


class SetupManipulationMoveBlackboard(Behaviour):
    """Behaviour that writes a blackboard-based move goal to the blackboard.

    This behaviour reads a target pose from a configured blackboard key and
    stores the corresponding move action goal under
    `/manipulation/move_goal`.
    """
    def __init__(self, pose_input_key: str):
        """Initialise the blackboard-based move goal setup behaviour.

        Args:
            pose_input_key (str): Blackboard key from which the target pose is
                read.
        """
        super().__init__(name=type(self).__name__)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="/manipulation/move_goal", access=Access.WRITE)
        self.blackboard.register_key(
            key="pose_input",
            access=Access.READ,
            # make sure to namespace it if not already
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", pose_input_key),
        )

    def update(self):
        """Create and store the configured blackboard-based move goal.

        Returns:
            Status: `Status.SUCCESS` after the goal has been written to the
                blackboard.
        """
        goal = ManipulationAction.Goal()
        goal.command.command_type = ManipulationCommand.COMMAND_MOVE
        goal.command.target_pose = self.blackboard.pose_input
        self.blackboard.manipulation.move_goal = goal
        return Status.SUCCESS


class CheckManipulationMove(Behaviour):
    """Behaviour that validates the result of the move manipulation action.

    This behaviour reads the manipulation result from the blackboard and
    returns success only if the action completed successfully.
    """
    def __init__(self):
        """Initialise the move manipulation result checking behaviour."""
        super().__init__(name=type(self).__name__)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="/manipulation/move_result", access=Access.READ)

    def update(self):
        """Check the move manipulation result stored on the blackboard.

        Returns:
            Status: `Status.SUCCESS` if the move manipulation action
                succeeded, otherwise `Status.FAILURE`.
        """
        result: ManipulationAction.Result = self.blackboard.manipulation.move_result
        print(f"Move message: {result.response.message}")
        if result.response.error_code != ManipulationResponse.SUCCESS:
            return Status.FAILURE
        return Status.SUCCESS
