from math import pi

from arlab_common_interfaces.action import ManipulationAction
from arlab_common_interfaces.msg import ManipulationCommand
from py_trees.behaviour import Behaviour
from py_trees.common import Access, Status
from py_trees.composites import Sequence
from transforms3d.euler import euler2quat

from .generic_manipulation import GenericManipulation


def get_tree() -> Behaviour:
    return Sequence(
        name="move_to_picture_pose_grab",
        memory=True,
        children=[
            SetupManipulationMove(),
            GenericManipulation(
                "move_to_picture_pose_grab",
                "/manipulation/move_to_picture_pose_grab_goal",
                "/manipulation/move_to_picture_pose_grab_result",
            ),
            CheckManipulationMove(),
        ],
    )


class SetupManipulationMove(Behaviour):
    def __init__(self):
        super().__init__(name=type(self).__name__)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="/manipulation/move_to_picture_pose_grab_goal", access=Access.WRITE
        )

    def update(self):
        goal = ManipulationAction.Goal()
        goal.command.command_type = ManipulationCommand.COMMAND_MOVE
        goal.command.target_pose.position.x = -0.2922
        goal.command.target_pose.position.y = 0.099
        goal.command.target_pose.position.z = 0.1965
        # roll = -1.571
        # pitch = -2.029
        # yaw = 0.892
        # quat = goal.command.target_pose.orientation
        # (quat.w, quat.x, quat.y, quat.z) = euler2quat(roll, pitch, yaw)
        # quat = goal.command.target_pose.orientation
        # (quat.w, quat.x, quat.y, quat.z) = euler2quat(0.0, pi, 0.0)
        goal.command.target_pose.orientation._x = 0.6952
        goal.command.target_pose.orientation._y = -0.5497
        goal.command.target_pose.orientation._z = -0.29057
        goal.command.target_pose.orientation._w = 0.3606
        self.blackboard.manipulation.move_to_picture_pose_grab_goal = goal
        return Status.SUCCESS


class CheckManipulationMove(Behaviour):
    def __init__(self):
        super().__init__(name=type(self).__name__)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="/manipulation/move_to_picture_pose_grab_result", access=Access.READ
        )

    def update(self):
        result: ManipulationAction.Result = (
            self.blackboard.manipulation.move_to_picture_pose_grab_result
        )
        print(result.response.message)
        if result.response.message != "SUCCESS":
            return Status.FAILURE
        return Status.SUCCESS
