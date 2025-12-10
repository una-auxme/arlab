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
        name="ManipulationMove",
        memory=True,
        children=[
            SetupManipulationMove(),
            GenericManipulation(
                "Move", "/manipulation/move_goal", "/manipulation/move_result"
            ),
            CheckManipulationMove(),
        ],
    )


class SetupManipulationMove(Behaviour):
    def __init__(self):
        super().__init__(name=type(self).__name__)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="/manipulation/move_goal", access=Access.WRITE)

    def update(self):
        goal = ManipulationAction.Goal()
        goal.command.command_type = ManipulationCommand.COMMAND_MOVE
        goal.command.target_pose.position.y = 0.3
        goal.command.target_pose.position.x = 0.0
        goal.command.target_pose.position.z = 0.3
        quat = goal.command.target_pose.orientation
        (quat.w, quat.x, quat.y, quat.z) = euler2quat(0.0, pi, 0.0)
        self.blackboard.manipulation.move_goal = goal
        return Status.SUCCESS


class CheckManipulationMove(Behaviour):
    def __init__(self):
        super().__init__(name=type(self).__name__)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="/manipulation/move_result", access=Access.READ
        )

    def update(self):
        result: ManipulationAction.Result = self.blackboard.manipulation.move_result
        print(result.response.message)
        if result.response.message != "SUCCESS":
            return Status.FAILURE
        return Status.SUCCESS
