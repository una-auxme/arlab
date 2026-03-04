from arlab_common_interfaces.action import ManipulationAction
from arlab_common_interfaces.msg import ManipulationCommand, ManipulationResponse
from py_trees.behaviour import Behaviour
from py_trees.common import Access, Status
from py_trees.composites import Sequence

from .generic_manipulation import GenericManipulation


def get_tree(
    x: float, y: float, z: float, ox: float, oy: float, oz: float, ow
) -> Behaviour:
    return Sequence(
        name="ManipulationMove",
        memory=True,
        children=[
            SetupManipulationMove(x=x, y=y, z=z, ox=ox, oy=oy, oz=oz, ow=ow),
            GenericManipulation(
                "Move", "/manipulation/move_goal", "/manipulation/move_result"
            ),
            CheckManipulationMove(),
        ],
    )


class SetupManipulationMove(Behaviour):
    def __init__(
        self, x: float, y: float, z: float, ox: float, oy: float, oz: float, ow
    ):
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
        goal = ManipulationAction.Goal()
        goal.command.command_type = ManipulationCommand.COMMAND_MOVE
        goal.command.target_pose.position.x = self.x
        goal.command.target_pose.position.y = self.y
        goal.command.target_pose.position.z = self.z
        quat = goal.command.target_pose.orientation
        (quat.x, quat.y, quat.z, quat.w) = (self.ox, self.oy, self.oz, self.ow)
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
        print(f"Move message: {result.response.message}")
        if result.response.error_code != ManipulationResponse.SUCCESS:
            return Status.FAILURE
        return Status.SUCCESS
