from math import pi

from arlab_common_interfaces.action import ManipulationAction
from arlab_common_interfaces.msg import ManipulationCommand, ManipulationResponse
from py_trees.behaviour import Behaviour
from py_trees.common import Access, Status
from py_trees.composites import Sequence

from .generic_manipulation import GenericManipulation


def get_tree(id: int) -> Behaviour:
    return Sequence(
        name="ManipulationPick",
        memory=True,
        children=[
            SetHandPick(id=id),
            GenericManipulation(
                "Pick", "/manipulation/pick_goal", "/manipulation/pick_result"
            ),
            CheckHandPick(),
        ],
    )


class SetHandPick(Behaviour):
    def __init__(self, id: int):
        super().__init__(name=type(self).__name__)
        self.id = id
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="/manipulation/pick_goal", access=Access.WRITE)

    def update(self):
        goal = ManipulationAction.Goal()
        goal.command.command_type = ManipulationCommand.COMMAND_PICK
        goal.command.target_entityid = self.id
        self.blackboard.manipulation.pick_goal = goal
        return Status.SUCCESS


class CheckHandPick(Behaviour):
    def __init__(self):
        super().__init__(name=type(self).__name__)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="/manipulation/pick_result", access=Access.READ
        )

    def update(self):
        result: ManipulationAction.Result = self.blackboard.manipulation.pick_result
        print(f"pick message: {result.response.message}")
        if result.response.error_code != ManipulationResponse.SUCCESS:
            return Status.FAILURE
        return Status.SUCCESS
