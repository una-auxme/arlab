from math import pi

from arlab_common_interfaces.action import ManipulationAction
from arlab_common_interfaces.msg import ManipulationCommand, ManipulationResponse
from py_trees.behaviour import Behaviour
from py_trees.common import Access, Status
from py_trees.composites import Sequence

from .generic_manipulation import GenericManipulation


def get_tree() -> Behaviour:
    return Sequence(
        name="ManipulationClose",
        memory=True,
        children=[
            SetHandClose(),
            GenericManipulation(
                "Close", "/manipulation/close_goal", "/manipulation/close_result"
            ),
            CheckHandClose(),
        ],
    )


class SetHandClose(Behaviour):
    def __init__(self):
        super().__init__(name=type(self).__name__)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="/manipulation/close_goal", access=Access.WRITE
        )

    def update(self):
        goal = ManipulationAction.Goal()
        goal.command.command_type = ManipulationCommand.COMMAND_CLOSE
        self.blackboard.manipulation.close_goal = goal
        return Status.SUCCESS


class CheckHandClose(Behaviour):
    def __init__(self):
        super().__init__(name=type(self).__name__)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="/manipulation/close_result", access=Access.READ
        )

    def update(self):
        result: ManipulationAction.Result = self.blackboard.manipulation.close_result
        print(f"close message: {result.response.message}")
        if result.response.error_code != ManipulationResponse.SUCCESS:
            return Status.FAILURE
        return Status.SUCCESS
