from arlab_common_interfaces.action import ManipulationAction
from arlab_common_interfaces.msg import ManipulationCommand
from py_trees.behaviour import Behaviour
from py_trees.common import Access, Status
from py_trees.composites import Sequence

from .generic_manipulation import GenericManipulation


def get_tree() -> Behaviour:
    return Sequence(
        name="ManipulationHome",
        memory=True,
        children=[
            SetupManipulationHome(),
            GenericManipulation(
                "Home", "/manipulation/home_goal", "/manipulation/home_result"
            ),
            CheckManipulationHome(),
        ],
    )


class SetupManipulationHome(Behaviour):
    def __init__(self):
        super().__init__(name=type(self).__name__)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="/manipulation/home_goal", access=Access.WRITE)

    def update(self):
        goal = ManipulationAction.Goal()
        goal.command.command_type = ManipulationCommand.COMMAND_HOME
        goal.command.target_entityid = 1
        self.blackboard.manipulation.home_goal = goal
        return Status.SUCCESS


class CheckManipulationHome(Behaviour):
    def __init__(self):
        super().__init__(name=type(self).__name__)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="/manipulation/home_result", access=Access.READ
        )

    def update(self):
        result: ManipulationAction.Result = self.blackboard.manipulation.home_result
        # print(result.response.message)
        # TODO: Check message and arm position
        return Status.RUNNING
