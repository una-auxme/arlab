from typing import Optional

import py_trees
from arlab_common_interfaces.action import ManipulationAction
from arlab_common_interfaces.msg import ManipulationCommand, ManipulationResponse
from py_trees.behaviour import Behaviour
from py_trees.common import Access, Status
from py_trees.composites import Sequence

from .generic_manipulation import GenericManipulation


def get_tree(id_input_key: Optional[str] = None, fixed_id: Optional[int] = None) -> Behaviour:
    return Sequence(
        name="ManipulationPick",
        memory=True,
        children=[
            SetHandPick(id_input_key=id_input_key, fixed_id=fixed_id),
            GenericManipulation("Pick", "/manipulation/pick_goal", "/manipulation/pick_result"),
            CheckHandPick(),
        ],
    )


class SetHandPick(Behaviour):
    def __init__(self, id_input_key: Optional[str] = None, fixed_id: Optional[int] = None):
        super().__init__(name=type(self).__name__)
        self.fixed_id = fixed_id
        self.id_input_key = id_input_key

        self.blackboard = self.attach_blackboard_client(name=self.name)
        if self.id_input_key is not None:
            self.blackboard.register_key(
                key="id_input",
                access=Access.READ,
                # make sure to namespace it if not already
                remap_to=py_trees.blackboard.Blackboard.absolute_name("/", self.id_input_key),
            )
        self.blackboard.register_key(key="/manipulation/pick_goal", access=Access.WRITE)

    def update(self):
        goal = ManipulationAction.Goal()
        goal.command.command_type = ManipulationCommand.COMMAND_PICK
        if self.id_input_key is not None:
            goal.command.target_entityid = self.blackboard.id_input
        elif self.fixed_id is not None:
            goal.command.target_entityid = self.fixed_id
        else:
            return Status.FAILURE
        self.blackboard.manipulation.pick_goal = goal
        return Status.SUCCESS


class CheckHandPick(Behaviour):
    def __init__(self):
        super().__init__(name=type(self).__name__)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="/manipulation/pick_result", access=Access.READ)

    def update(self):
        result: ManipulationAction.Result = self.blackboard.manipulation.pick_result
        print(f"pick message: {result.response.message}")
        if result.response.error_code != ManipulationResponse.SUCCESS:
            return Status.FAILURE
        return Status.SUCCESS
