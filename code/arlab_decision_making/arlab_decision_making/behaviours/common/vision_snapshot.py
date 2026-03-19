from arlab_common_interfaces.action import VisionSnapshotAction
from arlab_common_interfaces.msg import VisionSnapshotResponse
from py_trees.behaviour import Behaviour
from py_trees.common import Access, Status
from py_trees.composites import Sequence

from .action import RosActionBehaviour


def get_tree(clear: bool, mask_hand: bool = False) -> Behaviour:
    return Sequence(
        name="VisionSnapshot",
        memory=True,
        children=[
            SetupVisionSnapshot(clear=clear, mask_hand=mask_hand),
            RosActionBehaviour(
                name="VisionSnapshotAction",
                action_type=VisionSnapshotAction,
                action_name="/vision/snapshot",
                key="/vision/snapshot_goal",
                result_output_key="/vision/snapshot_result",
            ),
            CheckVisionSnapshot(),
        ],
    )


class SetupVisionSnapshot(Behaviour):
    def __init__(self, clear: bool, mask_hand: bool = False):
        super().__init__(name=type(self).__name__)
        self.clear = clear
        self.mask_hand = mask_hand
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="/vision/snapshot_goal", access=Access.WRITE)

    def update(self):
        goal = VisionSnapshotAction.Goal()
        goal.command.clear_database = self.clear
        goal.command.mask_hand = self.mask_hand
        self.blackboard.vision.snapshot_goal = goal
        return Status.SUCCESS


class CheckVisionSnapshot(Behaviour):
    def __init__(self):
        super().__init__(name=type(self).__name__)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="/vision/snapshot_result", access=Access.READ)

    def update(self):
        result: VisionSnapshotAction.Result = self.blackboard.vision.snapshot_result
        if result.response.result != VisionSnapshotResponse.SUCCESS:
            print(f"Snapshot error: {result.response.error_msg}")
            return Status.FAILURE
        return Status.SUCCESS
