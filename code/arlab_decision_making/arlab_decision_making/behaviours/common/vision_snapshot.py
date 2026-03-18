"""Vision snapshot behaviours for py_trees with ROS2 action integration.

Provides a behaviour tree subtree to:
    - configure and send a vision snapshot action goal
    - request an optional database clear before taking the snapshot
    - optionally enable hand masking for the snapshot
    - validate the returned snapshot result

Maintainers:
    Peter Viechter <peter.viechter@uni-augsburg.de>
    Daniel Gabler <daniel.gabler@uni-augsburg.de>
"""

from arlab_common_interfaces.action import VisionSnapshotAction
from arlab_common_interfaces.msg import VisionSnapshotResponse
from py_trees.behaviour import Behaviour
from py_trees.common import Access, Status
from py_trees.composites import Sequence

from .action import RosActionBehaviour


def get_tree(clear: bool, mask_hand: bool = False) -> Behaviour:
    """Create the behaviour tree subtree for a vision snapshot request.

    The subtree prepares the action goal, executes the vision snapshot action,
    and checks whether the returned result indicates success.

    Args:
        clear (bool): Whether the vision database should be cleared before
            taking the snapshot.
        mask_hand (bool, optional): Whether the robot hand should be masked in
            the snapshot. Defaults to False.

    Returns:
        Behaviour: Root behaviour of the configured vision snapshot subtree.
    """
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
    """Behaviour that writes a vision snapshot action goal to the blackboard.

    This behaviour prepares the goal message for the vision snapshot action
    and stores it under the blackboard key `/vision/snapshot_goal`.
    """
    def __init__(self, clear: bool, mask_hand: bool = False):
        """Initialise the snapshot goal setup behaviour.

        Args:
            clear (bool): Whether the snapshot request should clear the
                database before execution.
            mask_hand (bool, optional): Whether hand masking should be enabled
                for the snapshot request. Defaults to False.
        """
        super().__init__(name=type(self).__name__)
        self.clear = clear
        self.mask_hand = mask_hand
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="/vision/snapshot_goal", access=Access.WRITE)

    def update(self):
        """Create and store the configured vision snapshot goal.

        Returns:
            Status: `Status.SUCCESS` after the goal has been written to the
                blackboard.
        """
        goal = VisionSnapshotAction.Goal()
        goal.command.clear_database = self.clear
        goal.command.mask_hand = self.mask_hand
        self.blackboard.vision.snapshot_goal = goal
        return Status.SUCCESS


class CheckVisionSnapshot(Behaviour):
    """Behaviour that validates the result of a vision snapshot action.

    This behaviour reads the action result from the blackboard and returns
    success only if the snapshot action completed successfully.
    """
    def __init__(self):
        """Initialise the snapshot result checking behaviour."""
        super().__init__(name=type(self).__name__)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="/vision/snapshot_result", access=Access.READ)

    def update(self):
        """Check the snapshot action result stored on the blackboard.

        Returns:
            Status: `Status.SUCCESS` if the snapshot action succeeded,
                otherwise `Status.FAILURE`.
        """
        result: VisionSnapshotAction.Result = self.blackboard.vision.snapshot_result
        if result.response.result != VisionSnapshotResponse.SUCCESS:
            print(f"Snapshot error: {result.response.error_msg}")
            return Status.FAILURE
        return Status.SUCCESS
