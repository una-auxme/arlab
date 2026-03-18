"""Direct hand grasp behaviours for py_trees with ROS2 action integration.

Provides a behaviour tree subtree to:
    - configure a direct hand grasp goal on the py_trees blackboard
    - execute the grasp action for the MIA hand
    - limit grasp execution time with a timeout
    - validate the returned grasp result

Maintainers:
    Peter Viechter <peter.viechter@uni-augsburg.de>
    Daniel Gabler <daniel.gabler@uni-augsburg.de>
"""

from mia_hand_msgs.action import Grasp
from py_trees.behaviour import Behaviour
from py_trees.common import Access, ParallelPolicy, Status
from py_trees.composites import Parallel, Sequence
from py_trees.timers import Timer

from ..common.action import RosActionBehaviour


def get_tree(target_closure_percent: int) -> Behaviour:
    """Create the behaviour tree subtree for a direct hand grasp action.

    The subtree prepares the grasp goal, executes the grasp action in
    parallel with a timeout timer, and checks whether the grasp result
    indicates success.

    Args:
        target_closure_percent (int): Target hand closure in percent for the
            grasp action.

    Returns:
        Behaviour: Root behaviour of the configured direct hand grasp subtree.
    """
    return Sequence(
        name="DirectHandGrasp",
        memory=True,
        children=[
            SetupHandGrasp(target_closure_percent),
            Parallel(
                "GraspTimeout",
                policy=ParallelPolicy.SuccessOnOne(),
                children=[
                    Timer("GraspTimeout", 3.0),
                    RosActionBehaviour(
                        name="GraspAction",
                        action_type=Grasp,
                        action_name="/mia_hand/grasps/cylindrical/action",
                        key="/manipulation/hand/grasp_goal",
                        result_output_key="/manipulation/hand/grasp_result",
                    ),
                ],
            ),
            CheckHandGrasp(),
        ],
    )


class SetupHandGrasp(Behaviour):
    """Behaviour that writes a hand grasp action goal to the blackboard.

    This behaviour prepares the goal message for the direct grasp action
    and stores it under the blackboard key
    `/manipulation/hand/grasp_goal`.
    """
    def __init__(self, target_closure_percent: int):
        """Initialise the hand grasp goal setup behaviour.

        Args:
            target_closure_percent (int): Target hand closure in percent for
                the grasp request.
        """
        super().__init__(name=type(self).__name__)
        self.target_closure_percent = target_closure_percent
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="/manipulation/hand/grasp_goal", access=Access.WRITE)

    def update(self):
        """Create and store the configured hand grasp goal.

        Returns:
            Status: `Status.SUCCESS` after the goal has been written to the
                blackboard.
        """
        goal = Grasp.Goal()
        goal.spe_for_percent = 15
        goal.target_closure_percent = self.target_closure_percent
        self.blackboard.manipulation.hand.grasp_goal = goal
        return Status.SUCCESS


class CheckHandGrasp(Behaviour):
    """Behaviour that validates the result of the hand grasp action.

    This behaviour reads the grasp result from the blackboard and returns
    success only if the action completed without an error message.
    """
    def __init__(self):
        """Initialise the hand grasp result checking behaviour."""
        super().__init__(name=type(self).__name__)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="/manipulation/hand/grasp_result", access=Access.READ)

    def update(self):
        """Check the hand grasp result stored on the blackboard.

        Returns:
            Status: `Status.SUCCESS` if the grasp action completed without
                errors, otherwise `Status.FAILURE`.
        """
        result: Grasp.Result = self.blackboard.manipulation.hand.grasp_result
        if result.err_message:
            print(result.err_message)
            return Status.FAILURE
        return Status.SUCCESS
