from mia_hand_msgs.action import Grasp
from py_trees.behaviour import Behaviour
from py_trees.common import Access, ParallelPolicy, Status
from py_trees.composites import Parallel, Sequence
from py_trees.timers import Timer

from ..common.action import RosActionBehaviour


def get_tree(target_closure_percent: int) -> Behaviour:
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
    def __init__(self, target_closure_percent: int):
        super().__init__(name=type(self).__name__)
        self.target_closure_percent = target_closure_percent
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="/manipulation/hand/grasp_goal", access=Access.WRITE)

    def update(self):
        goal = Grasp.Goal()
        goal.spe_for_percent = 15
        goal.target_closure_percent = self.target_closure_percent
        self.blackboard.manipulation.hand.grasp_goal = goal
        return Status.SUCCESS


class CheckHandGrasp(Behaviour):
    def __init__(self):
        super().__init__(name=type(self).__name__)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="/manipulation/hand/grasp_result", access=Access.READ)

    def update(self):
        result: Grasp.Result = self.blackboard.manipulation.hand.grasp_result
        if result.err_message:
            print(result.err_message)
            return Status.FAILURE
        return Status.SUCCESS
