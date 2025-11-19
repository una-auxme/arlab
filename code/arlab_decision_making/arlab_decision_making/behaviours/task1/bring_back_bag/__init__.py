from py_trees.behaviour import Behaviour
from py_trees.composites import Sequence

from . import check_for_bag_in_gripper, move_to_starting_position


def get_tree() -> Behaviour:
    return Sequence(
        name="BringBackBag",
        memory=False,
        children=[
            check_for_bag_in_gripper.get_tree(),
            move_to_starting_position.get_tree(),
        ],
    )
