from py_trees.behaviour import Behaviour
from py_trees.common import Sequence

from . import (
    check_for_items,
    check_for_target_placement,
    move_to_cupboard,
    move_to_table,
    pickItem,
    placeItem,
)


def get_tree() -> Behaviour:
    return Sequence(
        name="Task2",
        memory=True,
        children=[
            check_for_items.get_tree(),
            move_to_cupboard.get_tree(),
            pickItem.get_tree(),
            move_to_table.get_tree(),
            check_for_target_placement.get_tree(),
            placeItem.get_tree(),
        ],
    )
