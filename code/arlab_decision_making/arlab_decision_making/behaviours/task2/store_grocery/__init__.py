from py_trees.behaviour import Behaviour
from py_trees.composites import Sequence

from . import (
    check_for_items,
    check_for_target_placement,
    move_to_cupboard,
    move_to_table,
    pick_item,
    place_item,
)


def get_tree() -> Behaviour:
    return Sequence(
        name="StoreGrocery",
        memory=True,
        children=[
            move_to_table.get_tree(),
            check_for_items.get_tree(),
            pick_item.get_tree(),
            move_to_cupboard.get_tree(),
            check_for_target_placement.get_tree(),
            place_item.get_tree(),
        ],
    )
