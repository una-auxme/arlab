from py_trees.behaviour import Behaviour
from py_trees.composites import Selector, Sequence

from . import (
    cupboard_found,
    search_for_furniture,
    store_grocery,
    table_found,
    task_done,
    wait_for_task_start,
)


def get_tree() -> Behaviour:
    return Sequence(
        name="Task2",
        memory=False,
        children=[
            wait_for_task_start.get_tree(),
            Selector(
                name="MainTask2Selector",
                memory=False,
                children=[
                    task_done.get_tree(),
                    Sequence(
                        name="StoreGrocerySequence",
                        memory=False,
                        children=[
                            cupboard_found.get_tree(),
                            table_found.get_tree(),
                            store_grocery.get_tree(),
                        ],
                    ),
                    search_for_furniture.get_tree(),
                ],
            ),
        ],
    )
