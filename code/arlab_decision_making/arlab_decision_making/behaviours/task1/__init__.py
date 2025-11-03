from py_trees.behaviour import Behaviour
from py_trees.composites import Selector, Sequence

from . import (
    bring_back_bag,
    done,
    follow_operator,
    get_bag,
    operator_found,
    search_for_operator,
    wait_for_task_start,
)


def get_tree() -> Behaviour:
    return Sequence(
        name="Task1",
        memory=False,
        children=[
            wait_for_task_start.get_tree(),
            Selector(
                name="Task1Main",
                memory=False,
                children=[
                    done.get_tree(),
                    bring_back_bag.get_tree(),
                    get_bag.get_tree(),
                    Sequence(
                        name="FollowOperatorSeq",
                        memory=False,
                        children=[
                            operator_found.get_tree(),
                            follow_operator.get_tree(),
                        ],
                    ),
                    search_for_operator.get_tree(),
                ],
            ),
        ],
    )
