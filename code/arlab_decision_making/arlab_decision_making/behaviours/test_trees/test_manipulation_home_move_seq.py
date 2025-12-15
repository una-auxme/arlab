from py_trees.behaviour import Behaviour
from py_trees.composites import Sequence
from py_trees.timers import Timer

from ..manipulation import home, move


def get_tree() -> Behaviour:
    return Sequence(
        "MoveHomeSeq",
        memory=True,
        children=[
            Timer("InitialWait", 5.0),
            home.get_tree(),
            Timer("MoveWait", 5.0),
            move.get_tree(),
        ],
    )
