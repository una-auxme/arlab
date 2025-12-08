from py_trees.behaviour import Behaviour
from py_trees.composites import Sequence

from ..manipulation import home, move


def get_tree() -> Behaviour:
    return Sequence(
        "MoveHomeSeq", memory=True, children=[home.get_tree(), move.get_tree()]
    )
