from py_trees.behaviour import Behaviour

from ..manipulation import move


def get_tree() -> Behaviour:
    return move.get_tree(
        x=-0.2922,
        y=0.099,
        z=0.15,
        ox=0.6952,
        oy=-0.5497,
        oz=-0.29057,
        ow=0.3606,
    )
