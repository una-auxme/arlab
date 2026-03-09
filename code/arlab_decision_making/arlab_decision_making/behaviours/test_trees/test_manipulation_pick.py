from py_trees.behaviour import Behaviour
from py_trees.composites import Sequence
from py_trees.timers import Timer

from ..manipulation import hand_close, home, manipulator_pick, move


def get_tree() -> Behaviour:
    return Sequence(
        "GrabSequence",
        memory=True,
        children=[
            Timer("InitialWait", 1.0),
            hand_close.get_tree(),
            home.get_tree(),
            Timer("MoveWait", 0.5),
            move.get_tree(
                x=-0.407395,
                y=-0.00167976,
                z=0.606272,
                ox=0.708559,
                oy=-0.699248,
                oz=-0.0733024,
                ow=0.0601985,
            ),
            Timer("Wait after picture", 0.5),
            manipulator_pick.get_tree(id=35),
        ],
    )
