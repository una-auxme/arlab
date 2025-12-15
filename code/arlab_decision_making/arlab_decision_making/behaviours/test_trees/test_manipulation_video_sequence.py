from py_trees.behaviour import Behaviour
from py_trees.composites import Sequence
from py_trees.timers import Timer

from ..manipulation import home, move_to_picture_pose_grab, move_to_pose_grab


def get_tree() -> Behaviour:
    return Sequence(
        "GrabSequence",
        memory=True,
        children=[
            Timer("InitialWait", 5.0),
            home.get_tree(),
            Timer("MoveWait", 5.0),
            move_to_picture_pose_grab.get_tree(),
            Timer("Wait after picture", 5.0),
            # move_to_pose_grab.get_tree(),
            # Timer("Waiting to grab object", 5.0),
        ],
    )
