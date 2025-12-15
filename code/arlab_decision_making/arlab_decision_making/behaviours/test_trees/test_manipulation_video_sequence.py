from py_trees.behaviour import Behaviour
from py_trees.composites import Sequence
from py_trees.timers import Timer

from ..manipulation import home, move_to_picture_pose_grab, move_to_pose_grab, move_to_pose_cupboard_picture, move_to_in_cupboard


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
            move_to_pose_grab.get_tree(),
            Timer("Waiting to grab object", 5.0),
            move_to_picture_pose_grab.get_tree(),
            Timer("Wait after grab for picture", 5.0),
            move_to_pose_cupboard_picture.get_tree(),
            Timer("Wait for cupbaord picture"),
            move_to_in_cupboard.get_tree(),
            Timer("Waiting in cupboard", 5.0),
            move_to_pose_cupboard_picture.get_tree(),
            Timer("Wait for cupbaord picture"),
        ],
    )
