from py_trees.behaviour import Behaviour

from ..manipulation import move_to_picture_pose_grab


def get_tree() -> Behaviour:
    return move_to_picture_pose_grab.get_tree()
