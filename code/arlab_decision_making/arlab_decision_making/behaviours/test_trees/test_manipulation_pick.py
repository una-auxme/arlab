from py_trees.behaviour import Behaviour
from py_trees.composites import Sequence
from py_trees.timers import Timer

from ..manipulation import (
    direct_hand_grasp,
    home,
    manipulator_pick,
    move,
)


def get_tree() -> Behaviour:
    return Sequence(
        "GrabSequence",
        memory=True,
        children=[
            Timer("InitialWait", 1.0),
            direct_hand_grasp.get_tree(70),
            home.get_tree(),
            Timer("MoveWait", 0.5),
            move.get_tree(
                x=-0.2922,
                y=0.099,
                z=0.25,
                ox=0.6952,
                oy=-0.5497,
                oz=-0.29057,
                ow=0.3606,
            ),
            move.get_tree(
                x=-0.2922,
                y=0.099,
                z=0.10,
                ox=0.6952,
                oy=-0.5497,
                oz=-0.29057,
                ow=0.3606,
            ),
            Timer("Wait after picture", 0.5),
            manipulator_pick.get_tree(id=1),
            move.get_tree(
                x=-0.2922,
                y=0.099,
                z=0.1465,
                ox=0.6952,
                oy=-0.5497,
                oz=-0.29057,
                ow=0.3606,
            ),
            Timer("Wait after grab for picture", 2.5),
            move.get_tree(
                x=-0.0969,
                y=0.4099,
                z=0.07868,
                ox=-0.28162,
                oy=0.714643,
                oz=0.600243,
                ow=-0.222892,
            ),
            Timer("Wait for cupboard picture"),
            move.get_tree(
                x=-0.2936,
                y=0.6563,
                z=0.1065,
                ox=-0.262,
                oy=0.6416,
                oz=0.6843,
                ow=-0.2242,
            ),
            move.get_tree(
                x=-0.2936,
                y=0.6563,
                z=0.1,
                ox=-0.262,
                oy=0.6416,
                oz=0.6843,
                ow=-0.2242,
            ),
            Timer("Waiting in cupboard", 2.5),
            direct_hand_grasp.get_tree(0),
            Timer("Wait after hand open", 2.5),
            move.get_tree(
                x=-0.2936,
                y=0.6563,
                z=0.1565,
                ox=-0.262,
                oy=0.6416,
                oz=0.6843,
                ow=-0.2242,
            ),
            move.get_tree(
                x=-0.0969,
                y=0.4099,
                z=0.12868,
                ox=-0.28162,
                oy=0.714643,
                oz=0.600243,
                ow=-0.222892,
            ),
            Timer("Wait for cupbaord picture"),
        ],
    )
