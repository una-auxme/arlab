"""Direct hand grasp demo behaviour tree for py_trees.

Provides a predefined behaviour tree sequence to:
    - wait before starting execution
    - home the manipulator and execute direct hand grasp commands
    - move through predefined pick, transport, and placement poses
    - open and close the hand at specific stages of the demo
    - perform a scripted grasp-and-place demonstration

Maintainers:
    Peter Viechter <peter.viechter@uni-a.de>
    Daniel Gabler <daniel.gabler@uni-a.de>
"""

from py_trees.behaviour import Behaviour
from py_trees.composites import Sequence
from py_trees.timers import Timer

from ..manipulation import (
    direct_hand_grasp,
    home,
    move,
)


def get_tree() -> Behaviour:
    """Create the scripted direct hand grasp demo behaviour tree.

    This tree executes a fixed demonstration sequence including homing,
    moving to predefined poses, grasping and releasing objects with the
    direct hand grasp behaviour, and performing a scripted place action.

    Returns:
        Behaviour: Root behaviour of the configured direct hand grasp demo
            sequence.
    """
    return Sequence(
        "GrabSequence",
        memory=True,
        children=[
            Timer("InitialWait", 5.0),
            direct_hand_grasp.get_tree(70),
            home.get_tree(),
            Timer("MoveWait", 2.5),
            move.get_tree(
                x=-0.2922,
                y=0.099,
                z=0.3,
                ox=0.6952,
                oy=-0.5497,
                oz=-0.29057,
                ow=0.3606,
            ),
            move.get_tree(
                x=-0.2922,
                y=0.099,
                z=0.15,
                ox=0.6952,
                oy=-0.5497,
                oz=-0.29057,
                ow=0.3606,
            ),
            Timer("Wait after picture", 2.5),
            move.get_tree(
                x=-0.2922,
                y=0.099,
                z=0.3,
                ox=0.6952,
                oy=-0.5497,
                oz=-0.29057,
                ow=0.3606,
            ),
            direct_hand_grasp.get_tree(0),
            Timer("Wait after hand open", 2.5),
            move.get_tree(
                x=-0.4322,
                y=0.0608,
                z=0.3,
                ox=0.5616,
                oy=-0.41149,
                oz=-0.46496,
                ow=0.546879,
            ),
            move.get_tree(
                x=-0.4322,
                y=0.0608,
                z=0.08,
                ox=0.5616,
                oy=-0.41149,
                oz=-0.46496,
                ow=0.546879,
            ),
            Timer("Waiting to grab object", 2.5),
            direct_hand_grasp.get_tree(80),
            Timer("Wait after hand close", 2.5),
            move.get_tree(
                x=-0.2922,
                y=0.099,
                z=0.1965,
                ox=0.6952,
                oy=-0.5497,
                oz=-0.29057,
                ow=0.3606,
            ),
            Timer("Wait after grab for picture", 2.5),
            move.get_tree(
                x=-0.0969,
                y=0.4099,
                z=0.12868,
                ox=-0.28162,
                oy=0.714643,
                oz=0.600243,
                ow=-0.222892,
            ),
            Timer("Wait for cupboard picture"),
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
