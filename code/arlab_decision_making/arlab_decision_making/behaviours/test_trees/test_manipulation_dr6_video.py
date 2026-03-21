"""Scripted grab-and-place demo behaviour tree for py_trees.

Provides a predefined behaviour tree sequence to:
    - initialise the manipulator and speech output
    - take scene snapshots before manipulation
    - execute scripted pick and place motions for multiple fruits
    - move objects to predefined cupboard positions
    - return the manipulator to the home position between tasks

Maintainers:
    Peter Viechter <peter.viechter@uni-a.de>
    Daniel Gabler <daniel.gabler@uni-a.de>
"""

from py_trees.behaviour import Behaviour
from py_trees.composites import Sequence
from py_trees.timers import Timer

# from ..common.speech import QueueSpeech
from ..manipulation import (
    # hand_close,
    # hand_open,
    home,
    move,
)
from . import test_view


def get_tree() -> Behaviour:
    """Create the scripted grab-and-place demo behaviour tree.

    This tree executes a fixed demonstration sequence including homing,
    scene snapshot acquisition, grasping predefined fruits, placing them
    into predefined cupboard positions, and returning the manipulator to
    the home position.

    Returns:
        Behaviour: Root behaviour of the configured grab-and-place demo tree.
    """
    return Sequence(
        "GrabSequence",
        memory=True,
        children=[
            # QueueSpeech("Hello. I am homing the arm."),
            Timer("InitialWait", 3.0),
            # hand_close.get_tree(),
            home.get_tree(),
            # 3 snapshots
            # QueueSpeech("I will now take three snapshots of the scene. Please hold still."),
            test_view.get_tree(),
            ### Grip mango subtree right front
            # QueueSpeech("Alright. Preparing to grab the mango..."),
            move.get_tree(
                x=-0.464,
                y=0.056,
                z=0.424,
                ox=0.786,
                oy=-0.607,
                oz=-0.106,
                ow=0.023,
            ),
            Timer("MoveWait", 1.0),
            # hand_open.get_tree(),
            move.get_tree(
                x=-0.481,
                y=0.032,
                z=0.225,
                ox=0.804,
                oy=-0.536,
                oz=-0.220,
                ow=-0.129,
            ),
            Timer("MoveWait", 1.0),
            # hand_close.get_tree(),
            # QueueSpeech("Moving to the cupboard"),
            Timer("GripWait", 1.0),
            move.get_tree(
                x=-0.464,
                y=0.056,
                z=0.424,
                ox=0.786,
                oy=-0.607,
                oz=-0.106,
                ow=0.023,
            ),
            home.get_tree(),
            Timer("Wait before placement", 1.0),
            # placing bottom right
            move.get_tree(
                x=-0.138,
                y=0.823,
                z=0.280,
                ox=-0.203,
                oy=0.666,
                oz=0.707,
                ow=-0.119,
            ),
            Timer("PlaceWait", 1.0),
            move.get_tree(
                x=-0.162,
                y=0.825,
                z=0.234,
                ox=-0.548,
                oy=0.434,
                oz=0.655,
                ow=0.284,
            ),
            Timer("MoveWaite", 1.0),
            # hand_open.get_tree(),
            Timer("ReleaseWait", 1.0),
            move.get_tree(
                x=-0.138,
                y=0.823,
                z=0.280,
                ox=-0.203,
                oy=0.666,
                oz=0.707,
                ow=-0.119,
            ),
            # QueueSpeech("Now homing the arm again."),
            home.get_tree(),
            # QueueSpeech("Alright. Preparing to grab the apple."),
            ### Left side front subtree
            move.get_tree(
                x=-0.370,
                y=0.068,
                z=0.346,
                ox=0.605,
                oy=-0.367,
                oz=-0.407,
                ow=0.576,
            ),
            Timer("MoveWaite", 1.0),
            # hand_open.get_tree(),
            Timer("MoveWaite", 1.0),
            move.get_tree(
                x=-0.414,
                y=-0.057,
                z=0.0730,
                ox=0.533,
                oy=-0.429,
                oz=-0.502,
                ow=0.527,
            ),
            Timer("MoveWaite", 1.0),
            # hand_close.get_tree(),
            # QueueSpeech("I will now move to the cupboard."),
            Timer("MoveWaite", 1.0),
            move.get_tree(
                x=-0.370,
                y=0.068,
                z=0.330,
                ox=0.605,
                oy=-0.367,
                oz=-0.407,
                ow=0.576,
            ),
            Timer("MoveWaite", 1.0),
            home.get_tree(),
            Timer("MoveWaite", 1.0),
            move.get_tree(
                x=-0.352,
                y=0.657,
                z=0.270,
                ox=-0.270,
                oy=0.6166,
                oz=0.698,
                ow=-0.243,
            ),
            Timer("MoveWaite", 1.0),
            move.get_tree(
                x=-0.354,
                y=0.66,
                z=0.145,
                ox=-0.281,
                oy=0.649,
                oz=0.665,
                ow=-0.235,
            ),
            Timer("MoveWaite", 1.0),
            # hand_open.get_tree(),
            Timer("MoveWaite", 1.0),
            move.get_tree(
                x=-0.352,
                y=0.657,
                z=0.270,
                ox=-0.270,
                oy=0.6166,
                oz=0.698,
                ow=-0.243,
            ),
            # QueueSpeech("Homing."),
            Timer("MoveWaite", 1.0),
            home.get_tree(),
            # QueueSpeech("And I am done. Thank you for your attention."),
        ],
    )
