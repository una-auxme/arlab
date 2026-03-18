"""Vision snapshot demo behaviour tree for py_trees.

Provides a predefined behaviour tree sequence to:
    - wait before starting execution
    - prepare and home the manipulator
    - move to predefined snapshot poses
    - execute multiple vision snapshot actions
    - optionally clear the vision database before the first snapshot

Maintainers:
    Peter Viechter <peter.viechter@uni-augsburg.de>
    Daniel Gabler <daniel.gabler@uni-augsburg.de>
"""

from py_trees.behaviour import Behaviour
from py_trees.composites import Sequence
from py_trees.timers import Timer

from ..common import vision_snapshot
from ..manipulation import (
    hand_close,
    home,
    move,
)


def get_tree() -> Behaviour:
    """Create the vision snapshot demo behaviour tree.

    This tree waits briefly, prepares and homes the manipulator, moves to
    three predefined observation poses, and executes vision snapshot actions
    at each pose. The first snapshot clears the vision database, while the
    following snapshots extend the current scene information.

    Returns:
        Behaviour: Root behaviour of the configured vision snapshot demo
            sequence.
    """
    return Sequence(
        "GrabSequence",
        memory=True,
        children=[
            Timer("InitialWait", 3.0),
            hand_close.get_tree(),
            home.get_tree(),
            Timer("MoveWait", 2.5),
            move.get_tree(  # picture 1
                x=0.131,
                y=0.021,
                z=0.729,
                ox=-0.062,
                oy=0.951,
                oz=0.302,
                ow=0.018,
            ),
            vision_snapshot.get_tree(clear=True),
            move.get_tree(  # picture 2 (Schrank)
                x=0.091,
                y=0.087,
                z=0.749,
                ox=-0.304,
                oy=0.741,
                oz=0.584,
                ow=-0.127,
            ),
            vision_snapshot.get_tree(clear=False),
            move.get_tree(  # picture 3
                x=-0.039,
                y=0.119,
                z=0.729,
                ox=0.768,
                oy=-0.557,
                oz=-0.202,
                ow=0.240,
            ),
            vision_snapshot.get_tree(clear=False),
        ],
    )