"""Home and move demo behaviour tree for py_trees.

Provides a simple behaviour tree sequence to:
    - wait before starting execution
    - move the manipulator to the home position
    - wait again after homing
    - execute a move behaviour

Maintainers:
    Peter Viechter <peter.viechter@uni-augsburg.de>
    Daniel Gabler <daniel.gabler@uni-augsburg.de>
"""

from py_trees.behaviour import Behaviour
from py_trees.composites import Sequence
from py_trees.timers import Timer

from ..manipulation import home, move


def get_tree() -> Behaviour:
    """Create the home-and-move demo behaviour tree.

    This tree waits for an initial delay, executes the homing behaviour,
    waits again, and then executes the move behaviour.

    Returns:
        Behaviour: Root behaviour of the configured home-and-move sequence.
    """
    return Sequence(
        "MoveHomeSeq",
        memory=True,
        children=[
            Timer("InitialWait", 5.0),
            home.get_tree(),
            Timer("MoveWait", 5.0),
            move.get_tree(),
        ],
    )