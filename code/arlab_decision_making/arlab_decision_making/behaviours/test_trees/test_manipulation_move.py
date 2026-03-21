"""Move behaviour tree wrapper for py_trees.

Provides a simple wrapper to:
    - return a predefined manipulator move subtree
    - configure the subtree with a fixed target pose

Maintainers:
    Peter Viechter <peter.viechter@uni-a.de>
    Daniel Gabler <daniel.gabler@uni-a.de>
"""

from py_trees.behaviour import Behaviour

from ..manipulation import move


def get_tree() -> Behaviour:
    """Create the behaviour tree subtree for a predefined move action.

    Returns:
        Behaviour: Root behaviour of the configured move subtree.
    """
    return move.get_tree(
        x=-0.2922,
        y=0.099,
        z=0.15,
        ox=0.6952,
        oy=-0.5497,
        oz=-0.29057,
        ow=0.3606,
    )
