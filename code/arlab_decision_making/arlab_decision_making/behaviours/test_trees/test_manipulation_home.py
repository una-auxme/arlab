"""Home behaviour tree wrapper for py_trees.

Provides a simple wrapper to:
    - expose the manipulator home behaviour as a subtree entry point

Maintainers:
    Peter Viechter <peter.viechter@uni-augsburg.de>
    Daniel Gabler <daniel.gabler@uni-augsburg.de>
"""

from py_trees.behaviour import Behaviour

from ..manipulation import home


def get_tree() -> Behaviour:
    """Create the behaviour tree subtree for the home action.

    Returns:
        Behaviour: Root behaviour of the home subtree.
    """
    return home.get_tree()