from py_trees.behaviour import Behaviour
from py_trees.common import Status


def get_tree() -> Behaviour:
    raise NotImplementedError()


class MoveToStartingPosition(Behaviour):
    def update(self):
        self.feedback_message = "not implemented"
        return Status.FAILURE
