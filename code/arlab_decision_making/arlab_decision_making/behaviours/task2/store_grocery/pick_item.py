from py_trees.behaviour import Behaviour
from py_trees.common import Status


def get_tree() -> Behaviour:
    return PickItem()


class PickItem(Behaviour):
    def __init__(self, name: str = "PickItem"):
        super().__init__(name=type(self).__name__)

    def update(self):
        self.feedback_message = "not implemented"
        return Status.FAILURE
