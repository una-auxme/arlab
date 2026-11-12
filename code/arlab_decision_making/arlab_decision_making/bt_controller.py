"""Start, stop, tick and rebuild the selected py_trees task."""

import py_trees

from . import laundry, pick_and_place
from .prototype_actions import TaskContext


class BehaviorTreeController:
    """In-memory checkpoint-based resume, NOT continuous motion resume."""

    BUILDERS = {
        "laundry": laundry.create_tree,
        "pick_and_place": pick_and_place.create_tree,
    }

    def __init__(self, task: str, context: TaskContext) -> None:
        if task not in self.BUILDERS:
            raise ValueError(f"Unknown task '{task}'; use laundry/pick_and_place")
        self.task = task
        self.context = context
        self.tree = None
        self.tick_count = 0
        self.is_paused = False

    def start(self) -> None:
        """Build/rebuild the BT; already completed steps are skipped."""
        self.tree = py_trees.trees.BehaviourTree(
            self.BUILDERS[self.task](self.context)
        )
        self.is_paused = False
        self.context.log(f"BT built: {self.task} (done={sorted(self.context.completed)})")

    def tick(self) -> str:
        if self.tree is None:
            raise RuntimeError("Call start before tick")
        if self.is_paused:
            raise RuntimeError("Cannot tick paused tree")
        self.tree.tick()
        self.tick_count += 1
        return self.tree.root.status.name

    def pause(self) -> None:
        """Request cancellation; actual ROS action cancellation is TODO."""
        if self.tree is not None and not self.is_paused:
            self.tree.root.stop(py_trees.common.Status.INVALID)
            self.is_paused = True
            self.context.log("BT paused, current process INVALIDATED")

#    def restart(self) -> None:
#        """Restart incomplete process; completed process checkpoints persist."""
#        self.is_paused = False

    def resume(self) -> None:
        """Restart incomplete process; completed process checkpoints persist."""
        self.start()

    def shutdown(self) -> None:
        if self.tree is not None:
            self.pause()
            self.tree.shutdown()
