"""Mock high-level py_trees actions and completion checkpoints.

Nothing in this file drives the robot. Replace TODOs in actual ROS actions
before using the tree with hardware.
"""

from dataclasses import dataclass, field
from typing import Callable, Set

import py_trees


@dataclass
class TaskContext:
    """In-memory execution checkpoint; not persisted across process restarts."""

    completed: Set[str] = field(default_factory=set)
    failed_once: Set[str] = field(default_factory=set)
    simulate_actions: bool = False
    fail_step_once: str = ""
    log: Callable[[str], None] = print


class ProcessAction(py_trees.behaviour.Behaviour):
    """One process per leaf, implemented in a later task-specific ROS node.

    The real action must implement non-blocking start/poll/cancel semantics.
    In non-simulated mode this placeholder intentionally NEVER succeeds.
    """

    def __init__(self, step_id: str, name: str, context: TaskContext):
        super().__init__(name=name)
        self.step_id = step_id
        self.context = context
        self._updates = 0

    def initialise(self) -> None:
        self._updates = 0
        # TODO: Start an asynchronous action goal, unless step completed.

    def update(self) -> py_trees.common.Status:
        if self.step_id in self.context.completed:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING
        if not self.context.simulate_actions:
            # TODO: Poll task-specific ROS action/service for goal feedback.
            # TODO: Return SUCCESS only after the real result is confirmed.
            # TODO: Return FAILURE with a structured reason on a real error.
            return py_trees.common.Status.RUNNING

        self._updates += 1
        if self._updates < 2:
            return py_trees.common.Status.RUNNING
        if (
            self.context.fail_step_once == self.step_id
            and self.step_id not in self.context.failed_once
        ):
            self.context.failed_once.add(self.step_id)
            self.context.log(f"DEMO injected error: {self.step_id}")
            return py_trees.common.Status.FAILURE
        self.context.completed.add(self.step_id)
        self.context.log(f"TASK completed: {self.step_id}")
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: py_trees.common.Status) -> None:
        if new_status == py_trees.common.Status.INVALID:
            # TODO: Cancel the in-flight ROS action goal and verify stopping.
            # TODO: Do not claim physical suspension from BT invalidation.
            self.context.log(f"CANCEL requested: {self.step_id}")
