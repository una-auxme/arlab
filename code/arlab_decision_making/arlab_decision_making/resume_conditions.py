"""Resume condition nodes inspired by CheckMemoryTop/CheckMemoryUnder.

A selector returns SUCCESS for a *skip*, FAILURE for *execute child*.
Memory is only consumed at the exact interrupted action: all preceding
processes and actions are skipped, while later ones execute normally.
"""

import py_trees
from py_trees.common import Access, Status


class CheckResumeProcess(py_trees.behaviour.Behaviour):
    """Skip other processes while the resume pointer is still active."""

    def __init__(self, process_id: str):
        super().__init__(name=f"Skip process? {process_id}")
        self.process_id = process_id
        self.bb = py_trees.blackboard.Client(name=f"CheckProcess:{process_id}")
        self.bb.register_key("retry_flag", access=Access.READ)
        self.bb.register_key("memory_process_name", access=Access.READ)

    def update(self):
        if self.bb.retry_flag and self.bb.memory_process_name != self.process_id:
            return Status.SUCCESS  # not the interrupted process: skip
        return Status.FAILURE      # run the process subtree


class CheckResumeAction(py_trees.behaviour.Behaviour):
    """Skip earlier actions and consume retry only at the matching one."""

    def __init__(self, process_id: str, action_id: str):
        super().__init__(name=f"Skip action? {action_id}")
        self.process_id = process_id
        self.action_id = action_id
        self.bb = py_trees.blackboard.Client(name=f"CheckAction:{action_id}")
        self.bb.register_key("retry_flag", access=Access.WRITE)
        self.bb.register_key("memory_process_name", access=Access.READ)
        self.bb.register_key("memory_action_name", access=Access.READ)

    def update(self):
        if not self.bb.retry_flag:
            return Status.FAILURE  # normal execution
        if self.bb.memory_process_name != self.process_id:
            return Status.FAILURE  # process condition handles this case
        if self.bb.memory_action_name != self.action_id:
            return Status.SUCCESS  # earlier action -> skip
        self.bb.retry_flag = False  # exact target reached; resume normal BT
        return Status.FAILURE


def with_action_resume(action: py_trees.behaviour.Behaviour):
    """Research thesis: Selector(CheckMemoryUnder, concrete action)."""
    return py_trees.composites.Selector(
        name=f"Resume action: {action.name}",
        memory=True,
        children=[CheckResumeAction(action.PROCESS_ID, action.ACTION_ID), action],
    )


def with_process_resume(process_id: str, subtree):
    """Research thesis: Selector(CheckMemoryTop, process subtree)."""
    return py_trees.composites.Selector(
        name=f"Resume process: {process_id}",
        memory=True,
        children=[CheckResumeProcess(process_id), subtree],
    )
