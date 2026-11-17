"""Completion (PDF): editable per-action py_trees behaviours.
Each update() writes the current process/action to the shared Blackboard.
This is a non-driving prototype; every real ROS step must be implemented."""

import py_trees
from py_trees.common import Access, Status
from ...resume_conditions import with_action_resume, with_process_resume

PROCESS_ID = 'pick.complete'


class AnnounceTaskCompletion(py_trees.behaviour.Behaviour):
    """PDF state: Announce-Task-Completion."""

    PROCESS_ID = PROCESS_ID
    ACTION_ID = 'pick.complete.announce_task_completion'

    def __init__(self):
        super().__init__(name='Announce Task Completion')
        self.bb = py_trees.blackboard.Client(name='pick_and_place:AnnounceTaskCompletion')
        self.bb.register_key("memory_process_name", access=Access.WRITE)
        self.bb.register_key("memory_action_name", access=Access.WRITE)
        self._ticks = 0

    def initialise(self):
        self._ticks = 0

    def update(self):
        self.bb.memory_process_name = self.PROCESS_ID
        self.bb.memory_action_name = self.ACTION_ID

        # TODO: Announce task completion and await completion confirmation.
        # Nur für den Strukturtest: nach 2 Ticks erfolgreich.
        # Bei echter ROS-Integration durch Ergebnisprüfung ersetzen.
        self._ticks += 1
        if self._ticks < 2:
            return Status.RUNNING
        self.logger.info(f'{self.ACTION_ID} Succeeded')
        return Status.SUCCESS

    def terminate(self, new_status):
        pass


def create_process():
    """Build Completion as an ordered py_trees sequence."""
    sequence = py_trees.composites.Sequence(
        name='Completion',
        memory=True,
        children=[
            with_action_resume(AnnounceTaskCompletion()),
        ],
    )
    return with_process_resume(PROCESS_ID, sequence)

