"""Disposing of Trash (PDF): editable per-action py_trees behaviours.
Each update() writes the current process/action to the shared Blackboard.
This is a non-driving prototype; every real ROS step must be implemented."""

import py_trees
from py_trees.common import Access, Status
from ...resume_conditions import with_action_resume, with_process_resume

PROCESS_ID = 'pick.trash'


class LocalizeTrashObjectOnTable(py_trees.behaviour.Behaviour):
    """PDF state: Localize-Trash-Object-on-Table."""

    PROCESS_ID = PROCESS_ID
    ACTION_ID = 'pick.trash.localize_trash_object_on_table'

    def __init__(self):
        super().__init__(name='Localize Trash Object on Table')
        self.bb = py_trees.blackboard.Client(name='pick_and_place:LocalizeTrashObjectOnTable')
        self.bb.register_key("memory_process_name", access=Access.WRITE)
        self.bb.register_key("memory_action_name", access=Access.WRITE)
        self._ticks = 0

    def initialise(self):
        self._ticks = 0

    def update(self):
        self.bb.memory_process_name = self.PROCESS_ID
        self.bb.memory_action_name = self.ACTION_ID

        # TODO: Locate object to dispose of.
        # Nur für den Strukturtest: nach 2 Ticks erfolgreich.
        # Bei echter ROS-Integration durch Ergebnisprüfung ersetzen.
        self._ticks += 1
        if self._ticks < 2:
            return Status.RUNNING
        self.logger.info(f'{self.ACTION_ID} Succeeded')
        return Status.SUCCESS

    def terminate(self, new_status):
        pass


class GrabTrashObjectOnTable(py_trees.behaviour.Behaviour):
    """PDF state: Grab-Trash-Object-on-Table."""

    PROCESS_ID = PROCESS_ID
    ACTION_ID = 'pick.trash.grab_trash_object_on_table'

    def __init__(self):
        super().__init__(name='Grab Trash Object on Table')
        self.bb = py_trees.blackboard.Client(name='pick_and_place:GrabTrashObjectOnTable')
        self.bb.register_key("memory_process_name", access=Access.WRITE)
        self.bb.register_key("memory_action_name", access=Access.WRITE)
        self._ticks = 0

    def initialise(self):
        self._ticks = 0

    def update(self):
        self.bb.memory_process_name = self.PROCESS_ID
        self.bb.memory_action_name = self.ACTION_ID

        # TODO: Grasp identified trash.
        # Nur für den Strukturtest: nach 2 Ticks erfolgreich.
        # Bei echter ROS-Integration durch Ergebnisprüfung ersetzen.
        self._ticks += 1
        if self._ticks < 2:
            return Status.RUNNING
        self.logger.info(f'{self.ACTION_ID} Succeeded')
        return Status.SUCCESS

    def terminate(self, new_status):
        pass


class DisposeTrashObject(py_trees.behaviour.Behaviour):
    """PDF state: Dispose-Trash-Object."""

    PROCESS_ID = PROCESS_ID
    ACTION_ID = 'pick.trash.dispose_trash_object'

    def __init__(self):
        super().__init__(name='Dispose Trash Object')
        self.bb = py_trees.blackboard.Client(name='pick_and_place:DisposeTrashObject')
        self.bb.register_key("memory_process_name", access=Access.WRITE)
        self.bb.register_key("memory_action_name", access=Access.WRITE)
        self._ticks = 0

    def initialise(self):
        self._ticks = 0

    def update(self):
        self.bb.memory_process_name = self.PROCESS_ID
        self.bb.memory_action_name = self.ACTION_ID

        # TODO: Navigate/release trash into correct bin and verify.
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
    """Build Disposing Of Trash as an ordered py_trees sequence."""
    sequence = py_trees.composites.Sequence(
        name='Disposing Of Trash',
        memory=True,
        children=[
            with_action_resume(LocalizeTrashObjectOnTable()),
            with_action_resume(GrabTrashObjectOnTable()),
            with_action_resume(DisposeTrashObject()),
        ],
    )
    return with_process_resume(PROCESS_ID, sequence)

