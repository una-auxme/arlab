"""Fetching Cereal (PDF): editable per-action py_trees behaviours.
Each update() writes the current process/action to the shared Blackboard.
This is a non-driving prototype; every real ROS step must be implemented."""

import py_trees
from py_trees.common import Access, Status
from ...resume_conditions import with_action_resume, with_process_resume

PROCESS_ID = 'pick.cereal'


class LocalizeCereal(py_trees.behaviour.Behaviour):
    """PDF state: Localize-Cereal."""

    PROCESS_ID = PROCESS_ID
    ACTION_ID = 'pick.cereal.localize_cereal'

    def __init__(self):
        super().__init__(name='Localize Cereal')
        self.bb = py_trees.blackboard.Client(name='pick_and_place:LocalizeCereal')
        self.bb.register_key("memory_process_name", access=Access.WRITE)
        self.bb.register_key("memory_action_name", access=Access.WRITE)
        self._ticks = 0

    def initialise(self):
        self._ticks = 0

    def update(self):
        self.bb.memory_process_name = self.PROCESS_ID
        self.bb.memory_action_name = self.ACTION_ID

        # TODO: Find cereal on shelf.
        # Nur für den Strukturtest: nach 2 Ticks erfolgreich.
        # Bei echter ROS-Integration durch Ergebnisprüfung ersetzen.
        self._ticks += 1
        if self._ticks < 2:
            return Status.RUNNING
        self.logger.info(f'{self.ACTION_ID} Succeeded')
        return Status.SUCCESS

    def terminate(self, new_status):
        pass


class GrabCereal(py_trees.behaviour.Behaviour):
    """PDF state: Grab-Cereal."""

    PROCESS_ID = PROCESS_ID
    ACTION_ID = 'pick.cereal.grab_cereal'

    def __init__(self):
        super().__init__(name='Grab Cereal')
        self.bb = py_trees.blackboard.Client(name='pick_and_place:GrabCereal')
        self.bb.register_key("memory_process_name", access=Access.WRITE)
        self.bb.register_key("memory_action_name", access=Access.WRITE)
        self._ticks = 0

    def initialise(self):
        self._ticks = 0

    def update(self):
        self.bb.memory_process_name = self.PROCESS_ID
        self.bb.memory_action_name = self.ACTION_ID

        # TODO: Grasp cereal package and check stability.
        # Nur für den Strukturtest: nach 2 Ticks erfolgreich.
        # Bei echter ROS-Integration durch Ergebnisprüfung ersetzen.
        self._ticks += 1
        if self._ticks < 2:
            return Status.RUNNING
        self.logger.info(f'{self.ACTION_ID} Succeeded')
        return Status.SUCCESS

    def terminate(self, new_status):
        pass


class PlaceCerealOnTable(py_trees.behaviour.Behaviour):
    """PDF state: Place-Cereal-on-Table."""

    PROCESS_ID = PROCESS_ID
    ACTION_ID = 'pick.cereal.place_cereal_on_table'

    def __init__(self):
        super().__init__(name='Place Cereal on Table')
        self.bb = py_trees.blackboard.Client(name='pick_and_place:PlaceCerealOnTable')
        self.bb.register_key("memory_process_name", access=Access.WRITE)
        self.bb.register_key("memory_action_name", access=Access.WRITE)
        self._ticks = 0

    def initialise(self):
        self._ticks = 0

    def update(self):
        self.bb.memory_process_name = self.PROCESS_ID
        self.bb.memory_action_name = self.ACTION_ID

        # TODO: Place cereal package on the table.
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
    """Build Fetching Cereal as an ordered py_trees sequence."""
    sequence = py_trees.composites.Sequence(
        name='Fetching Cereal',
        memory=True,
        children=[
            with_action_resume(LocalizeCereal()),
            with_action_resume(GrabCereal()),
            with_action_resume(PlaceCerealOnTable()),
        ],
    )
    return with_process_resume(PROCESS_ID, sequence)

