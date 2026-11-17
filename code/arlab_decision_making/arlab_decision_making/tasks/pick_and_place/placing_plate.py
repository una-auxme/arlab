"""Placing the Plate in the Dishwasher (PDF): editable per-action py_trees behaviours.
Each update() writes the current process/action to the shared Blackboard.
This is a non-driving prototype; every real ROS step must be implemented."""

import py_trees
from py_trees.common import Access, Status
from ...resume_conditions import with_action_resume, with_process_resume

PROCESS_ID = 'pick.plate'


class LocalizePlateOnTable(py_trees.behaviour.Behaviour):
    """PDF state: Localize-the-Plate-on-Table."""

    PROCESS_ID = PROCESS_ID
    ACTION_ID = 'pick.plate.localize_plate_on_table'

    def __init__(self):
        super().__init__(name='Localize Plate on Table')
        self.bb = py_trees.blackboard.Client(name='pick_and_place:LocalizePlateOnTable')
        self.bb.register_key("memory_process_name", access=Access.WRITE)
        self.bb.register_key("memory_action_name", access=Access.WRITE)
        self._ticks = 0

    def initialise(self):
        self._ticks = 0

    def update(self):
        self.bb.memory_process_name = self.PROCESS_ID
        self.bb.memory_action_name = self.ACTION_ID

        # TODO: Find plate on table using perception.
        # Nur für den Strukturtest: nach 2 Ticks erfolgreich.
        # Bei echter ROS-Integration durch Ergebnisprüfung ersetzen.
        self._ticks += 1
        if self._ticks < 2:
            return Status.RUNNING
        self.logger.info(f'{self.ACTION_ID} Succeeded')
        return Status.SUCCESS

    def terminate(self, new_status):
        pass


class GrabPlateOnTable(py_trees.behaviour.Behaviour):
    """PDF state: Grab-the-Plate-on-Table."""

    PROCESS_ID = PROCESS_ID
    ACTION_ID = 'pick.plate.grab_plate_on_table'

    def __init__(self):
        super().__init__(name='Grab Plate on Table')
        self.bb = py_trees.blackboard.Client(name='pick_and_place:GrabPlateOnTable')
        self.bb.register_key("memory_process_name", access=Access.WRITE)
        self.bb.register_key("memory_action_name", access=Access.WRITE)
        self._ticks = 0

    def initialise(self):
        self._ticks = 0

    def update(self):
        self.bb.memory_process_name = self.PROCESS_ID
        self.bb.memory_action_name = self.ACTION_ID

        # TODO: Plan and execute plate grasp; validate success.
        # Nur für den Strukturtest: nach 2 Ticks erfolgreich.
        # Bei echter ROS-Integration durch Ergebnisprüfung ersetzen.
        self._ticks += 1
        if self._ticks < 2:
            return Status.RUNNING
        self.logger.info(f'{self.ACTION_ID} Succeeded')
        return Status.SUCCESS

    def terminate(self, new_status):
        pass


class PlacePlateInDishwasherRack(py_trees.behaviour.Behaviour):
    """PDF state: Place-Plate-in-Dishwasher-Rack."""

    PROCESS_ID = PROCESS_ID
    ACTION_ID = 'pick.plate.place_plate_in_dishwasher_rack'

    def __init__(self):
        super().__init__(name='Place Plate in Dishwasher Rack')
        self.bb = py_trees.blackboard.Client(name='pick_and_place:PlacePlateInDishwasherRack')
        self.bb.register_key("memory_process_name", access=Access.WRITE)
        self.bb.register_key("memory_action_name", access=Access.WRITE)
        self._ticks = 0

    def initialise(self):
        self._ticks = 0

    def update(self):
        self.bb.memory_process_name = self.PROCESS_ID
        self.bb.memory_action_name = self.ACTION_ID

        # TODO: Place plate in rack and confirm release.
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
    """Build Placing Plate as an ordered py_trees sequence."""
    sequence = py_trees.composites.Sequence(
        name='Placing Plate',
        memory=True,
        children=[
            with_action_resume(LocalizePlateOnTable()),
            with_action_resume(GrabPlateOnTable()),
            with_action_resume(PlacePlateInDishwasherRack()),
        ],
    )
    return with_process_resume(PROCESS_ID, sequence)

