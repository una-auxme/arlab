"""Placing Cutlery in the Dishwasher (PDF): editable per-action py_trees behaviours.
Each update() writes the current process/action to the shared Blackboard.
This is a non-driving prototype; every real ROS step must be implemented."""

import py_trees
from py_trees.common import Access, Status
from ...resume_conditions import with_action_resume, with_process_resume

PROCESS_ID = 'pick.cutlery'


class LocalizeCutleryOnTable(py_trees.behaviour.Behaviour):
    """PDF state: Localize-Cutlery-on-Table."""

    PROCESS_ID = PROCESS_ID
    ACTION_ID = 'pick.cutlery.localize_cutlery_on_table'

    def __init__(self):
        super().__init__(name='Localize Cutlery on Table')
        self.bb = py_trees.blackboard.Client(name='pick_and_place:LocalizeCutleryOnTable')
        self.bb.register_key("memory_process_name", access=Access.WRITE)
        self.bb.register_key("memory_action_name", access=Access.WRITE)
        self._ticks = 0

    def initialise(self):
        self._ticks = 0

    def update(self):
        self.bb.memory_process_name = self.PROCESS_ID
        self.bb.memory_action_name = self.ACTION_ID

        # TODO: Detect cutlery.
        # Nur für den Strukturtest: nach 2 Ticks erfolgreich.
        # Bei echter ROS-Integration durch Ergebnisprüfung ersetzen.
        self._ticks += 1
        if self._ticks < 2:
            return Status.RUNNING
        self.logger.info(f'{self.ACTION_ID} Succeeded')
        return Status.SUCCESS

    def terminate(self, new_status):
        pass


class GrabCutleryOnTable(py_trees.behaviour.Behaviour):
    """PDF state: Grab-Cutlery-on-Table."""

    PROCESS_ID = PROCESS_ID
    ACTION_ID = 'pick.cutlery.grab_cutlery_on_table'

    def __init__(self):
        super().__init__(name='Grab Cutlery on Table')
        self.bb = py_trees.blackboard.Client(name='pick_and_place:GrabCutleryOnTable')
        self.bb.register_key("memory_process_name", access=Access.WRITE)
        self.bb.register_key("memory_action_name", access=Access.WRITE)
        self._ticks = 0

    def initialise(self):
        self._ticks = 0

    def update(self):
        self.bb.memory_process_name = self.PROCESS_ID
        self.bb.memory_action_name = self.ACTION_ID

        # TODO: Grasp cutlery with appropriate gripper configuration.
        # Nur für den Strukturtest: nach 2 Ticks erfolgreich.
        # Bei echter ROS-Integration durch Ergebnisprüfung ersetzen.
        self._ticks += 1
        if self._ticks < 2:
            return Status.RUNNING
        self.logger.info(f'{self.ACTION_ID} Succeeded')
        return Status.SUCCESS

    def terminate(self, new_status):
        pass


class PlaceCutleryInDishwasherRack(py_trees.behaviour.Behaviour):
    """PDF state: Place-Cutlery-in-Dishwasher-Rack."""

    PROCESS_ID = PROCESS_ID
    ACTION_ID = 'pick.cutlery.place_cutlery_in_dishwasher_rack'

    def __init__(self):
        super().__init__(name='Place Cutlery in Dishwasher Rack')
        self.bb = py_trees.blackboard.Client(name='pick_and_place:PlaceCutleryInDishwasherRack')
        self.bb.register_key("memory_process_name", access=Access.WRITE)
        self.bb.register_key("memory_action_name", access=Access.WRITE)
        self._ticks = 0

    def initialise(self):
        self._ticks = 0

    def update(self):
        self.bb.memory_process_name = self.PROCESS_ID
        self.bb.memory_action_name = self.ACTION_ID

        # TODO: Place cutlery in the dishwasher rack.
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
    """Build Placing Cutlery as an ordered py_trees sequence."""
    sequence = py_trees.composites.Sequence(
        name='Placing Cutlery',
        memory=True,
        children=[
            with_action_resume(LocalizeCutleryOnTable()),
            with_action_resume(GrabCutleryOnTable()),
            with_action_resume(PlaceCutleryInDishwasherRack()),
        ],
    )
    return with_process_resume(PROCESS_ID, sequence)

