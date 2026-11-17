"""Fetching Milk (PDF): editable per-action py_trees behaviours.
Each update() writes the current process/action to the shared Blackboard.
This is a non-driving prototype; every real ROS step must be implemented."""

import py_trees
from py_trees.common import Access, Status
from ...resume_conditions import with_action_resume, with_process_resume

PROCESS_ID = 'pick.milk'


class LocalizeMilk(py_trees.behaviour.Behaviour):
    """PDF state: Localize-Milk."""

    PROCESS_ID = PROCESS_ID
    ACTION_ID = 'pick.milk.localize_milk'

    def __init__(self):
        super().__init__(name='Localize Milk')
        self.bb = py_trees.blackboard.Client(name='pick_and_place:LocalizeMilk')
        self.bb.register_key("memory_process_name", access=Access.WRITE)
        self.bb.register_key("memory_action_name", access=Access.WRITE)
        self._ticks = 0

    def initialise(self):
        self._ticks = 0

    def update(self):
        self.bb.memory_process_name = self.PROCESS_ID
        self.bb.memory_action_name = self.ACTION_ID

        # TODO: Find milk on shelf.
        # Nur für den Strukturtest: nach 2 Ticks erfolgreich.
        # Bei echter ROS-Integration durch Ergebnisprüfung ersetzen.
        self._ticks += 1
        if self._ticks < 2:
            return Status.RUNNING
        self.logger.info(f'{self.ACTION_ID} Succeeded')
        return Status.SUCCESS

    def terminate(self, new_status):
        pass


class GrabMilk(py_trees.behaviour.Behaviour):
    """PDF state: Grab-Milk."""

    PROCESS_ID = PROCESS_ID
    ACTION_ID = 'pick.milk.grab_milk'

    def __init__(self):
        super().__init__(name='Grab Milk')
        self.bb = py_trees.blackboard.Client(name='pick_and_place:GrabMilk')
        self.bb.register_key("memory_process_name", access=Access.WRITE)
        self.bb.register_key("memory_action_name", access=Access.WRITE)
        self._ticks = 0

    def initialise(self):
        self._ticks = 0

    def update(self):
        self.bb.memory_process_name = self.PROCESS_ID
        self.bb.memory_action_name = self.ACTION_ID

        # TODO: Grasp milk package and check stability.
        # Nur für den Strukturtest: nach 2 Ticks erfolgreich.
        # Bei echter ROS-Integration durch Ergebnisprüfung ersetzen.
        self._ticks += 1
        if self._ticks < 2:
            return Status.RUNNING
        self.logger.info(f'{self.ACTION_ID} Succeeded')
        return Status.SUCCESS

    def terminate(self, new_status):
        pass


class PlaceMilkOnTable(py_trees.behaviour.Behaviour):
    """PDF state: Place-Milk-on-Table."""

    PROCESS_ID = PROCESS_ID
    ACTION_ID = 'pick.milk.place_milk_on_table'

    def __init__(self):
        super().__init__(name='Place Milk on Table')
        self.bb = py_trees.blackboard.Client(name='pick_and_place:PlaceMilkOnTable')
        self.bb.register_key("memory_process_name", access=Access.WRITE)
        self.bb.register_key("memory_action_name", access=Access.WRITE)
        self._ticks = 0

    def initialise(self):
        self._ticks = 0

    def update(self):
        self.bb.memory_process_name = self.PROCESS_ID
        self.bb.memory_action_name = self.ACTION_ID

        # TODO: Place milk on the table.
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
    """Build Fetching Milk as an ordered py_trees sequence."""
    sequence = py_trees.composites.Sequence(
        name='Fetching Milk',
        memory=True,
        children=[
            with_action_resume(LocalizeMilk()),
            with_action_resume(GrabMilk()),
            with_action_resume(PlaceMilkOnTable()),
        ],
    )
    return with_process_resume(PROCESS_ID, sequence)

