"""Sorting Another Object into the Shelf (PDF): editable per-action py_trees behaviours.
Each update() writes the current process/action to the shared Blackboard.
This is a non-driving prototype; every real ROS step must be implemented."""

import py_trees
from py_trees.common import Access, Status
from ...resume_conditions import with_action_resume, with_process_resume

PROCESS_ID = 'pick.shelf'


class IdentifyObjectCategory(py_trees.behaviour.Behaviour):
    """PDF state: Identify-Object-Category."""

    PROCESS_ID = PROCESS_ID
    ACTION_ID = 'pick.shelf.identify_object_category'

    def __init__(self):
        super().__init__(name='Identify Object Category')
        self.bb = py_trees.blackboard.Client(name='pick_and_place:IdentifyObjectCategory')
        self.bb.register_key("memory_process_name", access=Access.WRITE)
        self.bb.register_key("memory_action_name", access=Access.WRITE)
        self._ticks = 0

    def initialise(self):
        self._ticks = 0

    def update(self):
        self.bb.memory_process_name = self.PROCESS_ID
        self.bb.memory_action_name = self.ACTION_ID

        # TODO: Classify the object using vision/knowledge base.
        # Nur für den Strukturtest: nach 2 Ticks erfolgreich.
        # Bei echter ROS-Integration durch Ergebnisprüfung ersetzen.
        self._ticks += 1
        if self._ticks < 2:
            return Status.RUNNING
        self.logger.info(f'{self.ACTION_ID} Succeeded')
        return Status.SUCCESS

    def terminate(self, new_status):
        pass


class DetermineCorrectShelfPosition(py_trees.behaviour.Behaviour):
    """PDF state: Determine-Correct-Shelf-Position."""

    PROCESS_ID = PROCESS_ID
    ACTION_ID = 'pick.shelf.determine_correct_shelf_position'

    def __init__(self):
        super().__init__(name='Determine Correct Shelf Position')
        self.bb = py_trees.blackboard.Client(name='pick_and_place:DetermineCorrectShelfPosition')
        self.bb.register_key("memory_process_name", access=Access.WRITE)
        self.bb.register_key("memory_action_name", access=Access.WRITE)
        self._ticks = 0

    def initialise(self):
        self._ticks = 0

    def update(self):
        self.bb.memory_process_name = self.PROCESS_ID
        self.bb.memory_action_name = self.ACTION_ID

        # TODO: Select suitable shelf pose with semantic rules.
        # Nur für den Strukturtest: nach 2 Ticks erfolgreich.
        # Bei echter ROS-Integration durch Ergebnisprüfung ersetzen.
        self._ticks += 1
        if self._ticks < 2:
            return Status.RUNNING
        self.logger.info(f'{self.ACTION_ID} Succeeded')
        return Status.SUCCESS

    def terminate(self, new_status):
        pass


class SortObjectIntoShelf(py_trees.behaviour.Behaviour):
    """PDF state: Sort-Other-Object-Into-Shelf."""

    PROCESS_ID = PROCESS_ID
    ACTION_ID = 'pick.shelf.sort_object_into_shelf'

    def __init__(self):
        super().__init__(name='Sort Object Into Shelf')
        self.bb = py_trees.blackboard.Client(name='pick_and_place:SortObjectIntoShelf')
        self.bb.register_key("memory_process_name", access=Access.WRITE)
        self.bb.register_key("memory_action_name", access=Access.WRITE)
        self._ticks = 0

    def initialise(self):
        self._ticks = 0

    def update(self):
        self.bb.memory_process_name = self.PROCESS_ID
        self.bb.memory_action_name = self.ACTION_ID

        # TODO: Pick and place to the chosen shelf pose; check result.
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
    """Build Sorting Into Shelf as an ordered py_trees sequence."""
    sequence = py_trees.composites.Sequence(
        name='Sorting Into Shelf',
        memory=True,
        children=[
            with_action_resume(IdentifyObjectCategory()),
            with_action_resume(DetermineCorrectShelfPosition()),
            with_action_resume(SortObjectIntoShelf()),
        ],
    )
    return with_process_resume(PROCESS_ID, sequence)

