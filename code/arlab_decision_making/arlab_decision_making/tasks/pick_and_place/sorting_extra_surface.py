"""Sorting Objects from the Extra Surface (PDF): editable per-action py_trees behaviours.
Each update() writes the current process/action to the shared Blackboard.
This is a non-driving prototype; every real ROS step must be implemented."""

import py_trees
from py_trees.common import Access, Status
from ...resume_conditions import with_action_resume, with_process_resume

PROCESS_ID = 'pick.extra'


class LocalizeOtherObjectOnExtraSurface(py_trees.behaviour.Behaviour):
    """PDF state: Localize-Other-Object-on-Extra-Surface."""

    PROCESS_ID = PROCESS_ID
    ACTION_ID = 'pick.extra.localize_other_object_on_extra_surface'

    def __init__(self):
        super().__init__(name='Localize Object on Extra Surface')
        self.bb = py_trees.blackboard.Client(name='pick_and_place:LocalizeOtherObjectOnExtraSurface')
        self.bb.register_key("memory_process_name", access=Access.WRITE)
        self.bb.register_key("memory_action_name", access=Access.WRITE)
        self._ticks = 0

    def initialise(self):
        self._ticks = 0

    def update(self):
        self.bb.memory_process_name = self.PROCESS_ID
        self.bb.memory_action_name = self.ACTION_ID

        # TODO: Localize target on extra surface.
        # Nur für den Strukturtest: nach 2 Ticks erfolgreich.
        # Bei echter ROS-Integration durch Ergebnisprüfung ersetzen.
        self._ticks += 1
        if self._ticks < 2:
            return Status.RUNNING
        self.logger.info(f'{self.ACTION_ID} Succeeded')
        return Status.SUCCESS

    def terminate(self, new_status):
        pass


class GrabOtherObjectOnExtraSurface(py_trees.behaviour.Behaviour):
    """PDF state: Grab-Other-Object-on-Extra-Surface."""

    PROCESS_ID = PROCESS_ID
    ACTION_ID = 'pick.extra.grab_other_object_on_extra_surface'

    def __init__(self):
        super().__init__(name='Grab Object on Extra Surface')
        self.bb = py_trees.blackboard.Client(name='pick_and_place:GrabOtherObjectOnExtraSurface')
        self.bb.register_key("memory_process_name", access=Access.WRITE)
        self.bb.register_key("memory_action_name", access=Access.WRITE)
        self._ticks = 0

    def initialise(self):
        self._ticks = 0

    def update(self):
        self.bb.memory_process_name = self.PROCESS_ID
        self.bb.memory_action_name = self.ACTION_ID

        # TODO: Pick up one object from the surface.
        # Nur für den Strukturtest: nach 2 Ticks erfolgreich.
        # Bei echter ROS-Integration durch Ergebnisprüfung ersetzen.
        self._ticks += 1
        if self._ticks < 2:
            return Status.RUNNING
        self.logger.info(f'{self.ACTION_ID} Succeeded')
        return Status.SUCCESS

    def terminate(self, new_status):
        pass


class SortOtherObjectIntoShelf(py_trees.behaviour.Behaviour):
    """PDF state: Sort-Other-Object-Into-Shelf."""

    PROCESS_ID = PROCESS_ID
    ACTION_ID = 'pick.extra.sort_other_object_into_shelf'

    def __init__(self):
        super().__init__(name='Sort Extra Object Into Shelf')
        self.bb = py_trees.blackboard.Client(name='pick_and_place:SortOtherObjectIntoShelf')
        self.bb.register_key("memory_process_name", access=Access.WRITE)
        self.bb.register_key("memory_action_name", access=Access.WRITE)
        self._ticks = 0

    def initialise(self):
        self._ticks = 0

    def update(self):
        self.bb.memory_process_name = self.PROCESS_ID
        self.bb.memory_action_name = self.ACTION_ID

        # TODO: Sort one object and check remaining objects for future loop.
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
    """Build Sorting Extra Surface as an ordered py_trees sequence."""
    sequence = py_trees.composites.Sequence(
        name='Sorting Extra Surface',
        memory=True,
        children=[
            with_action_resume(LocalizeOtherObjectOnExtraSurface()),
            with_action_resume(GrabOtherObjectOnExtraSurface()),
            with_action_resume(SortOtherObjectIntoShelf()),
        ],
    )
    return with_process_resume(PROCESS_ID, sequence)

