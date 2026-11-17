"""Entering and Localization (PDF): editable per-action py_trees behaviours.
Each update() writes the current process/action to the shared Blackboard.
This is a non-driving prototype; every real ROS step must be implemented."""

import py_trees
from py_trees.common import Access, Status
from ...resume_conditions import with_action_resume, with_process_resume

PROCESS_ID = 'laundry.enter'


class WaitingState(py_trees.behaviour.Behaviour):
    """PDF state: Waiting-State."""

    PROCESS_ID = PROCESS_ID
    ACTION_ID = 'laundry.enter.waiting_state'

    def __init__(self):
        super().__init__(name='Waiting State')
        self.bb = py_trees.blackboard.Client(name='laundry:WaitingState')
        self.bb.register_key("memory_process_name", access=Access.WRITE)
        self.bb.register_key("memory_action_name", access=Access.WRITE)
        self._ticks = 0

    def initialise(self):
        self._ticks = 0

    def update(self):
        self.bb.memory_process_name = self.PROCESS_ID
        self.bb.memory_action_name = self.ACTION_ID

        # TODO: Read a real door-open signal before returning SUCCESS.
        # Nur für den Strukturtest: nach 2 Ticks erfolgreich.
        # Bei echter ROS-Integration durch Ergebnisprüfung ersetzen.
        self._ticks += 1
        if self._ticks < 2:
            return Status.RUNNING
        self.logger.info(f'{self.ACTION_ID} Succeeded')
        return Status.SUCCESS

    def terminate(self, new_status):
        pass


class NavigationToLaundryRoom(py_trees.behaviour.Behaviour):
    """PDF state: Navigation-to-Laundry-Room."""

    PROCESS_ID = PROCESS_ID
    ACTION_ID = 'laundry.enter.navigation_to_laundry_room'

    def __init__(self):
        super().__init__(name='Navigation to Laundry Room')
        self.bb = py_trees.blackboard.Client(name='laundry:NavigationToLaundryRoom')
        self.bb.register_key("memory_process_name", access=Access.WRITE)
        self.bb.register_key("memory_action_name", access=Access.WRITE)
        self._ticks = 0

    def initialise(self):
        self._ticks = 0

    def update(self):
        self.bb.memory_process_name = self.PROCESS_ID
        self.bb.memory_action_name = self.ACTION_ID

        # TODO: Send a Nav2 NavigateToPose goal and check the action result.
        # Nur für den Strukturtest: nach 2 Ticks erfolgreich.
        # Bei echter ROS-Integration durch Ergebnisprüfung ersetzen.
        self._ticks += 1
        if self._ticks < 2:
            return Status.RUNNING
        self.logger.info(f'{self.ACTION_ID} Succeeded')
        return Status.SUCCESS

    def terminate(self, new_status):
        pass


class LocalizeWashingMachine(py_trees.behaviour.Behaviour):
    """PDF state: Localize-the-Washing-Machine."""

    PROCESS_ID = PROCESS_ID
    ACTION_ID = 'laundry.enter.localize_washing_machine'

    def __init__(self):
        super().__init__(name='Localize Washing Machine')
        self.bb = py_trees.blackboard.Client(name='laundry:LocalizeWashingMachine')
        self.bb.register_key("memory_process_name", access=Access.WRITE)
        self.bb.register_key("memory_action_name", access=Access.WRITE)
        self._ticks = 0

    def initialise(self):
        self._ticks = 0

    def update(self):
        self.bb.memory_process_name = self.PROCESS_ID
        self.bb.memory_action_name = self.ACTION_ID

        # TODO: Request perception/knowledge-base detection and validate location.
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
    """Build Entering And Localization as an ordered py_trees sequence."""
    sequence = py_trees.composites.Sequence(
        name='Entering And Localization',
        memory=True,
        children=[
            with_action_resume(WaitingState()),
            with_action_resume(NavigationToLaundryRoom()),
            with_action_resume(LocalizeWashingMachine()),
        ],
    )
    return with_process_resume(PROCESS_ID, sequence)

