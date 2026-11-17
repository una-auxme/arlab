"""Entering and Dishwasher Preparation (PDF): editable per-action py_trees behaviours.
Each update() writes the current process/action to the shared Blackboard.
This is a non-driving prototype; every real ROS step must be implemented."""

import py_trees
from py_trees.common import Access, Status
from ...resume_conditions import with_action_resume, with_process_resume

PROCESS_ID = 'pick.prepare'


class WaitingState(py_trees.behaviour.Behaviour):
    """PDF state: Waiting-State."""

    PROCESS_ID = PROCESS_ID
    ACTION_ID = 'pick.prepare.waiting_state'

    def __init__(self):
        super().__init__(name='Waiting State')
        self.bb = py_trees.blackboard.Client(name='pick_and_place:WaitingState')
        self.bb.register_key("memory_process_name", access=Access.WRITE)
        self.bb.register_key("memory_action_name", access=Access.WRITE)
        self._ticks = 0

    def initialise(self):
        self._ticks = 0

    def update(self):
        self.bb.memory_process_name = self.PROCESS_ID
        self.bb.memory_action_name = self.ACTION_ID

        # TODO: Check door open with sensor/confirmed user signal.
        # Nur für den Strukturtest: nach 2 Ticks erfolgreich.
        # Bei echter ROS-Integration durch Ergebnisprüfung ersetzen.
        self._ticks += 1
        if self._ticks < 2:
            return Status.RUNNING
        self.logger.info(f'{self.ACTION_ID} Succeeded')
        return Status.SUCCESS

    def terminate(self, new_status):
        pass


class NavigationToKitchen(py_trees.behaviour.Behaviour):
    """PDF state: Navigation-to-Kitchen."""

    PROCESS_ID = PROCESS_ID
    ACTION_ID = 'pick.prepare.navigation_to_kitchen'

    def __init__(self):
        super().__init__(name='Navigation to Kitchen')
        self.bb = py_trees.blackboard.Client(name='pick_and_place:NavigationToKitchen')
        self.bb.register_key("memory_process_name", access=Access.WRITE)
        self.bb.register_key("memory_action_name", access=Access.WRITE)
        self._ticks = 0

    def initialise(self):
        self._ticks = 0

    def update(self):
        self.bb.memory_process_name = self.PROCESS_ID
        self.bb.memory_action_name = self.ACTION_ID

        # TODO: Navigate to kitchen using Nav2.
        # Nur für den Strukturtest: nach 2 Ticks erfolgreich.
        # Bei echter ROS-Integration durch Ergebnisprüfung ersetzen.
        self._ticks += 1
        if self._ticks < 2:
            return Status.RUNNING
        self.logger.info(f'{self.ACTION_ID} Succeeded')
        return Status.SUCCESS

    def terminate(self, new_status):
        pass


class RequestOpeningOfDishwasher(py_trees.behaviour.Behaviour):
    """PDF state: Request-Opening-of-Dishwasher."""

    PROCESS_ID = PROCESS_ID
    ACTION_ID = 'pick.prepare.request_opening_of_dishwasher'

    def __init__(self):
        super().__init__(name='Request Opening of Dishwasher')
        self.bb = py_trees.blackboard.Client(name='pick_and_place:RequestOpeningOfDishwasher')
        self.bb.register_key("memory_process_name", access=Access.WRITE)
        self.bb.register_key("memory_action_name", access=Access.WRITE)
        self._ticks = 0

    def initialise(self):
        self._ticks = 0

    def update(self):
        self.bb.memory_process_name = self.PROCESS_ID
        self.bb.memory_action_name = self.ACTION_ID

        # TODO: Ask human to open the dishwasher.
        # Nur für den Strukturtest: nach 2 Ticks erfolgreich.
        # Bei echter ROS-Integration durch Ergebnisprüfung ersetzen.
        self._ticks += 1
        if self._ticks < 2:
            return Status.RUNNING
        self.logger.info(f'{self.ACTION_ID} Succeeded')
        return Status.SUCCESS

    def terminate(self, new_status):
        pass


class WaitForOpeningConfirmation(py_trees.behaviour.Behaviour):
    """PDF state: Wait-for-Opening-Confirmation."""

    PROCESS_ID = PROCESS_ID
    ACTION_ID = 'pick.prepare.wait_for_opening_confirmation'

    def __init__(self):
        super().__init__(name='Wait for Opening Confirmation')
        self.bb = py_trees.blackboard.Client(name='pick_and_place:WaitForOpeningConfirmation')
        self.bb.register_key("memory_process_name", access=Access.WRITE)
        self.bb.register_key("memory_action_name", access=Access.WRITE)
        self._ticks = 0

    def initialise(self):
        self._ticks = 0

    def update(self):
        self.bb.memory_process_name = self.PROCESS_ID
        self.bb.memory_action_name = self.ACTION_ID

        # TODO: Wait for dishwasher-open confirmation.
        # Nur für den Strukturtest: nach 2 Ticks erfolgreich.
        # Bei echter ROS-Integration durch Ergebnisprüfung ersetzen.
        self._ticks += 1
        if self._ticks < 2:
            return Status.RUNNING
        self.logger.info(f'{self.ACTION_ID} Succeeded')
        return Status.SUCCESS

    def terminate(self, new_status):
        pass


class RequestPullingOutDishwasherRack(py_trees.behaviour.Behaviour):
    """PDF state: Request-Pulling-Out-Dishwasher-Rack."""

    PROCESS_ID = PROCESS_ID
    ACTION_ID = 'pick.prepare.request_pulling_out_dishwasher_rack'

    def __init__(self):
        super().__init__(name='Request Pulling Out Dishwasher Rack')
        self.bb = py_trees.blackboard.Client(name='pick_and_place:RequestPullingOutDishwasherRack')
        self.bb.register_key("memory_process_name", access=Access.WRITE)
        self.bb.register_key("memory_action_name", access=Access.WRITE)
        self._ticks = 0

    def initialise(self):
        self._ticks = 0

    def update(self):
        self.bb.memory_process_name = self.PROCESS_ID
        self.bb.memory_action_name = self.ACTION_ID

        # TODO: Ask human to pull out rack.
        # Nur für den Strukturtest: nach 2 Ticks erfolgreich.
        # Bei echter ROS-Integration durch Ergebnisprüfung ersetzen.
        self._ticks += 1
        if self._ticks < 2:
            return Status.RUNNING
        self.logger.info(f'{self.ACTION_ID} Succeeded')
        return Status.SUCCESS

    def terminate(self, new_status):
        pass


class WaitForRackConfirmation(py_trees.behaviour.Behaviour):
    """PDF state: Wait-for-Rack-Confirmation."""

    PROCESS_ID = PROCESS_ID
    ACTION_ID = 'pick.prepare.wait_for_rack_confirmation'

    def __init__(self):
        super().__init__(name='Wait for Rack Confirmation')
        self.bb = py_trees.blackboard.Client(name='pick_and_place:WaitForRackConfirmation')
        self.bb.register_key("memory_process_name", access=Access.WRITE)
        self.bb.register_key("memory_action_name", access=Access.WRITE)
        self._ticks = 0

    def initialise(self):
        self._ticks = 0

    def update(self):
        self.bb.memory_process_name = self.PROCESS_ID
        self.bb.memory_action_name = self.ACTION_ID

        # TODO: Wait for rack-pulled-out confirmation.
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
    """Build Entering Dishwasher Preparation as an ordered py_trees sequence."""
    sequence = py_trees.composites.Sequence(
        name='Entering Dishwasher Preparation',
        memory=True,
        children=[
            with_action_resume(WaitingState()),
            with_action_resume(NavigationToKitchen()),
            with_action_resume(RequestOpeningOfDishwasher()),
            with_action_resume(WaitForOpeningConfirmation()),
            with_action_resume(RequestPullingOutDishwasherRack()),
            with_action_resume(WaitForRackConfirmation()),
        ],
    )
    return with_process_resume(PROCESS_ID, sequence)

