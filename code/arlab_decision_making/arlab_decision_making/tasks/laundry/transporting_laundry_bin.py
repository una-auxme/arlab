"""Transporting the Laundry Bin (PDF): editable per-action py_trees behaviours.
Each update() writes the current process/action to the shared Blackboard.
This is a non-driving prototype; every real ROS step must be implemented."""

import py_trees
from py_trees.common import Access, Status
from ...resume_conditions import with_action_resume, with_process_resume

PROCESS_ID = 'laundry.transport'


class RequestLaundryBinTransport(py_trees.behaviour.Behaviour):
    """PDF state: Request-Laundry-Bin-Transport."""

    PROCESS_ID = PROCESS_ID
    ACTION_ID = 'laundry.transport.request_laundry_bin_transport'

    def __init__(self):
        super().__init__(name='Request Laundry Bin Transport')
        self.bb = py_trees.blackboard.Client(name='laundry:RequestLaundryBinTransport')
        self.bb.register_key("memory_process_name", access=Access.WRITE)
        self.bb.register_key("memory_action_name", access=Access.WRITE)
        self._ticks = 0

    def initialise(self):
        self._ticks = 0

    def update(self):
        self.bb.memory_process_name = self.PROCESS_ID
        self.bb.memory_action_name = self.ACTION_ID

        # TODO: Ask the user for permission to transport the bin.
        # Nur für den Strukturtest: nach 2 Ticks erfolgreich.
        # Bei echter ROS-Integration durch Ergebnisprüfung ersetzen.
        self._ticks += 1
        if self._ticks < 2:
            return Status.RUNNING
        self.logger.info(f'{self.ACTION_ID} Succeeded')
        return Status.SUCCESS

    def terminate(self, new_status):
        pass


class WaitForTransportConfirmation(py_trees.behaviour.Behaviour):
    """PDF state: Wait-for-Transport-Confirmation."""

    PROCESS_ID = PROCESS_ID
    ACTION_ID = 'laundry.transport.wait_for_transport_confirmation'

    def __init__(self):
        super().__init__(name='Wait for Transport Confirmation')
        self.bb = py_trees.blackboard.Client(name='laundry:WaitForTransportConfirmation')
        self.bb.register_key("memory_process_name", access=Access.WRITE)
        self.bb.register_key("memory_action_name", access=Access.WRITE)
        self._ticks = 0

    def initialise(self):
        self._ticks = 0

    def update(self):
        self.bb.memory_process_name = self.PROCESS_ID
        self.bb.memory_action_name = self.ACTION_ID

        # TODO: Wait until permission was positively confirmed.
        # Nur für den Strukturtest: nach 2 Ticks erfolgreich.
        # Bei echter ROS-Integration durch Ergebnisprüfung ersetzen.
        self._ticks += 1
        if self._ticks < 2:
            return Status.RUNNING
        self.logger.info(f'{self.ACTION_ID} Succeeded')
        return Status.SUCCESS

    def terminate(self, new_status):
        pass


class PickUpLaundryBin(py_trees.behaviour.Behaviour):
    """PDF state: Pick-Up-Laundry-Bin."""

    PROCESS_ID = PROCESS_ID
    ACTION_ID = 'laundry.transport.pick_up_laundry_bin'

    def __init__(self):
        super().__init__(name='Pick Up Laundry Bin')
        self.bb = py_trees.blackboard.Client(name='laundry:PickUpLaundryBin')
        self.bb.register_key("memory_process_name", access=Access.WRITE)
        self.bb.register_key("memory_action_name", access=Access.WRITE)
        self._ticks = 0

    def initialise(self):
        self._ticks = 0

    def update(self):
        self.bb.memory_process_name = self.PROCESS_ID
        self.bb.memory_action_name = self.ACTION_ID

        # TODO: Send grasp/carry manipulation goal and validate load.
        # Nur für den Strukturtest: nach 2 Ticks erfolgreich.
        # Bei echter ROS-Integration durch Ergebnisprüfung ersetzen.
        self._ticks += 1
        if self._ticks < 2:
            return Status.RUNNING
        self.logger.info(f'{self.ACTION_ID} Succeeded')
        return Status.SUCCESS

    def terminate(self, new_status):
        pass


class NavigationToTable(py_trees.behaviour.Behaviour):
    """PDF state: Navigation-to-Table."""

    PROCESS_ID = PROCESS_ID
    ACTION_ID = 'laundry.transport.navigation_to_table'

    def __init__(self):
        super().__init__(name='Navigation to Table')
        self.bb = py_trees.blackboard.Client(name='laundry:NavigationToTable')
        self.bb.register_key("memory_process_name", access=Access.WRITE)
        self.bb.register_key("memory_action_name", access=Access.WRITE)
        self._ticks = 0

    def initialise(self):
        self._ticks = 0

    def update(self):
        self.bb.memory_process_name = self.PROCESS_ID
        self.bb.memory_action_name = self.ACTION_ID

        # TODO: Drive to table with Nav2, monitoring basket constraints.
        # Nur für den Strukturtest: nach 2 Ticks erfolgreich.
        # Bei echter ROS-Integration durch Ergebnisprüfung ersetzen.
        self._ticks += 1
        if self._ticks < 2:
            return Status.RUNNING
        self.logger.info(f'{self.ACTION_ID} Succeeded')
        return Status.SUCCESS

    def terminate(self, new_status):
        pass


class PlaceLaundryBin(py_trees.behaviour.Behaviour):
    """PDF state: Place-Laundry-Bin."""

    PROCESS_ID = PROCESS_ID
    ACTION_ID = 'laundry.transport.place_laundry_bin'

    def __init__(self):
        super().__init__(name='Place Laundry Bin')
        self.bb = py_trees.blackboard.Client(name='laundry:PlaceLaundryBin')
        self.bb.register_key("memory_process_name", access=Access.WRITE)
        self.bb.register_key("memory_action_name", access=Access.WRITE)
        self._ticks = 0

    def initialise(self):
        self._ticks = 0

    def update(self):
        self.bb.memory_process_name = self.PROCESS_ID
        self.bb.memory_action_name = self.ACTION_ID

        # TODO: Place and release basket next to table with pose confirmation.
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
    """Build Transporting Laundry Bin as an ordered py_trees sequence."""
    sequence = py_trees.composites.Sequence(
        name='Transporting Laundry Bin',
        memory=True,
        children=[
            with_action_resume(RequestLaundryBinTransport()),
            with_action_resume(WaitForTransportConfirmation()),
            with_action_resume(PickUpLaundryBin()),
            with_action_resume(NavigationToTable()),
            with_action_resume(PlaceLaundryBin()),
        ],
    )
    return with_process_resume(PROCESS_ID, sequence)

