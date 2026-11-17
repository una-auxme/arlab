"""Unloading the Washing Machine (PDF): editable per-action py_trees behaviours.
Each update() writes the current process/action to the shared Blackboard.
This is a non-driving prototype; every real ROS step must be implemented."""

import py_trees
from py_trees.common import Access, Status
from ...resume_conditions import with_action_resume, with_process_resume

PROCESS_ID = 'laundry.unload'


class RequestOpeningOfWashingMachine(py_trees.behaviour.Behaviour):
    """PDF state: Request-Opening-of-Washing-Machine."""

    PROCESS_ID = PROCESS_ID
    ACTION_ID = 'laundry.unload.request_opening_of_washing_machine'

    def __init__(self):
        super().__init__(name='Request Opening of Washing Machine')
        self.bb = py_trees.blackboard.Client(name='laundry:RequestOpeningOfWashingMachine')
        self.bb.register_key("memory_process_name", access=Access.WRITE)
        self.bb.register_key("memory_action_name", access=Access.WRITE)
        self._ticks = 0

    def initialise(self):
        self._ticks = 0

    def update(self):
        self.bb.memory_process_name = self.PROCESS_ID
        self.bb.memory_action_name = self.ACTION_ID

        # TODO: Request opening from human via speech/HMI and wait for delivery acknowledgement.
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
    ACTION_ID = 'laundry.unload.wait_for_opening_confirmation'

    def __init__(self):
        super().__init__(name='Wait for Opening Confirmation')
        self.bb = py_trees.blackboard.Client(name='laundry:WaitForOpeningConfirmation')
        self.bb.register_key("memory_process_name", access=Access.WRITE)
        self.bb.register_key("memory_action_name", access=Access.WRITE)
        self._ticks = 0

    def initialise(self):
        self._ticks = 0

    def update(self):
        self.bb.memory_process_name = self.PROCESS_ID
        self.bb.memory_action_name = self.ACTION_ID

        # TODO: Subscribe to explicit user/scene confirmation.
        # Nur für den Strukturtest: nach 2 Ticks erfolgreich.
        # Bei echter ROS-Integration durch Ergebnisprüfung ersetzen.
        self._ticks += 1
        if self._ticks < 2:
            return Status.RUNNING
        self.logger.info(f'{self.ACTION_ID} Succeeded')
        return Status.SUCCESS

    def terminate(self, new_status):
        pass


class GrabAtLeastOneTshirt(py_trees.behaviour.Behaviour):
    """PDF state: Grab-at-Least-One-Tshirt."""

    PROCESS_ID = PROCESS_ID
    ACTION_ID = 'laundry.unload.grab_at_least_one_tshirt'

    def __init__(self):
        super().__init__(name='Grab At Least One T-Shirt')
        self.bb = py_trees.blackboard.Client(name='laundry:GrabAtLeastOneTshirt')
        self.bb.register_key("memory_process_name", access=Access.WRITE)
        self.bb.register_key("memory_action_name", access=Access.WRITE)
        self._ticks = 0

    def initialise(self):
        self._ticks = 0

    def update(self):
        self.bb.memory_process_name = self.PROCESS_ID
        self.bb.memory_action_name = self.ACTION_ID

        # TODO: Locate first, execute a manipulation goal and verify the grasp.
        # Nur für den Strukturtest: nach 2 Ticks erfolgreich.
        # Bei echter ROS-Integration durch Ergebnisprüfung ersetzen.
        self._ticks += 1
        if self._ticks < 2:
            return Status.RUNNING
        self.logger.info(f'{self.ACTION_ID} Succeeded')
        return Status.SUCCESS

    def terminate(self, new_status):
        pass


class PlaceShirtsInLaundryBin(py_trees.behaviour.Behaviour):
    """PDF state: Place-Shirts-in-Laundry-Bin."""

    PROCESS_ID = PROCESS_ID
    ACTION_ID = 'laundry.unload.place_shirts_in_laundry_bin'

    def __init__(self):
        super().__init__(name='Place Shirts in Laundry Bin')
        self.bb = py_trees.blackboard.Client(name='laundry:PlaceShirtsInLaundryBin')
        self.bb.register_key("memory_process_name", access=Access.WRITE)
        self.bb.register_key("memory_action_name", access=Access.WRITE)
        self._ticks = 0

    def initialise(self):
        self._ticks = 0

    def update(self):
        self.bb.memory_process_name = self.PROCESS_ID
        self.bb.memory_action_name = self.ACTION_ID

        # TODO: Place into bin and verify that the object was released.
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
    """Build Unloading Washing Machine as an ordered py_trees sequence."""
    sequence = py_trees.composites.Sequence(
        name='Unloading Washing Machine',
        memory=True,
        children=[
            with_action_resume(RequestOpeningOfWashingMachine()),
            with_action_resume(WaitForOpeningConfirmation()),
            with_action_resume(GrabAtLeastOneTshirt()),
            with_action_resume(PlaceShirtsInLaundryBin()),
        ],
    )
    return with_process_resume(PROCESS_ID, sequence)

