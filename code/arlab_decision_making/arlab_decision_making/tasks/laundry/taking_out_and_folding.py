"""Taking Out and Folding the T-Shirt (PDF): editable per-action py_trees behaviours.
Each update() writes the current process/action to the shared Blackboard.
This is a non-driving prototype; every real ROS step must be implemented."""

import py_trees
from py_trees.common import Access, Status
from ...resume_conditions import with_action_resume, with_process_resume

PROCESS_ID = 'laundry.fold'


class RobotTakesOneShirt(py_trees.behaviour.Behaviour):
    """PDF state: Robot-Takes-One-Shirt."""

    PROCESS_ID = PROCESS_ID
    ACTION_ID = 'laundry.fold.robot_takes_one_shirt'

    def __init__(self):
        super().__init__(name='Robot Takes One Shirt')
        self.bb = py_trees.blackboard.Client(name='laundry:RobotTakesOneShirt')
        self.bb.register_key("memory_process_name", access=Access.WRITE)
        self.bb.register_key("memory_action_name", access=Access.WRITE)
        self._ticks = 0

    def initialise(self):
        self._ticks = 0

    def update(self):
        self.bb.memory_process_name = self.PROCESS_ID
        self.bb.memory_action_name = self.ACTION_ID

        # TODO: Pick one shirt from laundry bin and verify grasp.
        # Nur für den Strukturtest: nach 2 Ticks erfolgreich.
        # Bei echter ROS-Integration durch Ergebnisprüfung ersetzen.
        self._ticks += 1
        if self._ticks < 2:
            return Status.RUNNING
        self.logger.info(f'{self.ACTION_ID} Succeeded')
        return Status.SUCCESS

    def terminate(self, new_status):
        pass


class RobotPlacesShirtOnTable(py_trees.behaviour.Behaviour):
    """PDF state: Robot-Places-Shirt-on-Table."""

    PROCESS_ID = PROCESS_ID
    ACTION_ID = 'laundry.fold.robot_places_shirt_on_table'

    def __init__(self):
        super().__init__(name='Robot Places Shirt on Table')
        self.bb = py_trees.blackboard.Client(name='laundry:RobotPlacesShirtOnTable')
        self.bb.register_key("memory_process_name", access=Access.WRITE)
        self.bb.register_key("memory_action_name", access=Access.WRITE)
        self._ticks = 0

    def initialise(self):
        self._ticks = 0

    def update(self):
        self.bb.memory_process_name = self.PROCESS_ID
        self.bb.memory_action_name = self.ACTION_ID

        # TODO: Place shirt on table and verify placement.
        # Nur für den Strukturtest: nach 2 Ticks erfolgreich.
        # Bei echter ROS-Integration durch Ergebnisprüfung ersetzen.
        self._ticks += 1
        if self._ticks < 2:
            return Status.RUNNING
        self.logger.info(f'{self.ACTION_ID} Succeeded')
        return Status.SUCCESS

    def terminate(self, new_status):
        pass


class RequestFoldingAssistance(py_trees.behaviour.Behaviour):
    """PDF state: Request-Folding-Assistance."""

    PROCESS_ID = PROCESS_ID
    ACTION_ID = 'laundry.fold.request_folding_assistance'

    def __init__(self):
        super().__init__(name='Request Folding Assistance')
        self.bb = py_trees.blackboard.Client(name='laundry:RequestFoldingAssistance')
        self.bb.register_key("memory_process_name", access=Access.WRITE)
        self.bb.register_key("memory_action_name", access=Access.WRITE)
        self._ticks = 0

    def initialise(self):
        self._ticks = 0

    def update(self):
        self.bb.memory_process_name = self.PROCESS_ID
        self.bb.memory_action_name = self.ACTION_ID

        # TODO: Ask human for folding help.
        # Nur für den Strukturtest: nach 2 Ticks erfolgreich.
        # Bei echter ROS-Integration durch Ergebnisprüfung ersetzen.
        self._ticks += 1
        if self._ticks < 2:
            return Status.RUNNING
        self.logger.info(f'{self.ACTION_ID} Succeeded')
        return Status.SUCCESS

    def terminate(self, new_status):
        pass


class WaitForFoldingConfirmation(py_trees.behaviour.Behaviour):
    """PDF state: Wait-for-Folding-Confirmation."""

    PROCESS_ID = PROCESS_ID
    ACTION_ID = 'laundry.fold.wait_for_folding_confirmation'

    def __init__(self):
        super().__init__(name='Wait for Folding Confirmation')
        self.bb = py_trees.blackboard.Client(name='laundry:WaitForFoldingConfirmation')
        self.bb.register_key("memory_process_name", access=Access.WRITE)
        self.bb.register_key("memory_action_name", access=Access.WRITE)
        self._ticks = 0

    def initialise(self):
        self._ticks = 0

    def update(self):
        self.bb.memory_process_name = self.PROCESS_ID
        self.bb.memory_action_name = self.ACTION_ID

        # TODO: Wait for user completion confirmation; optionally repeat shirts.
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
    """Build Taking Out And Folding as an ordered py_trees sequence."""
    sequence = py_trees.composites.Sequence(
        name='Taking Out And Folding',
        memory=True,
        children=[
            with_action_resume(RobotTakesOneShirt()),
            with_action_resume(RobotPlacesShirtOnTable()),
            with_action_resume(RequestFoldingAssistance()),
            with_action_resume(WaitForFoldingConfirmation()),
        ],
    )
    return with_process_resume(PROCESS_ID, sequence)

