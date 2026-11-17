import py_trees
from py_trees.common import Access, Status
from ...resume_conditions import with_action_resume, with_process_resume


class NavigationToLaundryRoom(py_trees.behaviour.Behaviour):
    """PDF state: Navigation-to-Laundry-Room."""

    def __init__(self, process_id, action_id, entity_type):
        super().__init__(name='Navigation to Laundry Room')
        self.bb = py_trees.blackboard.Client(name='laundry:NavigationToLaundryRoom')
        self.bb.register_key("memory_process_name", access=Access.WRITE)
        self.bb.register_key("memory_action_name", access=Access.WRITE)
        self.ACTION_ID = action_id
        self.PROCESS_ID = process_id
        self.entity_ID
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
