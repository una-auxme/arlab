"""Preparing the Bowl and Spoon (PDF): editable per-action py_trees behaviours.
Each update() writes the current process/action to the shared Blackboard.
This is a non-driving prototype; every real ROS step must be implemented."""

import py_trees
from py_trees.common import Access, Status
from ...resume_conditions import with_action_resume, with_process_resume

PROCESS_ID = 'pick.bowl'


class GrabSpoon(py_trees.behaviour.Behaviour):
    """PDF state: Grab-Spoon."""

    PROCESS_ID = PROCESS_ID
    ACTION_ID = 'pick.bowl.grab_spoon'

    def __init__(self):
        super().__init__(name='Grab Spoon')
        self.bb = py_trees.blackboard.Client(name='pick_and_place:GrabSpoon')
        self.bb.register_key("memory_process_name", access=Access.WRITE)
        self.bb.register_key("memory_action_name", access=Access.WRITE)
        self._ticks = 0

    def initialise(self):
        self._ticks = 0

    def update(self):
        self.bb.memory_process_name = self.PROCESS_ID
        self.bb.memory_action_name = self.ACTION_ID

        # TODO: Locate and grasp spoon.
        # Nur für den Strukturtest: nach 2 Ticks erfolgreich.
        # Bei echter ROS-Integration durch Ergebnisprüfung ersetzen.
        self._ticks += 1
        if self._ticks < 2:
            return Status.RUNNING
        self.logger.info(f'{self.ACTION_ID} Succeeded')
        return Status.SUCCESS

    def terminate(self, new_status):
        pass


class PlaceSpoonInBowl(py_trees.behaviour.Behaviour):
    """PDF state: Place-Spoon-in-Bowl."""

    PROCESS_ID = PROCESS_ID
    ACTION_ID = 'pick.bowl.place_spoon_in_bowl'

    def __init__(self):
        super().__init__(name='Place Spoon in Bowl')
        self.bb = py_trees.blackboard.Client(name='pick_and_place:PlaceSpoonInBowl')
        self.bb.register_key("memory_process_name", access=Access.WRITE)
        self.bb.register_key("memory_action_name", access=Access.WRITE)
        self._ticks = 0

    def initialise(self):
        self._ticks = 0

    def update(self):
        self.bb.memory_process_name = self.PROCESS_ID
        self.bb.memory_action_name = self.ACTION_ID

        # TODO: Insert spoon into bowl and verify.
        # Nur für den Strukturtest: nach 2 Ticks erfolgreich.
        # Bei echter ROS-Integration durch Ergebnisprüfung ersetzen.
        self._ticks += 1
        if self._ticks < 2:
            return Status.RUNNING
        self.logger.info(f'{self.ACTION_ID} Succeeded')
        return Status.SUCCESS

    def terminate(self, new_status):
        pass


class GrabBowl(py_trees.behaviour.Behaviour):
    """PDF state: Grab-Bowl."""

    PROCESS_ID = PROCESS_ID
    ACTION_ID = 'pick.bowl.grab_bowl'

    def __init__(self):
        super().__init__(name='Grab Bowl')
        self.bb = py_trees.blackboard.Client(name='pick_and_place:GrabBowl')
        self.bb.register_key("memory_process_name", access=Access.WRITE)
        self.bb.register_key("memory_action_name", access=Access.WRITE)
        self._ticks = 0

    def initialise(self):
        self._ticks = 0

    def update(self):
        self.bb.memory_process_name = self.PROCESS_ID
        self.bb.memory_action_name = self.ACTION_ID

        # TODO: Grasp bowl with spoon inside, respecting payload.
        # Nur für den Strukturtest: nach 2 Ticks erfolgreich.
        # Bei echter ROS-Integration durch Ergebnisprüfung ersetzen.
        self._ticks += 1
        if self._ticks < 2:
            return Status.RUNNING
        self.logger.info(f'{self.ACTION_ID} Succeeded')
        return Status.SUCCESS

    def terminate(self, new_status):
        pass


class PlaceBowlOnTable(py_trees.behaviour.Behaviour):
    """PDF state: Place-Bowl-on-Table."""

    PROCESS_ID = PROCESS_ID
    ACTION_ID = 'pick.bowl.place_bowl_on_table'

    def __init__(self):
        super().__init__(name='Place Bowl on Table')
        self.bb = py_trees.blackboard.Client(name='pick_and_place:PlaceBowlOnTable')
        self.bb.register_key("memory_process_name", access=Access.WRITE)
        self.bb.register_key("memory_action_name", access=Access.WRITE)
        self._ticks = 0

    def initialise(self):
        self._ticks = 0

    def update(self):
        self.bb.memory_process_name = self.PROCESS_ID
        self.bb.memory_action_name = self.ACTION_ID

        # TODO: Place bowl on table.
        # Nur für den Strukturtest: nach 2 Ticks erfolgreich.
        # Bei echter ROS-Integration durch Ergebnisprüfung ersetzen.
        self._ticks += 1
        if self._ticks < 2:
            return Status.RUNNING
        self.logger.info(f'{self.ACTION_ID} Succeeded')
        return Status.SUCCESS

    def terminate(self, new_status):
        pass


class PlaceSpoonNextToBowl(py_trees.behaviour.Behaviour):
    """PDF state: Place-Spoon-Next-to-Bowl."""

    PROCESS_ID = PROCESS_ID
    ACTION_ID = 'pick.bowl.place_spoon_next_to_bowl'

    def __init__(self):
        super().__init__(name='Place Spoon Next to Bowl')
        self.bb = py_trees.blackboard.Client(name='pick_and_place:PlaceSpoonNextToBowl')
        self.bb.register_key("memory_process_name", access=Access.WRITE)
        self.bb.register_key("memory_action_name", access=Access.WRITE)
        self._ticks = 0

    def initialise(self):
        self._ticks = 0

    def update(self):
        self.bb.memory_process_name = self.PROCESS_ID
        self.bb.memory_action_name = self.ACTION_ID

        # TODO: Remove spoon from bowl and place beside it.
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
    """Build Preparing Bowl And Spoon as an ordered py_trees sequence."""
    sequence = py_trees.composites.Sequence(
        name='Preparing Bowl And Spoon',
        memory=True,
        children=[
            with_action_resume(GrabSpoon()),
            with_action_resume(PlaceSpoonInBowl()),
            with_action_resume(GrabBowl()),
            with_action_resume(PlaceBowlOnTable()),
            with_action_resume(PlaceSpoonNextToBowl()),
        ],
    )
    return with_process_resume(PROCESS_ID, sequence)

