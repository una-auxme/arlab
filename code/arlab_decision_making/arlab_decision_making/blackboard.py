"""Gemeinsames Blackboard für den Wiedereinstieg im laufenden ROS-Node.

Die konkreten Aktionen schreiben memory_process_name / memory_action_name
selbst in ihrer jeweiligen update()-Methode.
"""

import py_trees
from py_trees.common import Access


class BTBlackboard:
    def __init__(self):
        self.bb = py_trees.blackboard.Client(name="DecisionMakingBlackboard")
        self.bb.register_key("memory_process_name", access=Access.WRITE)
        self.bb.register_key("memory_action_name", access=Access.WRITE)
        self.bb.register_key("retry_flag", access=Access.WRITE)

        # Nur beim Start einer NEUEN Task setzen, nicht bei BT-Wiederaufnahme.
        self.reset_for_new_task()

    @property
    def process(self):
        return self.bb.memory_process_name

    @property
    def action(self):
        return self.bb.memory_action_name

    @property
    def retry_pending(self):
        return self.bb.retry_flag

    def arm_resume(self):
        """Beim Stoppen/Unterbrechen Wiedereinstieg vorbereiten."""
        if self.process is not None and self.action is not None:
            self.bb.retry_flag = True

    def reset_for_new_task(self):
        """Checkpoint nur für eine bewusst neu gestartete Task löschen."""
        self.bb.memory_process_name = None
        self.bb.memory_action_name = None
        self.bb.retry_flag = False
