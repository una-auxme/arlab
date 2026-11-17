"""BT-Steuerung: Tick, Pause und Wiederaufbau am Blackboard-Checkpoint."""

import py_trees

from .blackboard import BTBlackboard
from .resume_conditions import CheckResumeAction
from .tasks.laundry import tree as laundry
from .tasks.pick_and_place import tree as pick_and_place


class BehaviorTreeController:
    BUILDERS = {
        "laundry": laundry.create_tree,
        "pick_and_place": pick_and_place.create_tree,
    }

    def __init__(self, task: str, blackboard: BTBlackboard):
        if task not in self.BUILDERS:
            raise ValueError(f"Unknown task '{task}'; use laundry/pick_and_place")
        self.task = task
        self.blackboard = blackboard
        self.tree = None
        self.is_paused = False

    def start(self):
        """Neuen BT aufbauen, ohne das vorhandene Blackboard zu löschen."""
        root = self.BUILDERS[self.task]()
        if self.blackboard.retry_pending:
            target = (self.blackboard.process, self.blackboard.action)
            known_actions = {
                (node.process_id, node.action_id)
                for node in root.iterate()
                if isinstance(node, CheckResumeAction)
            }
            if target not in known_actions:
                raise ValueError(
                    f"Cannot resume unknown process/action {target}; "
                    "checkpoint retained instead of skipping the entire task"
                )
        self.tree = py_trees.trees.BehaviourTree(root)
        self.is_paused = False

    def tick(self) -> str:
        if self.tree is None:
            raise RuntimeError("Call start before tick")
        if self.is_paused:
            raise RuntimeError("Cannot tick paused tree")
        self.tree.tick()
        return self.tree.root.status.name

    def pause(self):
        """BT stoppen und gespeicherte Aktion für Neustart markieren."""
        if self.tree is not None and not self.is_paused:
            if self.tree.root.status == py_trees.common.Status.SUCCESS:
                return
            self.blackboard.arm_resume()
            self.tree.root.stop(py_trees.common.Status.INVALID)
            self.is_paused = True
            # Wichtig: INVALID stoppt den py_trees-Knoten, aber kein reales
            # laufendes ROS-Action-Goal automatisch.

#    def restart(self) -> None:
#        """Restart incomplete process; completed process checkpoints persist."""
#        self.is_paused = False

    def resume(self) -> None:
        """Restart incomplete process; completed process checkpoints persist."""
    def resume(self):
        self.start()

    def shutdown(self) -> None:
        if self.tree is not None:
            self.pause()
            self.tree.shutdown()
