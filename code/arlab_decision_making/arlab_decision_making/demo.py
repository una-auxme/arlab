"""Pure-Python simulation demonstrating monitor fault + checkpointed recovery.

Requires py_trees but NOT ROS 2. Run: python3 -m arlab_decision_making.demo
"""

import argparse

from .bt_controller import BehaviorTreeController
from .estop_monitor import EstopMonitor
from .heartbeat_monitor import HeartbeatMonitor
from .prototype_actions import TaskContext
from .recovery import RecoveryManager
from .state_machine import DecisionStateMachine, Phase


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--task", choices=["laundry", "pick_and_place"], default="laundry")
    parser.add_argument("--fault", choices=["heartbeat", "estop", "bt"], default="heartbeat")
    arguments = parser.parse_args()

    heartbeat = HeartbeatMonitor([1], timeout_seconds=2.0)
    estop = EstopMonitor(timeout_seconds=2.0)
    context = TaskContext(
        simulate_actions=True,
        fail_step_once=("laundry.transport" if arguments.fault == "bt" and arguments.task == "laundry" else
                        "pick.trash" if arguments.fault == "bt" else ""),
    )
    controller = BehaviorTreeController(arguments.task, context)
    machine = DecisionStateMachine(heartbeat, estop, controller, RecoveryManager())
    heartbeat.receive(1, 0)
    estop.receive(False)
    machine.monitor_tick()

    for step in range(80):
        # Keep healthy sensors alive while the demo runs.
        if not (arguments.fault == "heartbeat" and step == 5):
            machine.heartbeat_event(1, 0)
        machine.estop_event(arguments.fault == "estop" and step == 5)
        if step == 5 and arguments.fault == "heartbeat":
            # An explicit unhealthy status produces an immediate interruption.
            machine.heartbeat_event(1, 9)
        if step > 5:
            machine.heartbeat_event(1, 0)
        if machine.phase == Phase.RECOVERY:
            machine.confirm_environment_checked()
            machine.authorize_restart()
        machine.monitor_tick()
        machine.tick_bt()
        print(step, machine.status()["phase"], machine.status()["bt_region"])
        if machine.phase in (Phase.FINISHED, Phase.FAILED):
            break
    print("Finished:", machine.status(), "completed:", sorted(context.completed))


if __name__ == "__main__":
    main()
