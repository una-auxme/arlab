"""Ohne ROS-Hardware: die Tasks per Tick simulieren.

Beispiele:
    python -m arlab_decision_making.demo --task laundry
    python -m arlab_decision_making.demo --task pick_and_place --heartbeat-pause
"""

import argparse

from .blackboard import BTBlackboard
from .bt_controller import BehaviorTreeController
from .recovery import RecoveryManager
from .state_machine import DecisionStateMachine, Phase


class DemoHeartbeat:
    def __init__(self):
        self.healthy = True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--task", choices=["laundry", "pick_and_place"], default="laundry")
    parser.add_argument("--heartbeat-pause", action="store_true")
    args = parser.parse_args()

    heartbeat = DemoHeartbeat()
    blackboard = BTBlackboard()
    controller = BehaviorTreeController(args.task, blackboard)
    machine = DecisionStateMachine(heartbeat, controller, RecoveryManager())

    for step in range(180):
        heartbeat.healthy = not (args.heartbeat_pause and step == 12)
        machine.monitor_tick()
        if machine.phase == Phase.RECOVERY:
            machine.confirm_environment_checked()
            machine.authorize_restart()
        machine.tick_bt()
        print(step, machine.status())
        if machine.phase in (Phase.FINISHED, Phase.FAILED):
            break


if __name__ == "__main__":
    main()
