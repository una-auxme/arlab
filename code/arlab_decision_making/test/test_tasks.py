"""Tests der individuellen py_trees-Aktionen und des Blackboard-Wiedereinstiegs."""

import inspect

import pytest

py_trees = pytest.importorskip("py_trees")
from py_trees.common import Status

from arlab_decision_making.blackboard import BTBlackboard
from arlab_decision_making.bt_controller import BehaviorTreeController
from arlab_decision_making.resume_conditions import CheckResumeAction, CheckResumeProcess
from arlab_decision_making.tasks.laundry import tree as laundry
from arlab_decision_making.tasks.pick_and_place import tree as pick_and_place


def make_controller(task="laundry"):
    blackboard = BTBlackboard()
    controller = BehaviorTreeController(task, blackboard)
    controller.start()
    return controller, blackboard


def run_to_end(controller, limit=150):
    for _ in range(limit):
        outcome = controller.tick()
        if outcome != "RUNNING":
            return outcome
    pytest.fail("Tree did not finish inside tick limit")


def action_nodes(controller):
    return [
        node for node in controller.tree.root.iterate()
        if isinstance(node, py_trees.behaviour.Behaviour)
        and hasattr(node, "ACTION_ID")
    ]


def record_successes(controller, monkeypatch):
    """Tests beobachten Erfolge, ohne den produktiven Code zu instrumentieren."""
    completed = []
    for cls in {type(node) for node in action_nodes(controller)}:
        original = cls.update

        def update_and_record(self, _original=original):
            outcome = _original(self)
            if outcome == Status.SUCCESS:
                completed.append(self.ACTION_ID)
            return outcome

        monkeypatch.setattr(cls, "update", update_and_record)
    return completed


@pytest.mark.parametrize("task,processes,actions", [
    ("laundry", 5, 17), ("pick_and_place", 10, 33),
])
def test_all_actions_complete_after_two_ticks_each(task, processes, actions):
    controller, blackboard = make_controller(task)
    root = controller.tree.root
    assert sum(isinstance(n, CheckResumeProcess) for n in root.iterate()) == processes
    assert sum(isinstance(n, CheckResumeAction) for n in root.iterate()) == actions
    assert run_to_end(controller) == "SUCCESS"
    assert blackboard.retry_pending is False
    # memory=True: Eine Aktion, die SUCCESS meldet, kann auf demselben
    # BT-Tick den Beginn der nächsten ermöglichen. Daher 1 + Anzahl.
    assert all(n._ticks == 2 for n in action_nodes(controller))


def test_first_action_sets_blackboard_and_stays_running_on_first_tick():
    controller, blackboard = make_controller()
    assert controller.tick() == "RUNNING"
    assert blackboard.process == "laundry.enter"
    assert blackboard.action == "laundry.enter.waiting_state"
    first = next(n for n in action_nodes(controller) if n.ACTION_ID == blackboard.action)
    assert first._ticks == 1
    assert controller.tick() == "RUNNING"
    assert first._ticks == 2
    assert blackboard.action == "laundry.enter.navigation_to_laundry_room"


@pytest.mark.parametrize("task,target", [
    ("laundry", "laundry.transport.pick_up_laundry_bin"),
    ("pick_and_place", "pick.trash.grab_trash_object_on_table"),
])
def test_pause_resume_skips_finished_actions(task, target, monkeypatch):
    controller, blackboard = make_controller(task)
    successes = record_successes(controller, monkeypatch)
    for _ in range(100):
        assert controller.tick() == "RUNNING"
        if blackboard.action == target:
            break
    else:
        pytest.fail("Target action never reached")
    before = successes.copy()
    assert target not in before
    controller.pause()
    assert blackboard.retry_pending
    controller.resume()
    assert blackboard.retry_pending
    assert run_to_end(controller) == "SUCCESS"
    assert blackboard.retry_pending is False
    assert successes[:len(before)] == before
    assert successes[len(before)] == target
    assert len(successes) == len(set(successes)), "Vorherige Aktionen wurden erneut ausgeführt"


def test_resume_from_manually_set_checkpoint_skips_previous_processes(monkeypatch):
    controller, blackboard = make_controller()
    successes = record_successes(controller, monkeypatch)
    blackboard.bb.memory_process_name = "laundry.unload"
    blackboard.bb.memory_action_name = "laundry.unload.place_shirts_in_laundry_bin"
    blackboard.bb.retry_flag = True
    controller.start()
    assert run_to_end(controller) == "SUCCESS"
    assert successes[0] == "laundry.unload.place_shirts_in_laundry_bin"
    assert all(not action.startswith("laundry.enter") for action in successes)
    assert blackboard.retry_pending is False


def test_failure_resume_retries_the_failed_action_once(monkeypatch):
    controller, blackboard = make_controller()
    actions = action_nodes(controller)
    target = next(type(n) for n in actions if n.ACTION_ID == "laundry.transport.pick_up_laundry_bin")
    original = target.update
    first_failure = [True]

    def fail_only_once(self):
        outcome = original(self)  # schreibt den Checkpoint
        if first_failure[0]:
            first_failure[0] = False
            return Status.FAILURE
        return outcome

    monkeypatch.setattr(target, "update", fail_only_once)
    assert run_to_end(controller) == "FAILURE"
    assert blackboard.action == "laundry.transport.pick_up_laundry_bin"
    controller.pause()
    assert blackboard.retry_pending
    controller.resume()
    assert run_to_end(controller) == "SUCCESS"
    assert blackboard.retry_pending is False


def test_unknown_checkpoint_is_rejected():
    controller, blackboard = make_controller()
    blackboard.bb.memory_process_name = "laundry.unload"
    blackboard.bb.memory_action_name = "laundry.unload.unknown"
    blackboard.bb.retry_flag = True
    with pytest.raises(ValueError, match="unknown process/action"):
        controller.resume()
    assert blackboard.retry_pending


def test_actions_individually_editable_without_context_or_simulation_switch():
    for builder in (laundry, pick_and_place):
        root = builder.create_tree()
        for node in root.iterate():
            if not hasattr(node, "ACTION_ID"):
                continue
            assert not hasattr(node, "context")
            assert hasattr(node, "_ticks")
            assert set(inspect.signature(type(node).__init__).parameters) == {"self"}
            source = inspect.getsource(type(node).update)
            assert "self.bb.memory_process_name = self.PROCESS_ID" in source
            assert "self.bb.memory_action_name = self.ACTION_ID" in source
            assert "self._ticks += 1" in source
            assert "simulate_actions" not in source
            assert "self.context" not in source


def test_heartbeat_pause_uses_resume_checkpoint():
    from arlab_decision_making.state_machine import DecisionStateMachine, Phase, BTRegion
    from arlab_decision_making.recovery import RecoveryManager

    class Heartbeat:
        healthy = True

    hb = Heartbeat()
    controller, blackboard = make_controller()
    machine = DecisionStateMachine(
        heartbeat=hb,
        bt_controller=controller,
        recovery=RecoveryManager(log=lambda _: None),
        log=lambda _: None,
    )
    machine.monitor_tick()  # INIT -> OPERATIONAL
    machine.tick_bt()        # STARTING -> RUNNING, Tree-Aufbau
    for _ in range(6):
        machine.tick_bt()
    interrupted_at = blackboard.action
    hb.healthy = False
    machine.monitor_tick()
    assert machine.bt_region == BTRegion.PAUSED
    assert blackboard.retry_pending
    hb.healthy = True  # Recovery vor dem nächsten BT-Tick
    machine.monitor_tick()
    machine.tick_bt()
    assert machine.bt_region == BTRegion.RUNNING
    machine.tick_bt()
    assert blackboard.action == interrupted_at
    assert not blackboard.retry_pending
    for _ in range(100):
        machine.tick_bt()
        if machine.phase in (Phase.FINISHED, Phase.FAILED):
            break
    assert machine.phase == Phase.FINISHED
