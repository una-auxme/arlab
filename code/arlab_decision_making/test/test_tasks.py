"""Integration tests requiring py_trees (skip if not installed)."""

import pytest

py_trees = pytest.importorskip("py_trees")
from arlab_decision_making.bt_controller import BehaviorTreeController
from arlab_decision_making.prototype_actions import TaskContext


@pytest.mark.parametrize("task,expected_count", [("laundry", 5), ("pick_and_place", 10)])
def test_tasks_complete_in_simulation(task, expected_count):
    context = TaskContext(simulate_actions=True, log=lambda x: None)
    controller = BehaviorTreeController(task, context)
    controller.start()
    for i in range(50):
        status = controller.tick()
        if status == "SUCCESS":
            break
    assert status == "SUCCESS"
    assert len(context.completed) == expected_count


def test_checkpoint_resume_skips_completed_actions():
    context = TaskContext(simulate_actions=True, log=lambda x: None)
    controller = BehaviorTreeController("laundry", context)
    controller.start()
    controller.tick()  # enter step 1 RUNNING
    controller.tick()  # step 1 finishes, next step RUNNING
    assert "laundry.enter" in context.completed
    controller.pause()
    controller.resume()
    for i in range(20):
        status = controller.tick()
        if status == "SUCCESS":
            break
    assert status == "SUCCESS"
    assert "laundry.complete" in context.completed


def test_one_mock_failed_action_can_be_resumed():
    context = TaskContext(
        simulate_actions=True, fail_step_once="laundry.transport", log=lambda x: None,
    )
    controller = BehaviorTreeController("laundry", context)
    controller.start()
    for _ in range(30):
        status = controller.tick()
        if status == "FAILURE":
            break
    assert status == "FAILURE"
    assert "laundry.unload" in context.completed
    assert "laundry.transport" not in context.completed
    controller.pause()
    controller.resume()
    for _ in range(30):
        status = controller.tick()
        if status == "SUCCESS":
            break
    assert status == "SUCCESS"
