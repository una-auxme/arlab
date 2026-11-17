"""Bildausgabe aus dem echten BT und der HFSM-Beschreibung."""
import pytest

py_trees = pytest.importorskip("py_trees")

from arlab_decision_making.visualization import (
    _statechart_definition,
    render_behavior_tree,
    render_state_machine,
)


@pytest.mark.parametrize("task", ["laundry", "pick_and_place"])
def test_behavior_tree_exports_all_formats(tmp_path, task):
    files = render_behavior_tree(task, tmp_path)
    assert len(files) == 8
    assert {f.suffix for f in files} == {".dot", ".png", ".svg", ".pdf"}
    assert all(f.is_file() and f.stat().st_size > 0 for f in files)


def test_statechart_declares_operational_parallel_regions():
    states, transitions = _statechart_definition()
    operational = next(s for s in states if isinstance(s, dict))
    assert operational["name"] == "OPERATIONAL"
    assert [r["name"] for r in operational["parallel"]] == ["HEARTBEAT", "BT"]
    assert any(t[1] == "OPERATIONAL_BT_RETRY" for t in transitions)
    assert any(t[2] == "RECOVERY" for t in transitions)


def test_state_machine_exports_all_formats(tmp_path):
    pytest.importorskip("transitions")
    files = render_state_machine(tmp_path)
    assert len(files) == 4
    assert all(f.is_file() and f.stat().st_size > 0 for f in files)
