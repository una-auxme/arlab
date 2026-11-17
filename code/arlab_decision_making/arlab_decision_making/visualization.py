"""Automatischer Export der aktuellen BT-Struktur und des HFSM-Diagramms.

BT: py_trees.display.dot_tree() wie im Forschungsprojekt.
HFSM: transitions.extensions.HierarchicalGraphMachine zeichnet aus einer
     deklarativen Zustands-/Transitionsbeschreibung ein Statechart.

Die HFSM-Laufzeit in state_machine.py bleibt unverändert: die Definition hier
ist die Darstellung der vorhandenen Übergänge, kein zweiter Laufzeit-Automat.
Beim Ändern der HFSM-Übergänge muss diese Definition entsprechend angepasst
werden (eine Python-Implementierung kann nicht automatisch ihre eigenen
if/else-Zweige vollständig als Zustandsdiagramm erkennen).
"""

from pathlib import Path

import py_trees

from .state_machine import Phase, HBRegion, BTRegion
from .tasks.laundry import tree as laundry_tree
from .tasks.pick_and_place import tree as pick_and_place_tree


TASK_TREES = {
    "laundry": laundry_tree.create_tree,
    "pick_and_place": pick_and_place_tree.create_tree,
}
FORMATS = ("dot", "png", "svg", "pdf")


def _write_dot_graph(graph, stem: Path) -> list[Path]:
    """Graphviz/pydot rendert die automatisch erzeugte DOT-Struktur."""
    graph.set_rankdir("TB")
    graph.set_graph_defaults(dpi="200")
    files = []
    for fmt in FORMATS:
        path = stem.with_suffix(f".{fmt}")
        graph.write(str(path), format=fmt, prog="dot")
        files.append(path)
    return files


def render_behavior_tree(task: str, output_directory: Path) -> list[Path]:
    """Baut den echten Task-BT; erzeugt Detailansicht + Prozessuebersicht."""
    if task not in TASK_TREES:
        raise ValueError(f"Unknown task: {task}")

    root = TASK_TREES[task]()
    # Detailgrafik: alle Prozesse, Bedingungen und Einzelaktionen.
    graph = py_trees.display.dot_tree(root)
    files = _write_dot_graph(graph, output_directory / f"bt_{task}")

    # Uebersicht: dieselbe Baumstruktur mit eingeklappten Prozess-Teilbaeumen.
    # Die hier neu erzeugten Instanzen sind NICHT die laufenden ROS-Actionen.
    for process in root.children:
        process.blackbox_level = py_trees.common.BlackBoxLevel.COMPONENT
    overview = py_trees.display.dot_tree(
        root, visibility_level=py_trees.common.VisibilityLevel.COMPONENT
    )
    files.extend(_write_dot_graph(overview, output_directory / f"bt_{task}_overview"))
    return files


def _statechart_definition():
    """Abbildung der bestehenden HFSM inklusive der orthogonalen BT/HB-Region.

    Die Zustandsnamen stammen aus den Enums der echten State Machine. Das
    transitions-Modell wird nur fuer die Visualisierung instanziiert.
    """
    hb = [state.value for state in HBRegion if state != HBRegion.INACTIVE]
    bt = [state.value for state in BTRegion if state != BTRegion.INACTIVE]

    states = [
        Phase.INIT.value,
        {
            "name": Phase.OPERATIONAL.value,
            "parallel": [
                {"name": "HEARTBEAT", "children": hb, "initial": HBRegion.MONITORING.value},
                {"name": "BT", "children": bt, "initial": BTRegion.STARTING.value},
            ],
        },
        Phase.RECOVERY.value,
        Phase.FINISHED.value,
        Phase.FAILED.value,
    ]
    # Die Trigger benennen die tatsaechlichen Verzweigungen in state_machine.py.
    # In diesem Prototyp sind einige Faelle (z. B. FAILED als finale Phase)
    # nur vorbereitet, nicht als aktive Laufzeittransition implementiert.
    transitions = [
        ["heartbeat_ok", "INIT", "OPERATIONAL"],
        ["start_bt", "OPERATIONAL_BT_STARTING", "OPERATIONAL_BT_RUNNING"],
        ["bt_success", "OPERATIONAL_BT_RUNNING", "OPERATIONAL_BT_SUCCEEDED"],
        ["finish", "OPERATIONAL_BT_SUCCEEDED", "FINISHED"],
        ["bt_failure", "OPERATIONAL_BT_RUNNING", "OPERATIONAL_BT_FAILED"],
        ["request_retry", "OPERATIONAL_BT_FAILED", "OPERATIONAL_BT_RETRY"],
        ["resume_retry", "OPERATIONAL_BT_RETRY", "OPERATIONAL_BT_RUNNING"],
        ["heartbeat_lost", "OPERATIONAL_HEARTBEAT_MONITORING", "OPERATIONAL_HEARTBEAT_PAUSE_BY_HEARTBEAT"],
        ["pause_bt", "OPERATIONAL_BT_RUNNING", "OPERATIONAL_BT_PAUSED"],
        ["heartbeat_restored", "OPERATIONAL_BT_PAUSED", "OPERATIONAL_BT_RUNNING"],
        ["enter_recovery", "OPERATIONAL_BT_PAUSED", "RECOVERY"],
        ["recovery_ready", "RECOVERY", "OPERATIONAL"],
    ]
    return states, transitions


def render_state_machine(output_directory: Path) -> list[Path]:
    """transitions rendert das verschachtelte Statechart in DOT/PNG/SVG/PDF."""
    try:
        from transitions.extensions import HierarchicalGraphMachine
    except ImportError as exc:
        raise RuntimeError(
            "State-Machine-Grafik benoetigt 'transitions' (und 'graphviz'). "
            "Installation: sudo apt install graphviz python3-transitions python3-graphviz"
        ) from exc

    states, transitions = _statechart_definition()
    # Kein model=self: Die echte DecisionStateMachine wird dadurch NICHT
    # veraendert. Dieses Modell dient ausschliesslich zum automatischen Rendern.
    machine = HierarchicalGraphMachine(
        states=states,
        transitions=transitions,
        initial=Phase.INIT.value,
        auto_transitions=False,
        show_auto_transitions=False,
        graph_engine="graphviz",
        title="Decision Making - State Machine",
    )
    files = []
    for fmt in FORMATS:
        path = output_directory / f"state_machine.{fmt}"
        machine.get_graph().draw(str(path), format=fmt, prog="dot")
        files.append(path)
    return files


def generate_graphs(task: str, output_directory: str | Path) -> list[Path]:
    """Beide Diagramme beim ROS-Node-Start in einen frei waehlbaren Ordner."""
    folder = Path(output_directory).expanduser().resolve()
    folder.mkdir(parents=True, exist_ok=True)
    paths = render_behavior_tree(task, folder)
    paths.extend(render_state_machine(folder))
    return paths
