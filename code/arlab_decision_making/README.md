# Arlab Decision Making – manuelle Prozess-/Aktionsstruktur

**Basis:** Das hochgeladene Decision-Making-Projekt und die Blackboard-/CheckMemory-Idee
von `fm_eberle_1.zip`. Die beiden Prozessvorlagen orientieren sich an der zuletzt
geänderten `PROZESS_VORLAGE.md`.

## Aufbau

```text
arlab_decision_making/
├── main.py                     # ROS2-Einstieg und HFSM
├── state_machine.py            # Operational / Recovery / BT-Region
├── bt_controller.py            # BT-Aufbau, Ticken, Pause, Wiederaufnahme
├── blackboard.py               # Checkpoint und Retry-Flag
├── resume_conditions.py        # CheckResumeProcess und CheckResumeAction
└── tasks/
    ├── laundry/
    │   ├── tree.py              # Prozessreihenfolge
    │   ├── PROZESS_VORLAGE.md   # Kopiervorlage
    │   └── *.py                 # 5 Prozesse, 17 eigene Action-Klassen
    └── pick_and_place/
        ├── tree.py
        ├── PROZESS_VORLAGE.md
        └── *.py                 # 10 Prozesse, 33 eigene Action-Klassen
```

Keine `prototype_actions.py`, keine gemeinsame `ProcessAction`-Klasse und
kein `context` in den Aktionen.

## Jede Action selbst anpassen

Jede Action ist eine eigene `py_trees.behaviour.Behaviour`-Klasse in ihrer
Prozessdatei und hat eigene Methoden `__init__()`, `initialise()`, `update()`
und `terminate()`.

```python
def update(self):
    self.bb.memory_process_name = self.PROCESS_ID
    self.bb.memory_action_name = self.ACTION_ID

    # Prototyp: nur Tick-Simulation, keine echte Robotik-Aktion
    self._ticks += 1
    if self._ticks < 2:
        return Status.RUNNING
    return Status.SUCCESS
```

`initialise()` setzt `_ticks` auf 0. Damit benötigt man **keinen**
`simulate_actions`-Schalter. Alle Action-Klassen führen momentan beim
zweiten eigenen Tick `SUCCESS` aus. **Dieser Erfolg ist kein Nachweis, dass
eine reale Bewegung oder Wahrnehmung funktioniert hat.** Bei der späteren
ROS-Integration muss die Tick-Simulation in der entsprechenden
`update()`-Methode durch echte Feedback-/Resultatprüfung ersetzt werden.

`terminate(new_status)` ist momentan nur `pass`; individuelle Abbruchlogik
für später eingesetzte ROS-Action-Goals muss an dieser Stelle ergänzt werden.
Der Log-Eintrag beim Verlassen von Operational stoppt **keine** reale
Roboterbewegung. Vor Wiederaufnahme sind tatsächlicher Stopp,
Umgebungszustand und eventuell schon ausgeführte Greif-/Ablagevorgänge zu
prüfen.

## Blackboard und Wiedereinstieg

| Blackboard-Key | Zweck |
|---|---|
| `memory_process_name` | Letztmalig gestarteter Prozess |
| `memory_action_name` | Letztmalig gestartete Action |
| `retry_flag` | Bei `True` an gespeicherter Stelle wieder anfangen |

- Jede einzelne Action setzt die zwei Namen selbst in ihrer `update()`.
- `CheckResumeProcess` prüft den Prozess, `CheckResumeAction` die Action.
- Bei Wiederaufnahme überspringen Selector-Knoten alle Schritte *vor* der
  gespeicherten Action. Der Treffer wird **neu gestartet**, nicht mitten
  in einem laufenden ROS-Goal fortgeführt. Danach geht der Tree normal weiter.
- `retry_flag` wird erst bei der **passenden** Action auf `False` gesetzt.
- Unbekannte Prozess-/Aktions-IDs werden beim Wiederaufbau abgelehnt.
- Der Checkpoint bleibt beim **BT-Neuaufbau innerhalb desselben ROS-Nodes**
  erhalten; ein kompletter Node-Neustart löscht ihn.
- Für wiederholte Objekte oder Schleifen ist später zusätzlich die
  jeweilige Iteration / Objekt-ID nötig.

## Automatische Graphen beim Start generieren

Wie im BA-Projekt wird der echte Task-Baum mit
`py_trees.display.dot_tree(root)` aus der bereits vorhandenen
`tasks/<task>/tree.py`-Struktur erzeugt. Die State Machine wird ueber die
`HierarchicalGraphMachine`-Diagrammfunktion von **transitions** dargestellt
(verschachteltes `OPERATIONAL` mit den zwei vorhandenen Regionen HEARTBEAT/BT).
Es werden **keine Bilder manuell gezeichnet**.

**Achtung:** Die State Machine in `state_machine.py` ist weiterhin in Python
implementiert und nicht selbst eine `transitions`-State-Machine. Ihre
Diagrammbeschreibung befindet sich deshalb in
`visualization.py` (`_statechart_definition`). Nach Aenderungen an den
Laufzeituebergaengen muss sie ebenfalls angepasst werden. Die Graphik ist
kein Nachweis, dass ein Zustandsuebergang ausgefuehrt oder getestet wurde.

### Einmalig installieren (Ubuntu 24.04 / ROS 2 Jazzy)

```bash
sudo apt update
sudo apt install graphviz python3-pydot python3-graphviz python3-transitions
```

Die Python-Bibliothek `py_trees` ist bereits Abhaengigkeit des ROS-Pakets.
Bei Python-Virtual-Environments muessen die angegebenen Bibliotheken in
derselben Python-Umgebung wie ROS installiert sein.

### Mit ROS 2 starten und automatisch Bilder schreiben

```bash
cd /workspace
source /opt/ros/jazzy/setup.bash
colcon build --packages-select arlab_decision_making --symlink-install
source install/setup.bash

# Laundry mit Graphenerzeugung
ros2 run arlab_decision_making decision_maker --ros-args \
  -p task:=laundry \
  -p generate_graphs:=true \
  -p graph_output_dir:=/workspace/graphs

# Alternativ: Pick and Place
ros2 run arlab_decision_making decision_maker --ros-args \
  -p task:=pick_and_place \
  -p generate_graphs:=true \
  -p graph_output_dir:=/workspace/graphs
```

Ohne `graph_output_dir` werden die Dateien in
`~/arlab_decision_making_graphs` gespeichert. Bei jedem Start mit
`generate_graphs:=true` werden die Graphen neu erzeugt bzw. ueberschrieben.
Standardmaessig ist `generate_graphs` **false**.

Ausgabe:

```text
/workspace/graphs/
  bt_laundry.dot
  bt_laundry.png
  bt_laundry.svg
  bt_laundry.pdf
  bt_laundry_overview.dot
  bt_laundry_overview.png
  bt_laundry_overview.svg
  bt_laundry_overview.pdf
  state_machine.dot
  state_machine.png
  state_machine.svg
  state_machine.pdf
```

Bei `task:=pick_and_place` heisst die erste Dateigruppe
`bt_pick_and_place.*`. Beide Formate sind **automatisch aus py_trees bzw.
transitions** erzeugt. Bei fehlenden Abhaengigkeiten wird ein ROS-Fehler
protokolliert, der Node laeuft unabhaengig davon weiter.

Bilder im Terminal pruefen:

```bash
ls -lh /workspace/graphs
```

## Start

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select arlab_decision_making --symlink-install
source install/setup.bash
ros2 run arlab_decision_making decision_maker --ros-args -p task:=laundry
```

Die ROS-Timerfrequenz `bt_tick_rate_hz` steuert, wie schnell die
simulierten Aktionen abgearbeitet werden. In `~/state` (am
`decision_maker`-Node) stehen `current_process`, `current_action` und
`retry_pending` im Status-JSON.

Ohne ROS-Node mit installiertem `py_trees`:

```bash
python -m arlab_decision_making.demo --task laundry
python -m arlab_decision_making.demo --task pick_and_place --heartbeat-pause
```

## Tests und Grenzen

```bash
python -m pytest -q test/test_tasks.py
```

Die Prototypen enthalten ausgewählte PDF-Aktionen, aber noch nicht alle
Zustände und keine vollständige Schleifenlogik. Heartbeat-/Bool-Themen und
Recovery-Bestätigungen sind **nicht** mit sicherheitsgerichteten
Hardwarefunktionen gleichzusetzen. Die State Machine bleibt eine
übergeordnete Ablaufsteuerung; eine physische Freigabe für den Wiederstart
bedarf gesonderter Validierung.
