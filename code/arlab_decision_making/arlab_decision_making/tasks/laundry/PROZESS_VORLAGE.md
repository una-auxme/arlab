# Prozessvorlage – laundry

**Dieser Ordner ist bewusst manuell aufgebaut.** Jede Aktion ist eine eigene
`py_trees.behaviour.Behaviour`-Klasse direkt in der jeweiligen Prozessdatei.
Es gibt keine gemeinsame `ProcessAction`- oder `prototype_actions.py`-Klasse.

## Neue Prozessdatei erstellen

1. Bestehende Datei (z. B. `entering_and_localization.py`) als Vorlage verwenden.
2. Eine eigene `PROCESS_ID` als Namen vergeben, etwa `laundry.my_process` in laundry-Ordner.
3. Für jede Aktion eine **eigene Klasse** mit korrekten/sprechbaren `PROCESS_ID`/
   `ACTION_ID`-Konstanten definieren.
4. In **jedes** `update()` zuerst den aktuellen Checkpoint setzen:

```python
def update(self):
    self.bb.memory_process_name = self.PROCESS_ID
    self.bb.memory_action_name = self.ACTION_ID

    # TODO: Individuelle ROS-/Robotik-Logik und Erfolgsprüfung einbauen.
    # Aktuell werden nur zwei BT-Ticks simuliert.
    self._ticks += 1
    if self._ticks < 2:
        return Status.RUNNING
    return Status.SUCCESS
```

5. Pro Action-Blackboard-Client `memory_process_name` und
   `memory_action_name` mit `Access.WRITE` registrieren.
6. Die `create_process()`-Funktion als `Sequence(memory=True)` bauen:

```python
def create_process():
    sequence = py_trees.composites.Sequence(
        name="My Process",
        memory=True,
        children=[
            with_action_resume(FirstAction()),
            with_action_resume(SecondAction()),
        ],
    )
    return with_process_resume(PROCESS_ID, sequence)
```

7. Prozess in `tree.py` importieren und **in der gewünschten Reihenfolge**
   in die Hauptsequence aufnehmen. Keine weiteren zentralen Dateien nötig.



## Vollständige Python-Dateivorlage zum Kopieren

```python
"""My Process: hier später konkrete Robotik-/ROS-Logik implementieren."""
import py_trees
from py_trees.common import Access, Status
from ...resume_conditions import with_action_resume, with_process_resume

PROCESS_ID = "laundry.my_process"


class MyFirstAction(py_trees.behaviour.Behaviour):
    PROCESS_ID = PROCESS_ID
    ACTION_ID = PROCESS_ID + ".my_first_action"

    def __init__(self):
        super().__init__(name="My First Action")
        self.bb = py_trees.blackboard.Client(name="MyFirstAction")
        self.bb.register_key("memory_process_name", access=Access.WRITE)
        self.bb.register_key("memory_action_name", access=Access.WRITE)
        self._ticks = 0

    def initialise(self):
        self._ticks = 0

    def update(self):
        # Jede Aktion setzt explizit ihren eigenen Wiedereinstiegspunkt.
        self.bb.memory_process_name = self.PROCESS_ID
        self.bb.memory_action_name = self.ACTION_ID
        # TODO: Individuelle Aktion und deren Rückmeldungen auswerten.
        # Momentan: SUCCESS nach zwei Ticks (nur Strukturtest).
        self._ticks += 1
        if self._ticks < 2:
            return Status.RUNNING
        return Status.SUCCESS

    def terminate(self, new_status):
        pass


def create_process():
    sequence = py_trees.composites.Sequence(
        name="My Process", memory=True,
        children=[with_action_resume(MyFirstAction())],
    )
    return with_process_resume(PROCESS_ID, sequence)
```

