"""Build the ordered task as a sequence of independently editable processes."""

import py_trees

from . import entering_and_localization
from . import unloading_washing_machine
from . import transporting_laundry_bin
from . import taking_out_and_folding
from . import completion


def create_tree():
    return py_trees.composites.Sequence(
        name="Laundry",
        memory=True,
        children=[
            entering_and_localization.create_process(),
            unloading_washing_machine.create_process(),
            transporting_laundry_bin.create_process(),
            taking_out_and_folding.create_process(),
            completion.create_process(),
        ],
    )
