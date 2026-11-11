"""Laundry: high-level process actions from States for Laundry(1).pdf.

The detailed state transitions are deliberately delegated to future nodes.
"""

import py_trees

from .prototype_actions import ProcessAction, TaskContext


def create_tree(context: TaskContext) -> py_trees.behaviour.Behaviour:
    """Build a short, sequential Laundry task with process checkpoints."""
    return py_trees.composites.Sequence(
        name="Laundry",
        memory=True,
        children=[
            # TODO: Door-open signal, navigation, washing machine/bin detection.
            ProcessAction("laundry.enter", "Enter and Localize", context),
            # TODO: Human opening request/confirmation, find and pick T-shirt,
            #       place at least one T-shirt into the laundry bin.
            ProcessAction("laundry.unload", "Unload Washing Machine", context),
            # TODO: Ask permission, navigate, transport and place laundry bin.
            ProcessAction("laundry.transport", "Transport Laundry Bin", context),
            # TODO: Take out shirt, place on table, ask for folding assistance.
            # TODO: Loop while more T-shirts remain (not yet implemented).
            ProcessAction("laundry.fold", "Take Out and Fold Shirt", context),
            # TODO: Announcement / task completion notification interface.
            ProcessAction("laundry.complete", "Announce Completion", context),
        ],
    )
