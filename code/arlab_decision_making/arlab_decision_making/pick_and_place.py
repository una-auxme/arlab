"""Pick and Place: high-level process actions from the uploaded PDF.

The PDF's Pick and Place task is dishwasher/kitchen, not a generic pick task.
"""

import py_trees

from .prototype_actions import ProcessAction, TaskContext


def create_tree(context: TaskContext) -> py_trees.behaviour.Behaviour:
    """Build one process-level step per PDF section, not all 65 states."""
    # Each ProcessAction is a placeholder for a future orchestrating ROS node.
    steps = [
        # TODO: Door, kitchen navigation, dishwasher locate/open/rack request.
        ("pick.prepare", "Enter Kitchen and Prepare Dishwasher"),
        # TODO: Table/plate locate, navigate, pick plate, dishwasher place.
        ("pick.plate", "Place Plate in Dishwasher"),
        # TODO: Table/cutlery locate, pick cutlery, dishwasher place.
        ("pick.cutlery", "Place Cutlery in Dishwasher"),
        # TODO: Identify category and suitable shelf, pick and sort object.
        ("pick.shelf", "Sort Object into Shelf"),
        # TODO: Find trash bin, pick trash from table, dispose.
        ("pick.trash", "Dispose of Trash"),
        # TODO: Extra surface object sorting, loop remaining objects.
        # TODO: Object detection may use Retry(num_failures=3) INSIDE the node,
        #       not a retry of a potentially non-idempotent entire manipulation.
        ("pick.extra", "Sort Objects from Extra Surface"),
        # TODO: Locate bowl/spoon, put spoon in bowl, place both on table,
        #       then move spoon next to bowl.
        ("pick.bowl", "Prepare Bowl and Spoon"),
        # TODO: Fetch cereal from shelf to table.
        ("pick.cereal", "Fetch Cereal"),
        # TODO: Fetch milk from shelf to table.
        ("pick.milk", "Fetch Milk"),
        # TODO: Announce task completion.
        ("pick.complete", "Announce Completion"),
    ]
    return py_trees.composites.Sequence(
        name="Pick and Place",
        memory=True,
        children=[ProcessAction(key, name, context) for key, name in steps],
    )
