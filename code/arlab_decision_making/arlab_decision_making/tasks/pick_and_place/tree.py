"""Build the ordered task as a sequence of independently editable processes."""

import py_trees

from . import entering_dishwasher_preparation
from . import placing_plate
from . import placing_cutlery
from . import sorting_into_shelf
from . import disposing_of_trash
from . import sorting_extra_surface
from . import preparing_bowl_and_spoon
from . import fetching_cereal
from . import fetching_milk
from . import completion


def create_tree():
    return py_trees.composites.Sequence(
        name="Pick and Place",
        memory=True,
        children=[
            entering_dishwasher_preparation.create_process(),
            placing_plate.create_process(),
            placing_cutlery.create_process(),
            sorting_into_shelf.create_process(),
            disposing_of_trash.create_process(),
            sorting_extra_surface.create_process(),
            preparing_bowl_and_spoon.create_process(),
            fetching_cereal.create_process(),
            fetching_milk.create_process(),
            completion.create_process(),
        ],
    )
