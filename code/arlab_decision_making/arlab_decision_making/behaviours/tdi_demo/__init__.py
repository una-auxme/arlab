from py_trees.behaviour import Behaviour
from py_trees.composites import Sequence
from py_trees.timers import Timer
from py_trees_error_selector.error_selector import ErrorSelector

from ..common import speech, vision_snapshot
from ..manipulation import hand_close, hand_open, home, manipulator_pick, move
from .main_controller import MainController


def get_tree() -> Behaviour:
    return Sequence(
        "TDI DEMO",
        memory=True,
        children=[
            Timer("HomeWait", 1.0),
            home.get_tree(),
            ErrorSelector(
                "Main error handler",
                children=[
                    Sequence(
                        "Main loop",
                        memory=True,
                        children=[
                            hand_open.get_tree(),
                            hand_close.get_tree(),
                            move.get_tree(
                                x=-0.407395,
                                y=-0.00167976,
                                z=0.606272,
                                ox=0.708559,
                                oy=-0.699248,
                                oz=-0.0733024,
                                ow=0.0601985,
                            ),
                            vision_snapshot.get_tree(clear=True),
                            MainController(
                                "Main Controller",
                                id_output_key="/main_controller/chosen_id",
                            ),
                            manipulator_pick.get_tree(
                                id_input_key="/main_controller/chosen_id"
                            ),
                        ],
                    ),
                    speech.QueueSpeech(
                        "I reset my systems and am now available again."
                    ),
                ],
            ),
        ],
    )
