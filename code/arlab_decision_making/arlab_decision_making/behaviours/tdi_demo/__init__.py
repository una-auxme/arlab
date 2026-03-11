from geometry_msgs.msg import Point, Pose, Quaternion
from py_trees.behaviour import Behaviour
from py_trees.behaviours import Failure, SetBlackboardVariable
from py_trees.composites import Selector, Sequence
from py_trees.decorators import SuccessIsRunning
from py_trees.timers import Timer
from py_trees_error_selector.error_selector import ErrorSelector

from ..common import speech, vision_snapshot
from ..manipulation import hand_close, hand_open, home, manipulator_pick, move
from .main_controller import ChoosePickable, ChoosePlacingPos


def _workspace_picture_pose():
    return move.get_tree(
        x=0.512326,
        y=-0.054071,
        z=0.539463,
        ox=0.697561,
        oy=0.701865,
        oz=0.10253,
        ow=0.1014,
    )


def _pick_sequence():
    return Sequence(
        name="Vision + Pick",
        memory=True,
        children=[
            _workspace_picture_pose(),
            vision_snapshot.get_tree(clear=True),
            ErrorSelector(
                name="Pick",
                children=[
                    Sequence(
                        name="Pick seq",
                        memory=True,
                        children=[
                            ChoosePickable(
                                "Choose pickable",
                                id_output_key="/main_controller/chosen_id",
                            ),
                            manipulator_pick.get_tree(
                                id_input_key="/main_controller/chosen_id"
                            ),
                        ],
                    ),
                    Sequence(
                        name="Pick error handler",
                        memory=True,
                        children=[
                            ## TODO: choose another fruit
                            Failure("TODO")
                        ],
                    ),
                ],
            ),
        ],
    )


def _place_sequence():
    place_approach_pose_key = "/main_controller/place_approach_pose"
    place_pose_key = "/main_controller/place_pose"
    blacklisted_positions_key = "/main_controller/blacklisted_positions"

    return Sequence(
        name="Setup place",
        memory=True,
        children=[
            SetBlackboardVariable(
                name="Setup blacklist",
                variable_name=blacklisted_positions_key,
                variable_value=[],
                overwrite=True,
            ),
            ErrorSelector(
                name="Place",
                children=[
                    Sequence(
                        name="Place seq",
                        memory=True,
                        children=[
                            move.get_tree(
                                x=0.0581504,
                                y=0.388174,
                                z=0.230927,
                                ox=-0.248759,
                                oy=0.673734,
                                oz=0.661041,
                                ow=-0.217315,
                            ),
                            vision_snapshot.get_tree(clear=False, mask_hand=True),
                            move.get_tree(
                                x=0.0548154,
                                y=0.351128,
                                z=0.681145,
                                ox=-0.257518,
                                oy=0.652194,
                                oz=0.680036,
                                ow=-0.214193,
                            ),
                            vision_snapshot.get_tree(clear=False, mask_hand=True),
                            ChoosePlacingPos(
                                name="Choose placing position",
                                place_approach_pose_key=place_approach_pose_key,
                                place_pose_key=place_pose_key,
                                blacklisted_positions_key=blacklisted_positions_key,
                                max_occupied_distance=0.15,
                                place_poses=[
                                    Pose(  # oben links
                                        position=Point(
                                            x=-0.329958, y=0.658023, z=0.580139
                                        ),
                                        orientation=Quaternion(
                                            x=-0.194432,
                                            y=0.636705,
                                            z=0.675844,
                                            w=-0.316288,
                                        ),
                                    ),
                                    Pose(  # oben rechts
                                        position=Point(
                                            x=-0.123057, y=0.769686, z=0.648342
                                        ),
                                        orientation=Quaternion(
                                            x=-0.0596376,
                                            y=0.627605,
                                            z=0.773812,
                                            w=-0.0614029,
                                        ),
                                    ),
                                    Pose(  # unten rechts
                                        position=Point(
                                            x=-0.144949, y=0.840292, z=0.165371
                                        ),
                                        orientation=Quaternion(
                                            x=-0.0747279,
                                            y=0.673969,
                                            z=0.710443,
                                            w=-0.188209,
                                        ),
                                    ),
                                    Pose(  # unten links
                                        position=Point(
                                            x=-0.313444, y=0.665618, z=0.153484
                                        ),
                                        orientation=Quaternion(
                                            x=-0.259282,
                                            y=0.636986,
                                            z=0.629809,
                                            w=-0.361058,
                                        ),
                                    ),
                                ],
                                approach_z_offset=0.08,
                            ),
                            move.get_tree_blackboard(
                                pose_input_key=place_approach_pose_key
                            ),
                            move.get_tree_blackboard(pose_input_key=place_pose_key),
                            hand_open.get_tree(),
                            move.get_tree_blackboard(
                                pose_input_key=place_approach_pose_key
                            ),
                        ],
                    ),
                    Sequence(
                        name="Place error handler",
                        memory=True,
                        children=[
                            ## TODO: choose another placing position
                        ],
                    ),
                ],
            ),
        ],
    )


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
                    SuccessIsRunning(
                        "Loop infinitely",
                        Sequence(
                            "Main loop",
                            memory=True,
                            children=[
                                hand_open.get_tree(),
                                hand_close.get_tree(),
                                Selector(
                                    name="Check hand",
                                    memory=True,
                                    children=[
                                        # TODO: check if we already carry a grabbed object
                                        _pick_sequence()
                                    ],
                                ),
                                _place_sequence(),
                            ],
                        ),
                    ),
                    speech.QueueSpeech(
                        "I reset my systems and am now available again."
                    ),
                ],
            ),
        ],
    )
