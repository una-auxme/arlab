"""Decision Maker Node orchestrating Behavior Tree for Task Execution
This node builds and runs a behavior tree to manage high-level decision-making
and task execution for the robot. It supports the safety check and different
tasks that can be selected via parameters.

Maintainers:
    Peter Viechter <peter.viechter@uni-augsburg.de>
    Daniel Gabler <daniel.gabler@uni-augsburg.de>
"""

import py_trees_ros
import rclpy
import rclpy.callback_groups
import rclpy.executors
from arlab_common.exceptions import emsg_with_trace
from py_trees.behaviour import Behaviour
from py_trees.common import ParallelPolicy
from py_trees.composites import Parallel, Sequence
from rclpy.node import Node

from .behaviours import task1, task2, tdi_demo
from .behaviours.common.speech import SpeechOutput
from .behaviours.test_trees import (
    test_manipulation_dr6_video,
    test_manipulation_home,
    test_manipulation_home_move_seq,
    test_manipulation_move,
    test_manipulation_pick,
    test_manipulation_video_sequence,
    test_view,
)


def get_tree(task: str) -> Behaviour:
    """Construct the behavior tree based on the selected task.

    Args:
        task (str): Name of the requested task.
            Supported tasks:
            - "task1": Execute Task 1 behavior tree.
            - "task2": Execute Task 2 behavior tree.
            - "test_manipulation_home": Execute manipulation home test tree.
            - "test_manipulation_move": Execute manipulation move test tree.
            - "test_manipulation_home_move_seq": Execute manipulation test tree.
            - "test_grab_seq": Execute test_manipulation_video_seq test tree.

    Raises:
        ValueError: If an unknown task name is provided.

    Returns:
        Behaviour: Behavior tree root node.
    """
    task = task.lower()
    if task == "task1":
        chosen_task = task1
    elif task == "task2":
        chosen_task = task2
    elif task == "test_manipulation_home":
        chosen_task = test_manipulation_home
    elif task == "test_manipulation_move":
        chosen_task = test_manipulation_move
    elif task == "test_manipulation_home_move_seq":
        chosen_task = test_manipulation_home_move_seq
    elif task == "test_grab_seq":
        chosen_task = test_manipulation_video_sequence
    elif task == "test_grab_dr6":
        chosen_task = test_manipulation_dr6_video
    elif task == "test_pick":
        chosen_task = test_manipulation_pick
    elif task == "test_view":
        chosen_task = test_view
    elif task == "tdi_demo":
        chosen_task = tdi_demo
    else:
        raise ValueError(f"Unknown task: {task}")

    return Parallel(
        name="DecisionMakerParallel",
        policy=ParallelPolicy.SuccessOnOne(),
        children=[
            Sequence(
                name="DecisionMaker",
                memory=False,
                children=[
                    # check_safety.get_tree(),
                    chosen_task.get_tree(),
                ],
            ),
            SpeechOutput(),
        ],
    )


class DecisionMaker(Node):
    """Decision Maker Node orchestrating Behavior Tree for Task Execution.
    This node builds and runs a behavior tree to manage high-level decision-making
    and task execution for the robot. It supports the safety check and different
    tasks that can be selected via parameters.

    Attributes:
        update_rate (float): Frequency (Hz) at which the behavior tree is ticked.
        task (str): Name of the selected task to execute.
        behavior_tree (py_trees_ros.trees.BehaviourTree): The constructed behavior tree.
    """

    def __init__(self):
        """Initialize the Decision Maker node and behavior tree."""
        super().__init__(type(self).__name__)
        self.get_logger().info(f"{type(self).__name__} node initializing...")

        # Parameters
        self.update_rate = (
            self.declare_parameter(
                "update_rate",
                5.0,
            )
            .get_parameter_value()
            .double_value
        )

        # Parameters
        self.task = (
            self.declare_parameter(
                "task",
                "task1",
            )
            .get_parameter_value()
            .string_value
        )

        root = get_tree(self.task)
        self.behavior_tree = py_trees_ros.trees.BehaviourTree(root)

        self.behavior_tree.setup(node=self, timeout=15.0)

        self.timer_callback_group = (
            rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        )
        self.loop_timer = self.create_timer(
            1.0 / self.update_rate,
            self.tick_tree_handler,
            callback_group=self.timer_callback_group,
        )
        self.get_logger().info(f"{type(self).__name__} node initialized.")

    def tick_tree_handler(self):
        """Timer callback to tick the behavior tree.
        Handles exceptions during the tick process and logs fatal errors.
        """
        try:
            self.tick_tree()
        except Exception as e:
            self.get_logger().fatal(emsg_with_trace(e), throttle_duration_sec=2)

    def tick_tree(self):
        """Tick the behavior tree once."""
        self.behavior_tree.tick()

    def shutdown(self):
        """Shutdown the behavior tree and clean up resources."""
        self.behavior_tree.interrupt()
        self.behavior_tree.shutdown()


def main(args=None):
    # from arlab_common.debugging import start_debugger

    # start_debugger(wait_for_client=False)

    rclpy.init(args=args)

    # Executor with exactly two threads
    # - One for the behavior tree tick timer
    # - One for internal ros callback
    # Note that this thread split is not enforced, but the two threads
    #   are necessary to not deadlock the node when issuing service calls
    # IMPORTANT: services must only be called
    #   from inside the timer callback -> from inside the behaviours
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)

    try:
        node = DecisionMaker()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
