import py_trees_ros
import rclpy
import rclpy.callback_groups
import rclpy.executors
from py_trees.behaviour import Behaviour
from py_trees.composites import Sequence
from rclpy.node import Node

from .behaviours import check_safety, task1, task2


def get_tree(task: str) -> Behaviour:
    task = task.lower()
    if task == "task1":
        chosen_task = task1
    elif task == "task2":
        chosen_task = task2
    else:
        raise ValueError(f"Unknown task: {task}")

    return Sequence(
        name="DecisionMaker",
        memory=False,
        children=[
            check_safety.get_tree(),
            chosen_task.get_tree(),
        ],
    )


class DecisionMaker(Node):
    def __init__(self):
        super().__init__(type(self).__name__)
        self.get_logger().info(f"{type(self).__name__} node initializing...")

        # Parameters
        self.update_rate = (
            self.declare_parameter(
                "update_rate",
                10.0,
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
            1.0 / self.update_rate, self.tick_tree_handler
        )
        self.get_logger().info(f"{type(self).__name__} node initialized.")

    def tick_tree_handler(self):
        try:
            self.tick_tree()
        except Exception as e:
            self.get_logger().fatal(f"{e}")
            # self.get_logger().fatal(emsg_with_trace(e), throttle_duration_sec=2)

    def tick_tree(self):
        self.behavior_tree.tick()

    def shutdown(self):
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
