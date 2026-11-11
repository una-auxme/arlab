"""ROS 2 entrypoint for Arlab's prototype HFSM + py_trees architecture.

Configure all parameters here. Independent timers/subscribers monitor safety
inputs even if no Behavior Tree ticks occur (e.g. during external Recovery).
"""

import json

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Int32MultiArray, String
from std_srvs.srv import Trigger

from .bt_controller import BehaviorTreeController
from .heartbeat_monitor import HeartbeatMonitor
from .prototype_actions import TaskContext
from .recovery import RecoveryManager
from .state_machine import DecisionStateMachine


class DecisionMaker(Node):
    """ROS adapter; supervision logic stays testable without ROS in state_machine."""

    def __init__(self) -> None:
        super().__init__("decision_maker")

        # Task selection and prototype execution mode.
        task = self.declare_parameter("task", "laundry").value
        simulate_actions = self.declare_parameter("simulate_actions", False).value
        fail_step_once = self.declare_parameter("fail_step_once", "").value

        # TODO: Replace this *prototype status adapter* with the robot's
        # documented safety-state source (e.g. teach pendant / driver).
        # A Bool topic is NOT itself a safety-rated E-stop implementation.

        monitor_rate_hz = self.declare_parameter("monitor_rate_hz", 20.0).value
        bt_tick_rate_hz = self.declare_parameter("bt_tick_rate_hz", 5.0).value
        max_recoveries = self.declare_parameter("max_recoveries", 3).value
        require_restart_authorization = self.declare_parameter(
            "require_restart_authorization", True
        ).value

        if monitor_rate_hz <= 0 or bt_tick_rate_hz <= 0:
            raise ValueError("Timer frequencies must be positive")

        logger = lambda message: self.get_logger().info(message)
        self.task_context = TaskContext(
            simulate_actions=simulate_actions,
            fail_step_once=fail_step_once,
            log=logger,
        )
        self.controller = BehaviorTreeController(task=task, context=self.task_context)
        self.machine = DecisionStateMachine(
            heartbeat=HeartbeatMonitor(node = self),
            bt_controller=self.controller,
            recovery=RecoveryManager(log=logger),
            max_recoveries=max_recoveries,
            require_restart_authorization=require_restart_authorization,
            log=logger,
        )

        # Monitoring and task timer have independent callback groups. They
        # can be dispatched by different ROS executor threads; the state
        # machine's lock serializes state transitions with BT ticks.
        monitor_group = ReentrantCallbackGroup()
        task_group = MutuallyExclusiveCallbackGroup()
        self.create_timer(
            1.0 / monitor_rate_hz,
            self.machine.monitor_tick,
            callback_group=monitor_group,
        )
        self.create_timer(
            1.0 / bt_tick_rate_hz,
            self.machine.tick_bt,
            callback_group=task_group,
        )
        self.create_service(
            Trigger,
            "~/confirm_recovery_checked",
            self._confirm_recovery,
            callback_group=monitor_group,
        )
        self.create_service(
            Trigger,
            "~/authorize_restart",
            self._authorize_restart,
            callback_group=monitor_group,
        )
        self.status_publisher = self.create_publisher(String, "~/state", 10)
        self.create_timer(1.0, self._publish_status, callback_group=monitor_group)
        self.get_logger().info(
            f"Ready: task={task}, simulation={simulate_actions}, "
        )
        self.get_logger().warn(
            "Prototype only. E-stop Bool and recovery acknowledgement are "
            "NOT robot safety functions. Real action cancellation is TODO."
        )

    def _confirm_recovery(self, request, response):
        # TODO: Replace this manual prototype acknowledgement with validated
        # checks: robot physical state, canceled goals, fresh scene/KB data.
        self.machine.confirm_environment_checked()
        response.success = True
        response.message = "Prototype environment-check acknowledgement recorded"
        return response

    def _authorize_restart(self, request, response):
        # TODO: Integrate authenticated operator/robot restart authorization.
        # This request never releases a hardware emergency stop.
        self.machine.authorize_restart()
        response.success = True
        response.message = "Prototype restart authorization recorded"
        return response

    def _publish_status(self) -> None:
        msg = String()
        msg.data = json.dumps(self.machine.status(), sort_keys=True)
        self.status_publisher.publish(msg)

    def shutdown(self) -> None:
        self.controller.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    executor = MultiThreadedExecutor(num_threads=3)
    node = None
    try:
        node = DecisionMaker()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.shutdown()
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
