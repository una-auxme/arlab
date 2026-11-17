"""ROS-independent observations for the heartbeat orthogonal region.

The current arlab_safety convention is /global_heartbeat, Int32MultiArray:
[module_id, module_state] with -1=registering, 0=healthy, >0=error.
"""

from dataclasses import dataclass
import time
from typing import Callable, Dict, Iterable
from arlab_common_interfaces.msg import SystemFreeze
from rclpy.node import Node


@dataclass
class HeartbeatSample:
    status: int
    last_seen: float


class HeartbeatMonitor():
    """Track ONLY required module IDs; fail closed if any are missing/stale."""

    def __init__(self, node: Node) -> None:
        self.node = node
        self.subscription = self.node.create_subscription(
            SystemFreeze,                                              # CHANGE
            'system_freeze',
            self.listener_callback,
            10)
        self.subscription
        self.freeze = False

    def listener_callback(self, msg: bool) -> None:
        self.node.get_logger().info(f'Freeze massage: {msg.freeze}')
        self.freeze = msg.freeze


    @property
    def healthy(self) -> bool:
        return not self.freeze
