"""Relay of LIDAR LaserScan messages using ROS 2.

This module defines a ROS 2 node that subscribes to ``/scan`` and republishes
incoming ``sensor_msgs.msg.LaserScan`` messages unchanged to ``/relay_scan``.

Maintainer:
    Meruna Yugarajah <m.yugarajah@gmail.com>
"""

from typing import Optional, Sequence

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarPubSub(Node):
    """Relay node for LIDAR scans.

    This node subscribes to ``/scan`` (``LaserScan``) and republishes each
    incoming message unchanged to ``/relay_scan``. It preserves the original
    header, ranges/intensities arrays, and all metadata.

    Topic Interface:
        * **Input**  ``/scan`` – ``sensor_msgs.msg.LaserScan``
        * **Output** ``/relay_scan`` – ``sensor_msgs.msg.LaserScan`` (forwarded)

    Notes:
        - Uses the default QoS profile with depth=10 (other QoS settings remain
        the ROS 2 defaults for rclpy).
        - No filtering, throttling, or frame transforms are applied.
        - If you later add filtering, publish a *new* message instance before
        mutating any fields to avoid side effects for other subscribers.

    Attributes:
        lidar_sub: Subscription handle for ``/scan``.
        lidar_pub: Publisher handle for ``/relay_scan``.
    """

    def __init__(self) -> None:
        """Construct the node and wire up the ROS 2 interfaces.

        Initializes the underlying ``Node`` with the class name as node name,
        creates the ``/scan`` subscription, and the ``/relay_scan`` publisher.
        The publisher handle is stored on ``self`` to prevent garbage collection,
        which would otherwise disable outgoing messages.

        Side Effects:
            Registers a subscription callback with the executor.
        """
        super().__init__(self.__class__.__name__)

        self.lidar_sub = self.create_subscription(
            msg_type=LaserScan,
            topic="/scan",
            callback=self.lidar_callback,
            qos_profile=10,  # depth; leave other QoS settings at defaults
        )

        self.lidar_pub = self.create_publisher(
            msg_type=LaserScan,
            topic="/relay_scan",
            qos_profile=10,  # depth; leave other QoS settings at defaults
        )

    def lidar_callback(self, msg: LaserScan) -> None:
        """Forward an incoming scan to ``/relay_scan`` unchanged.

        Args:
            msg: The received scan. All fields (``header``, ``angle_*``,
                ``range_*``, ``ranges``, ``intensities``) are forwarded as-is.

        Side Effects:
            Publishes the message on ``/relay_scan``.

        Threading:
            Called by the rclpy executor thread associated with this node.
        """
        self.lidar_pub.publish(msg)


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = LidarPubSub()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
