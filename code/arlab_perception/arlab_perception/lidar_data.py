"""Relay of LIDAR LaserScan messages using ROS 2.

This module defines a ROS 2 node that subscribes to ``/scan`` and republishes
incoming ``sensor_msgs.msg.LaserScan`` messages unchanged to ``/relay_scan``.

Maintainer:
    Meruna Yugarajah <m.yugarajah@gmail.com>
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarPubSub(Node):
    """ROS 2 node that forwards LIDAR scans without modification.

    The node subscribes to ``/scan`` and publishes identical messages on
    ``/relay_scan``. No filtering or transformations are applied.

    Attributes:
        lidar_sub (rclpy.subscription.Subscription): Subscription to ``/scan``
            for ``LaserScan`` messages.
        lidar_pub (rclpy.publisher.Publisher): Publisher to ``/relay_scan`` for
            ``LaserScan`` messages.
    """

    def __init__(self):
        """Initialize node, subscription, and publisher.

        Creates a subscription to ``/scan`` and a publisher to ``/relay_scan``.
        Uses the default QoS with queue depth 10.
        """

        super().__init__(type(self).__name__)

        self.lidar_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.lidar_callback,
            10, # queue depth
        )

        # Keep a reference so the publisher isn't garbage-collected.
        self.lidar_pub = self.create_publisher(LaserScan, "/relay_scan", 10)

    def lidar_callback(self, msg: LaserScan):
        """Republish an incoming ``LaserScan`` message unchanged.

        Args:
            msg (LaserScan): The received LIDAR scan (header, angle/range
                metadata, ranges, and intensities), forwarded as-is.
        """
        
        self.lidar_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    lidar_node = LidarPubSub()
    rclpy.spin(lidar_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
