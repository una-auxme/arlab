# ------------------------------------------------------------
# Filename: lidar_pubsub.py
# Description: A ROS 2 Node that subscribes to LIDAR data and relays it.
# Maintainer: Meruna Yugarajah <m.yugarajah@gmail.com>
# Created: 2025-07-10
# Last Modified: 2025-08-14
# License: MIT
# ------------------------------------------------------------

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarPubSub(Node):
    """
    ROS 2 Node that subscribes to LIDAR data from the /scan topic and relays
    the received data unchanged to the /relay_scan topic.
    """

    def __init__(self):
        """
        Initializes the LidarPubSub Node and creates the subscription
        to the /scan topic and the publisher for the /relay_scan topic.
        """
        # Initialize the ROS 2 Node with the class name as the node name
        super().__init__(type(self).__name__)

        # Subscribe to the "/scan" topic (LIDAR data) and set the callback function
        self.lidar_sub = self.create_subscription(
            LaserScan,  # Message type: LaserScan
            "/scan",  # Topic: /scan (LIDAR scanner data)
            self.lidar_callback,  # Callback function when new data is received
            10,  # Queue size for subscription
        )

        # Publisher for the "/relay_scan" topic to forward the received data
        # Keeps a reference to prevent garbage collection
        self.lidar_pub = self.create_publisher(LaserScan, "/relay_scan", 10)

    def lidar_callback(self, msg: LaserScan):
        """
        Callback function that is triggered when a new LaserScan message is received.
        Currently, this function simply relays the received LIDAR data unchanged.
        However, it is designed to be extended in the future to allow for data
        filtering, ensuring that the original data is preserved and not modified.
        """
        # Publish the received LIDAR data to the /relay_scan topic
        self.lidar_pub.publish(msg)


def main(args=None):
    """
    Initializes ROS 2, starts the node, and waits for incoming messages.
    """
    # Initialize the ROS 2 client library
    rclpy.init(args=args)

    # Create an instance of the LidarPubSub Node
    lidar_node = LidarPubSub()

    # Start the node and wait for incoming messages
    rclpy.spin(lidar_node)

    # Shutdown ROS 2 when the node finishes
    rclpy.shutdown()


if __name__ == "__main__":
    # Execute the main function if the script is run directly
    main()
