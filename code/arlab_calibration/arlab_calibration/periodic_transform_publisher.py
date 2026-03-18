"""Periodic transform publisher node

Node functionality:
    Periodically broadcasts a static transform between two TF frames
    Reads parent frame, child frame, translation, rotation, and period
    from ROS2 parameters

Maintainers:
    Peter Viechter <peter.viechter@uni-augsburg.de>
    Daniel Gabler <daniel.gabler@uni-augsburg.de>
"""

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class PeriodicTransformPublisher(Node):
    """ROS2 node for periodically broadcasting a transform.

    This node reads transform parameters from the ROS2 parameter server,
    creates a TransformStamped message, and republishes it periodically
    using a static transform broadcaster.
    """
    def __init__(self):
        """Initialize the transform publisher node.

        Declares and reads all transform-related ROS2 parameters, creates the
        transform message, initializes the static transform broadcaster, and
        starts a timer for periodic broadcasting.
        Parameters:
            frame_id (str): The parent frame ID for the transform.
            child_frame_id (str): The child frame ID for the transform.
            x (float): Translation in x direction.
            y (float): Translation in y direction.
            z (float): Translation in z direction.
            qx (float): Rotation quaternion x component.
            qy (float): Rotation quaternion y component.
            qz (float): Rotation quaternion z component.
            qw (float): Rotation quaternion w component.
            period (float): Time interval in seconds between broadcasts.
        """
        super().__init__(type(self).__name__)

        self.frame_id = self.declare_parameter("frame_id", "").get_parameter_value().string_value
        self.child_frame_id = self.declare_parameter("child_frame_id", "").get_parameter_value().string_value

        self.x = self.declare_parameter("x", 0.0).get_parameter_value().double_value
        self.y = self.declare_parameter("y", 0.0).get_parameter_value().double_value
        self.z = self.declare_parameter("z", 0.0).get_parameter_value().double_value
        self.qx = self.declare_parameter("qx", 0.0).get_parameter_value().double_value
        self.qy = self.declare_parameter("qy", 0.0).get_parameter_value().double_value
        self.qz = self.declare_parameter("qz", 0.0).get_parameter_value().double_value
        self.qw = self.declare_parameter("qw", 1.0).get_parameter_value().double_value

        self.period = self.declare_parameter("period", 1.0).get_parameter_value().double_value

        self.transform = TransformStamped()
        self.transform.header.frame_id = self.frame_id
        self.transform.child_frame_id = self.child_frame_id
        self.transform.transform.translation.x = self.x
        self.transform.transform.translation.y = self.y
        self.transform.transform.translation.z = self.z
        self.transform.transform.rotation.x = self.qx
        self.transform.transform.rotation.y = self.qy
        self.transform.transform.rotation.z = self.qz
        self.transform.transform.rotation.w = self.qw

        self.get_logger().info(f"Broadcasting transform: {self.transform}")

        self.broadcaster = StaticTransformBroadcaster(self)

        self.create_timer(self.period, callback=self._broadcast_transform)

    def _broadcast_transform(self):
        """Broadcast the configured transform with the current timestamp."""
        self.transform.header.stamp = self.get_clock().now().to_msg()
        self.broadcaster.sendTransform(self.transform)


def main(args=None):
    """Initialize ROS, start the transform publisher node, and spin it."""
    # from arlab_common.debugging import start_debugger
    # start_debugger(wait_for_client=True)

    rclpy.init(args=args)
    try:
        node = PeriodicTransformPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
