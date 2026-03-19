import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class PeriodicTransformPublisher(Node):
    def __init__(self):
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
        self.transform.header.stamp = self.get_clock().now().to_msg()
        self.broadcaster.sendTransform(self.transform)


def main(args=None):
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
