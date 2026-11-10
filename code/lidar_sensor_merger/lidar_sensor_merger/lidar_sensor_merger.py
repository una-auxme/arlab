import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from sensor_msgs.msg import LaserScan

from tf2_ros import Buffer, TransformListener, TransformException


class ScanMerger(Node):

    def __init__(self):
        super().__init__('scan_merger')

        # Parameters
        self.declare_parameter('target_frame', 'ranger_merged_sensor_link')
        self.declare_parameter('angle_min', -math.pi)
        self.declare_parameter('angle_max', math.pi)
        self.declare_parameter('angle_increment', math.radians(0.1))
        self.declare_parameter('range_min', 0.05)
        self.declare_parameter('range_max', 20.0)
        self.declare_parameter('max_sync_slop', 0.1)

        self.target_frame = self.get_parameter(
            'target_frame').value

        self.angle_min = self.get_parameter(
            'angle_min').value

        self.angle_max = self.get_parameter(
            'angle_max').value

        self.angle_increment = self.get_parameter(
            'angle_increment').value

        self.range_min = self.get_parameter(
            'range_min').value

        self.range_max = self.get_parameter(
            'range_max').value

        self.max_sync_slop = self.get_parameter(
            'max_sync_slop').value

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer, self
        )

        # Subscribers
        self.front_sub = self.create_subscription(
            LaserScan,
            '/scan_lidar_front',
            self.front_callback,
            10
        )

        self.back_sub = self.create_subscription(
            LaserScan,
            '/scan_lidar_back',
            self.back_callback,
            10
        )

        # Publisher
        self.scan_pub = self.create_publisher(
            LaserScan,
            '/scan',
            10
        )

        self.front_scan = None
        self.back_scan = None

        self.num_bins = int(
            round(
                (self.angle_max - self.angle_min)
                / self.angle_increment
            )
        ) + 1

        self.get_logger().info(
            'LaserScan merger started'
        )

    def front_callback(self, msg):
        self.front_scan = msg
        self.merge_scans()

    def back_callback(self, msg):
        self.back_scan = msg
        self.merge_scans()

    def transform_point(self, x, y, transform):

        t = transform.transform.translation
        q = transform.transform.rotation

        # Rotate point using quaternion
        tx = 2.0 * (q.y * 0.0 - q.z * y)
        ty = 2.0 * (q.z * x - q.x * 0.0)
        tz = 2.0 * (q.x * y - q.y * x)

        rx = x + q.w * tx + (q.y * tz - q.z * ty)
        ry = y + q.w * ty + (q.z * tx - q.x * tz)

        # Translate point
        rx += t.x
        ry += t.y

        return rx, ry

    def merge_scans(self):

        if self.front_scan is None or self.back_scan is None:
            return

        # Avoid merging scans that are too far apart in time
        front_time = Time.from_msg(
            self.front_scan.header.stamp
        ).nanoseconds / 1e9

        back_time = Time.from_msg(
            self.back_scan.header.stamp
        ).nanoseconds / 1e9

        if abs(front_time - back_time) > self.max_sync_slop:
            return

        # Initialize output scan
        ranges = [float('inf')] * self.num_bins

        scans = [
            self.front_scan,
            self.back_scan
        ]

        for scan in scans:

            try:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    scan.header.frame_id,
                    Time.from_msg(scan.header.stamp),
                    timeout=Duration(seconds=0.1)
                )

            except TransformException as ex:
                self.get_logger().warning(
                    f'TF transformation failed: {ex}'
                )
                return

            for i, distance in enumerate(scan.ranges):

                # Ignore invalid measurements
                if not math.isfinite(distance):
                    continue

                if distance < scan.range_min or distance > scan.range_max:
                    continue

                angle = (
                    scan.angle_min
                    + i * scan.angle_increment
                )

                # Point in LiDAR coordinate system
                x = distance * math.cos(angle)
                y = distance * math.sin(angle)

                # Transform into common coordinate system
                x, y = self.transform_point(
                    x, y, transform
                )

                # Convert back into polar coordinates
                merged_range = math.hypot(x, y)
                merged_angle = math.atan2(y, x)

                if not (
                    self.angle_min <= merged_angle <= self.angle_max
                ):
                    continue

                index = int(round(
                    (merged_angle - self.angle_min)
                    / self.angle_increment
                ))

                if 0 <= index < self.num_bins:

                    # Keep nearest obstacle
                    if self.range_min <= merged_range <= self.range_max:

                        ranges[index] = min(
                            ranges[index],
                            merged_range
                        )

        # Construct output message
        output = LaserScan()

        output.header.stamp = (
            self.front_scan.header.stamp
            if front_time >= back_time
            else self.back_scan.header.stamp
        )

        output.header.frame_id = self.target_frame

        output.angle_min = self.angle_min
        output.angle_max = self.angle_max
        output.angle_increment = self.angle_increment

        output.time_increment = 0.0
        output.scan_time = max(
            self.front_scan.scan_time,
            self.back_scan.scan_time
        )

        output.range_min = self.range_min
        output.range_max = self.range_max

        output.ranges = ranges
        output.intensities = []

        self.scan_pub.publish(output)


def main(args=None):

    rclpy.init(args=args)

    node = ScanMerger()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
