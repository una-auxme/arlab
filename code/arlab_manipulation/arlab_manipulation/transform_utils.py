import rclpy
import rclpy.duration
from geometry_msgs.msg import Pose, Point, PoseStamped, PointStamped
import numpy as np
from builtin_interfaces.msg import Time

def transform_data(tf_buffer, data, ref_frame, stamp):
    """
    Transform Pose, Point, list of Points, or Nx3 pointcloud to base_frame.
    Assumes PoseStamped/PointStamped already have a valid header.
    """

    base_frame="base_link"
    ref_frame="camera_link"

    # Already stamped -> just transform
    if isinstance(data, (PoseStamped, PointStamped)):
        if not data.header.frame_id:
            raise ValueError("Stamped message missing header.frame_id")
        if not data.header.stamp:
            raise ValueError("Stamped message missing header.stamp")
        return tf_buffer.transform(
            data,
            base_frame,
            timeout=rclpy.duration.Duration(seconds=0.5)
        )

    # Single Pose
    if isinstance(data, Pose):
        ps = PoseStamped()
        ps.header.frame_id = ref_frame
        ps.header.stamp = stamp
        ps.pose = data
        return tf_buffer.transform(ps, base_frame, timeout=rclpy.duration.Duration(seconds=0.5)).pose

    # Single Point
    if isinstance(data, Point):
        pts = PointStamped()
        pts.header.frame_id = ref_frame
        pts.header.stamp = stamp
        pts.point = data
        return tf_buffer.transform(pts, base_frame, timeout=rclpy.duration.Duration(seconds=0.5)).point

    # List of Points
    if isinstance(data, list) and all(isinstance(pt, Point) for pt in data):
        transformed = []
        for pt in data:
            pts = PointStamped()
            pts.header.frame_id = ref_frame
            pts.header.stamp = stamp
            pts.point = pt
            tpt = tf_buffer.transform(pts, base_frame, timeout=rclpy.duration.Duration(seconds=0.5))
            transformed.append(tpt.point)
        return transformed

    # Nx3 NumPy array (pointcloud)
    if isinstance(data, np.ndarray) and data.shape[1] == 3:
        transformed_points = []
        for pt in data:
            p = Point(x=pt[0], y=pt[1], z=pt[2])
            pts = PointStamped()
            pts.header.frame_id = ref_frame
            pts.header.stamp = stamp
            pts.point = p
            tpt = tf_buffer.transform(pts, base_frame, timeout=rclpy.duration.Duration(seconds=0.5))
            transformed_points.append([tpt.point.x, tpt.point.y, tpt.point.z])
        return np.array(transformed_points)

    raise TypeError(f"Unsupported type for transform: {type(data)}")


