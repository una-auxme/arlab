"""Contains functions to easily create debug markers that can be visualized in RViz

Overview of the main components:
- debug_marker(): Creates a ROS Marker based on different objects
- debug_marker_array(): Creates a ROS MarkerArray
  based on list of ROS Markers
"""

from collections.abc import Sequence
from typing import List, Optional, Tuple

from builtin_interfaces.msg import Duration as DurationMsg
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Point, Pose, Vector3
from rclpy.duration import Duration
from ros2_numpy.point_cloud2 import pointcloud2_to_array, split_rgb_field
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def debug_marker(
    base: Marker | str | Pose | Point | PointCloud2 | Sequence,
    frame_id: Optional[str] = "world",
    pose: Optional[Pose] = None,
    offset: Optional[Vector3] = None,
    color: Optional[Tuple[float, float, float, float]] = None,
    size_modifier: float = 1.0,
) -> Marker:
    """Creates a ROS Marker based on *base*

    Args:
        base (Any): Currently supported: Entity, Shape, Marker, str, Point2,
            [Point2, Point2] as Arrow
        frame_id (Optional[str], optional): Defaults to "world".
        pose (Optional[Pose], optional): Defaults to None.
            If None, the pose of base will be used.
        offset (Optional[Vector3], optional): Offset Vector.
            Added to the position of the marker. Defaults to None.
        color (Optional[Tuple[float, float, float, float]], optional):
            (r, g, b, a) color tuple. Defaults to (0.5, 0.5, 0.5, 0.5).

    Raises:
        TypeError: If the type of base is unsupported

    Returns:
        Marker: Marker
    """
    if isinstance(base, Marker):
        marker = base
    elif isinstance(base, str):
        marker = Marker(type=Marker.TEXT_VIEW_FACING, text=base)
        # https://github.com/ros2/rviz/pull/1261
        # Scale x controls the width of spaces
        marker.scale.x = 0.2
    elif isinstance(base, Pose):
        marker = Marker(type=Marker.ARROW)
        marker.pose = base
        marker.points.append(Point(x=0.0, y=0.0, z=size_modifier))
        # Arrow points against the local z direction of the pose
        marker.points.append(Point())
        # Scale xy controls arrow shaft and head thickness
        marker.scale.x = 0.1 * size_modifier
        marker.scale.y = 0.3 * size_modifier
        marker.scale.z = 0.3 * size_modifier
    elif isinstance(base, Point):
        marker = Marker(type=Marker.POINTS, points=[base])
    elif isinstance(base, PointCloud2):
        # Extract points from PointCloud2
        structured_points = split_rgb_field(pointcloud2_to_array(base))

        points: List[Point] = []
        colors: List[ColorRGBA] = []
        for p in structured_points:
            print(p["r"])
            points.append(Point(x=float(p["x"]), y=float(p["y"]), z=float(p["z"])))
            colors.append(
                ColorRGBA(r=p["r"] / 255.0, g=p["g"] / 255.0, b=p["b"] / 255.0, a=1.0)
            )

        marker = Marker(type=Marker.POINTS)
        marker.points = points
        marker.colors = colors

        marker.scale.x = size_modifier
        marker.scale.y = size_modifier
        marker.scale.z = size_modifier

        # Generate a unique color for this entity based on its ID
        # This ensures each entity has a distinct, consistent color
        # import colorsys
        # import hashlib

        # # Create a hash from the entity ID to get consistent colors
        # hash_obj = hashlib.md5(str(marker_id_val).encode())
        # hash_int = int(hash_obj.hexdigest()[:8], 16)

        # # Generate RGB values from hash using HSV color space
        # # for better color distribution
        # hue = (hash_int % 360) / 360.0  # 0.0 to 1.0
        # saturation = 0.7 + (hash_int % 30) / 100.0  # 0.7 to 1.0
        # value = 0.8 + (hash_int % 20) / 100.0  # 0.8 to 1.0

        # r, g, b = colorsys.hsv_to_rgb(hue, saturation, value)
    elif isinstance(base, Sequence):
        if len(base) == 2:
            p0 = base[0]
            p1 = base[1]
            if not (isinstance(p0, Point) and isinstance(p1, Point)):
                raise TypeError(
                    f"Unsupported debug marker base sequence: '{type(p0)}, {type(p1)}'"
                )
            marker = Marker(type=Marker.ARROW)
            marker.pose.position = p0
            # Scale xy controls arrow shaft and head thickness
            marker.scale.x = 0.1 * size_modifier
            marker.scale.y = 0.3 * size_modifier
            marker.points.append(Point())
            # Seconds point is Vector from the starting pose
            v = Point(x=p1.x - p0.x, y=p1.y - p0.y, z=p1.z - p0.z)
            marker.points.append(v)
        else:
            raise TypeError(
                f"Unsupported debug marker base sequence length: '{len(base)}'"
            )
    else:
        raise TypeError(f"Unsupported debug marker base type: '{type(base)}'")

    if frame_id:
        marker.header.frame_id = frame_id
    if pose:
        marker.pose = pose
    if offset:
        marker.pose.position.x += offset.x
        marker.pose.position.y += offset.y
        marker.pose.position.z += offset.z
    if color is None:
        color = (0.5, 0.5, 0.5, 0.5)
    (
        marker.color.r,
        marker.color.g,
        marker.color.b,
        marker.color.a,
    ) = color
    if marker.scale.z == 0.0:
        if marker.type == Marker.TEXT_VIEW_FACING:
            marker.scale.z = 0.3 * size_modifier
        else:
            marker.scale.z = 1.0

    return marker


def debug_marker_array(
    namespace: str,
    markers: List[Marker],
    timestamp: TimeMsg,
    lifetime: Optional[DurationMsg] = None,
) -> MarkerArray:
    """Builds a ROS MarkerArray based on *markers*

    Args:
        namespace (str): Namespace of the markers
        markers (List[Marker]): markers
        timestamp (builtin_interfaces.msg.Time): Timestamp of all markers.
        lifetime (Optional[builtin_interfaces.msg.Duration], optional): Marker lifetime.
            Defaults to 0.5.

    Returns:
        MarkerArray: MarkerArray
    """
    if lifetime is None:
        lifetime = Duration(seconds=0.5).to_msg()

    marker_array = MarkerArray(
        markers=[Marker(id=0, ns=namespace, action=Marker.DELETEALL)]
    )
    for id, marker in enumerate(markers):
        marker.header.stamp = timestamp
        marker.ns = namespace
        marker.id = id + 1
        marker.lifetime = lifetime
        marker_array.markers.append(marker)

    return marker_array
