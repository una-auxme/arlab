"""Contains functions to easily create debug markers that can be visualized in RViz

Overview of the main components:
- debug_marker(): Creates a ROS Marker based on different objects
- debug_marker_array(): Creates a ROS MarkerArray
  based on list of ROS Markers
"""

from collections.abc import Sequence
from typing import List, Optional, Tuple

import arlab_knowledge.db.entities.entity
import arlab_knowledge.db.entities.shape
from builtin_interfaces.msg import Duration as DurationMsg
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Point, Pose, Vector3
from rclpy.duration import Duration
from visualization_msgs.msg import Marker, MarkerArray


def debug_marker(
    base: arlab_knowledge.db.entities.entity.Entity
    | arlab_knowledge.db.entities.shape.Shape
    | Marker
    | str
    | Pose
    | Point
    | Sequence,
    frame_id: Optional[str] = "world",
    pose: Optional[Pose] = None,
    offset: Optional[Vector3] = None,
    color: Optional[Tuple[float, float, float, float]] = None,
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
    if isinstance(base, arlab_knowledge.db.entities.entity.Entity):
        marker = base.get_pose_marker()
    elif isinstance(base, arlab_knowledge.db.entities.shape.Shape):
        # TODO: Implement to_marker in shape and use here
        raise NotImplementedError()
    elif isinstance(base, Marker):
        marker = base
    elif isinstance(base, str):
        marker = Marker(type=Marker.TEXT_VIEW_FACING, text=base)
        # https://github.com/ros2/rviz/pull/1261
        # Scale x controls the width of spaces
        marker.scale.x = 0.2
    elif isinstance(base, Pose):
        marker = Marker(type=Marker.ARROW)
        marker.pose = base
        marker.points.append(Point())
        # Arrow points in the local z direction of the pose
        marker.points.append(Point(x=0.0, y=0.0, z=1.0))
        # Scale xy controls arrow shaft and head thickness
        marker.scale.x = 0.1
        marker.scale.y = 0.3
    elif isinstance(base, Point):
        marker = Marker(type=Marker.POINTS, points=[base])
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
            marker.scale.x = 0.1
            marker.scale.y = 0.3
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
            marker.scale.z = 0.3
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
