#!/usr/bin/env python3
"""Republish MoveIt's octomap as a PointCloud2 coloured by distance to the camera.

Reads the octomap from move_group (/get_planning_scene) once per second and publishes
the occupied voxel centres on /octomap_distance_cloud (frame: octomap frame). The
`intensity` field holds the distance from the camera in metres, so RViz's PointCloud2
display with the "Intensity" colour transformer can draw near voxels red and far ones blue.

Run with:
    python3 octomap_distance_cloud.py --ros-args -p camera_frame:=camera_depth_optical_frame
"""

import struct

import numpy as np
import rclpy
import tf2_ros
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField

TREE_DEPTH = 16


def occupied_voxels(data, resolution):
    """Decode a full (non-binary) OcTree stream and return the centres of occupied leaves."""
    buf = bytes(data)
    if not buf:
        return np.zeros((0, 3), dtype=np.float32)
    centres = []
    pos = 0
    # Each node is a float log-odds value followed by a byte flagging which children exist.
    stack = [(0.0, 0.0, 0.0, resolution * 2**TREE_DEPTH)]
    while stack:
        cx, cy, cz, size = stack.pop()
        (log_odds,) = struct.unpack_from("<f", buf, pos)
        children = buf[pos + 4]
        pos += 5
        if children == 0:
            if log_odds > 0.0:
                centres.append((cx, cy, cz))
            continue
        q = size / 4
        # Children are serialised in order 0..7, so push them in reverse onto the stack.
        for i in reversed(range(8)):
            if children & (1 << i):
                stack.append((cx + (q if i & 1 else -q), cy + (q if i & 2 else -q), cz + (q if i & 4 else -q), size / 2))
    return np.array(centres, dtype=np.float32)


class OctomapDistanceCloud(Node):
    def __init__(self):
        super().__init__("octomap_distance_cloud")
        self.camera_frame = self.declare_parameter("camera_frame", "camera_depth_optical_frame").value
        period = 1.0 / self.declare_parameter("rate", 1.0).value
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.client = self.create_client(GetPlanningScene, "/get_planning_scene")
        self.publisher = self.create_publisher(PointCloud2, "/octomap_distance_cloud", 1)
        self.pending = None
        self.pending_since = None
        self.create_timer(period, self.request_octomap)

    def request_octomap(self):
        if self.pending is not None:
            # A request can hang forever if move_group restarts while it is in flight; drop it after 5 s.
            if (self.get_clock().now() - self.pending_since).nanoseconds < 5e9:
                return
            self.get_logger().warn("No answer from /get_planning_scene for 5 s, retrying")
            self.client.remove_pending_request(self.pending)
            self.pending = None
        if not self.client.service_is_ready():
            return
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.OCTOMAP
        self.pending = self.client.call_async(request)
        self.pending_since = self.get_clock().now()
        self.pending.add_done_callback(self.publish_cloud)

    def publish_cloud(self, future):
        if future is not self.pending:
            return
        self.pending = None
        octomap = future.result().scene.world.octomap
        frame = octomap.header.frame_id or "world"
        try:
            t = self.tf_buffer.lookup_transform(frame, self.camera_frame, rclpy.time.Time()).transform.translation
        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f"No transform {frame} -> {self.camera_frame}: {ex}", throttle_duration_sec=5.0)
            return
        voxels = occupied_voxels(octomap.octomap.data, octomap.octomap.resolution)
        distance = np.linalg.norm(voxels - np.array([t.x, t.y, t.z], dtype=np.float32), axis=1)
        points = np.hstack([voxels, distance[:, None]]).astype(np.float32)

        cloud = PointCloud2()
        cloud.header.frame_id = frame
        cloud.header.stamp = self.get_clock().now().to_msg()
        cloud.height, cloud.width = 1, len(points)
        cloud.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate("xyz")]
        cloud.fields.append(PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1))
        cloud.point_step, cloud.row_step = 16, 16 * len(points)
        cloud.is_dense = True
        cloud.data = points.tobytes()
        self.publisher.publish(cloud)


def main():
    rclpy.init()
    rclpy.spin(OctomapDistanceCloud())


if __name__ == "__main__":
    main()
