#!/usr/bin/env python3
"""Turn the RealSense depth image into a range-limited point cloud for MoveIt's octomap.

Why: the camera container runs without a depth clip, and MoveIt's DepthImageOctomapUpdater
can't "clear without inserting" (far pixels either cast no ray or become phantom voxels).
MoveIt's PointCloudOctomapUpdater can: points beyond its `max_range` are ray-cast as free
space only. So pixels farther than max_range are pulled in along their ray to
max_range + far_margin -> the space in front of an empty (>2 m) background gets cleared,
without voxels at the range limit and with bounded ray length. Invalid pixels (0) are dropped.

De-duplication: many pixels hit the same octomap voxel, but MoveIt casts one ray per point.
The cloud is therefore reduced to one real point per voxel of the octomap grid in `map_frame`
(dedup_resolution must equal octomap_resolution in sensor3D.yaml; 0 disables it). Hits and
clearing-only points are de-duplicated separately. The cloud stays in the camera frame, since
MoveIt uses the cloud frame as the ray origin.

In : /camera_gripper/depth/image_rect_raw + camera_info       (848x480 @ 30 Hz, 16UC1 mm)
Out: /camera_gripper/depth_clipped/points       PointCloud2 xyz, depth optical frame (424x240 @ 15 Hz)
     /camera_gripper/depth_clipped/image_rect_raw + camera_info  (far pixels -> 0; for viewing/debug)
frame_skip=n publishes every n-th camera frame (camera: 30 Hz).
far_mode=free (default) clears in front of far background; far_mode=zero drops far pixels (old behaviour).
"""
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from tf2_ros import Buffer, TransformListener


class DepthClipRelay(Node):
    def __init__(self):
        super().__init__("depth_clip_relay")
        for name, default in (("max_range", 2.0), ("far_margin", 0.05), ("far_mode", "free"),
                              ("stride", 2), ("frame_skip", 2),
                              ("dedup_resolution", 0.02), ("map_frame", "world")):
            self.declare_parameter(name, default)
        self.info, self.rays, self.ray_key, self.n = None, None, None, 0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        out_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_pts = self.create_publisher(PointCloud2, "/camera_gripper/depth_clipped/points", out_qos)
        self.pub_img = self.create_publisher(Image, "/camera_gripper/depth_clipped/image_rect_raw", out_qos)
        self.pub_info = self.create_publisher(CameraInfo, "/camera_gripper/depth_clipped/camera_info", out_qos)
        self.create_subscription(CameraInfo, "/camera_gripper/depth/camera_info", self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, "/camera_gripper/depth/image_rect_raw", self.on_image, qos_profile_sensor_data)

    def p(self, name):
        return self.get_parameter(name).value

    def on_info(self, msg):
        s = int(self.p("stride"))
        msg.width, msg.height = -(-msg.width // s), -(-msg.height // s)   # = size of d[::s, ::s]
        k, p = list(msg.k), list(msg.p)
        for i in (0, 2, 4, 5): k[i] /= s       # fx, cx, fy, cy
        for i in (0, 2, 5, 6): p[i] /= s
        msg.k, msg.p = k, p
        key = (msg.width, msg.height, *map(float, k))
        if key != self.ray_key:                              # (re)compute pixel rays when intrinsics change
            self.ray_key = key
            u, v = np.meshgrid(np.arange(msg.width), np.arange(msg.height))
            self.rays = np.dstack(((u - k[2]) / k[0], (v - k[5]) / k[4], np.ones(u.shape))).astype(np.float32)  # z = 1
        self.info = msg

    def dedup(self, pts, far, res, header):
        """Keep one real point per voxel of the octomap grid (in map_frame); hits and far points separately."""
        # The TF only groups points into voxels (the points themselves are not moved), so a slightly
        # older TF is fine: use the image time if available, otherwise the latest TF. Never block.
        tf = None
        for stamp in (Time.from_msg(header.stamp), Time()):
            try:
                tf = self.tf_buffer.lookup_transform(self.p("map_frame"), header.frame_id, stamp).transform
                break
            except Exception as ex:
                err = ex
        if tf is None:
            self.get_logger().warn(f"No TF {self.p('map_frame')} <- {header.frame_id}, skipping frame: {err}",
                                   throttle_duration_sec=5.0)
            return None
        x, y, z, w = tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w
        rot = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]], dtype=np.float32)
        t = np.array([tf.translation.x, tf.translation.y, tf.translation.z], dtype=np.float32)
        key = np.floor((pts @ rot.T + t) / res).astype(np.int64) + (1 << 19)   # octomap grid cell (20 bit / axis)
        h = (((key[:, 0] << 20) | key[:, 1]) << 20 | key[:, 2]) << 1 | far     # + hit/far flag
        _, idx = np.unique(h, return_index=True)
        return pts[idx]

    def on_image(self, msg):
        self.n += 1
        if self.info is None or self.n % int(self.p("frame_skip")) or msg.encoding != "16UC1":
            return
        s, max_r = int(self.p("stride")), float(self.p("max_range"))
        d = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.step // 2)[:, : msg.width][::s, ::s]
        if d.shape != self.rays.shape[:2]:
            return
        z = d.astype(np.float32) * 0.001
        pts = self.rays * z[..., None]                       # xyz in the optical frame
        rng = np.linalg.norm(pts, axis=2)
        far = rng > max_r
        if self.p("far_mode") == "free":                     # pull far points in -> free-space ray only
            pts[far] *= ((max_r + float(self.p("far_margin"))) / rng[far])[:, None]
            keep = z > 0
        else:
            keep = (z > 0) & ~far
        pts, far_pts = pts[keep], far[keep]
        res = float(self.p("dedup_resolution"))
        if res > 0:
            pts = self.dedup(pts, far_pts, res, msg.header)
            if pts is None:
                return

        cloud = PointCloud2(header=msg.header, height=1, width=len(pts), is_bigendian=False,
                            point_step=12, row_step=12 * len(pts), is_dense=True,
                            fields=[PointField(name=a, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                                    for i, a in enumerate("xyz")])
        cloud.data = pts.astype(np.float32).tobytes()
        self.pub_pts.publish(cloud)

        img = d.copy(); img[(z > 0) & far] = 0                # clipped depth image, for viewing
        self.info.header = msg.header
        self.pub_info.publish(self.info)
        self.pub_img.publish(Image(header=msg.header, height=img.shape[0], width=img.shape[1], encoding="16UC1",
                                   is_bigendian=msg.is_bigendian, step=img.shape[1] * 2, data=img.tobytes()))


def main():
    rclpy.init()
    rclpy.spin(DepthClipRelay())


if __name__ == "__main__":
    main()
