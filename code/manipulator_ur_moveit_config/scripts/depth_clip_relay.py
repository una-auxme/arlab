#!/usr/bin/env python3
"""Turn the RealSense depth image into a range-limited point cloud for MoveIt's octomap.

Why: the camera container runs without a depth clip, and MoveIt's DepthImageOctomapUpdater
can't "clear without inserting" (far pixels either cast no ray or become phantom voxels).
MoveIt's PointCloudOctomapUpdater can: points beyond its `max_range` are ray-cast as free
space only. So pixels farther than max_range are pulled in along their ray to
max_range + far_margin -> the space in front of an empty (>2 m) background gets cleared,
without voxels at the range limit and with bounded ray length. Invalid pixels (0) are dropped.

In : /camera_gripper/depth/image_rect_raw + camera_info       (848x480 @ 30 Hz, 16UC1 mm)
Out: /camera_gripper/depth_clipped/points       PointCloud2 xyz, depth optical frame (424x240 @ 15 Hz)
     /camera_gripper/depth_clipped/image_rect_raw + camera_info  (far pixels -> 0; for viewing/debug)
far_mode=free (default) clears in front of far background; far_mode=zero drops far pixels (old behaviour).
"""
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField


class DepthClipRelay(Node):
    def __init__(self):
        super().__init__("depth_clip_relay")
        for name, default in (("max_range", 2.0), ("far_margin", 0.05), ("far_mode", "free"),
                              ("stride", 2), ("frame_skip", 2)):
            self.declare_parameter(name, default)
        self.info, self.rays, self.ray_key, self.n = None, None, None, 0
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
        pts = pts[keep]

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
