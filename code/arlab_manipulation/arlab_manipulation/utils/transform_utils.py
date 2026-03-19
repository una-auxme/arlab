#!/usr/bin/env python3
"""
voxel_utils.py
---------------

Author: Sofia Öttl
Date: 2025-10-22
"""

from typing import Optional, Tuple

import numpy as np
import rclpy
import rclpy.duration
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Pose, PoseStamped
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose_stamped

target_frame = "base_link"


def transform_pose(tf_buffer, pose: Pose, stamp, ref_frame: str) -> Tuple[Optional[Pose], int, str]:
    try:
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = ref_frame
        pose_stamped.header.stamp = stamp
        pose_stamped.pose = pose

        transform_stamped = tf_buffer.lookup_transform(target_frame, ref_frame, stamp, timeout=rclpy.duration.Duration(seconds=1.0))

        transformed_pose = do_transform_pose_stamped(pose_stamped, transform_stamped)

        return transformed_pose.pose, 1, "Success"

    except Exception as e:
        print("Failed to transform pose:", e)
        return None, -51, "Failed to transform pose"


def transform_pointCloud(tf_buffer, pointCloud, stamp, ref_frame: str):
    try:
        transformed_points = []

        transform_stamped = tf_buffer.lookup_transform(target_frame, ref_frame, stamp, timeout=rclpy.duration.Duration(seconds=1.0))

        t = transform_stamped.transform.translation
        q = transform_stamped.transform.rotation

        qw, qx, qy, qz = q.w, q.x, q.y, q.z
        R = np.array(
            [
                [
                    1 - 2 * qy * qy - 2 * qz * qz,
                    2 * qx * qy - 2 * qz * qw,
                    2 * qx * qz + 2 * qy * qw,
                ],
                [
                    2 * qx * qy + 2 * qz * qw,
                    1 - 2 * qx * qx - 2 * qz * qz,
                    2 * qy * qz - 2 * qx * qw,
                ],
                [
                    2 * qx * qz - 2 * qy * qw,
                    2 * qy * qz + 2 * qx * qw,
                    1 - 2 * qx * qx - 2 * qy * qy,
                ],
            ]
        )
        translation = np.array([t.x, t.y, t.z])

        for pt in pc2.read_points(pointCloud, field_names=["x", "y", "z"], skip_nans=True):
            p = np.array([pt[0], pt[1], pt[2]])
            p_transformed = R @ p + translation
            transformed_points.append(tuple(p_transformed))

        new_cloud = pc2.create_cloud_xyz32(pointCloud.header, transformed_points)
        new_cloud.header.frame_id = target_frame
        new_cloud.header.stamp = stamp

        return new_cloud, 1, "Success"

    except Exception as e:
        print("Failed to transform point cloud:", e)
        return None, -51, "Failed to transform point cloud"


def transform_bBox(tf_buffer, bBox, stamp, ref_frame: str):

    min_pose = Pose()
    min_pose.position = bBox.min_point
    min_pose.orientation.w = 1.0

    max_pose = Pose()
    max_pose.position = bBox.max_point
    max_pose.orientation.w = 1.0

    min_trans, err, msg = transform_pose(tf_buffer, min_pose, stamp, ref_frame)
    if min_trans is None:
        return None, err, msg

    max_trans, err, msg = transform_pose(tf_buffer, max_pose, stamp, ref_frame)
    if max_trans is None:
        return None, err, msg

    bBox.min_point = min_trans.position
    bBox.max_point = max_trans.position

    return bBox, 1, "Success"
