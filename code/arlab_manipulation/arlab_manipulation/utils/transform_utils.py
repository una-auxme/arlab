#!/usr/bin/env python3
"""
voxel_utils.py
---------------

Author: Sofia Öttl
Date: 2025-10-22
"""

import rclpy
import rclpy.duration
import numpy as np

from geometry_msgs.msg import Pose, PoseStamped
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
import sensor_msgs_py.point_cloud2 as pc2

target_frame = 'base_link'

def transform_pose(tf_buffer, pose, stamp, ref_frame):
    """Transform a Pose into the target frame safely."""
    try:
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = ref_frame
        pose_stamped.header.stamp = stamp
        pose_stamped.pose = pose

        transform_stamped = tf_buffer.lookup_transform(
            target_frame,
            ref_frame,
            stamp,
            timeout=rclpy.duration.Duration(seconds=1.0)
        )
        transformed_pose = do_transform_pose(pose_stamped, transform_stamped)
        error_code = 1
        msg = "Success"
        return transformed_pose.pose, error_code, msg

    except Exception as e:
        print("Failed to transform pose")
        error_code = -51
        msg = "Failed to transform pose"
        return error_code, msg


def transform_pointCloud(tf_buffer, pointCloud, stamp, ref_frame):
    """Transform a PointCloud2 safely into the target frame."""
    try:
        transformed_points = []

        transform_stamped = tf_buffer.lookup_transform(
            target_frame,
            ref_frame,
            stamp,
            timeout=rclpy.duration.Duration(seconds=1.0)
        )

        t = transform_stamped.transform.translation
        q = transform_stamped.transform.rotation

        # Build rotation matrix from quaternion
        qw, qx, qy, qz = q.w, q.x, q.y, q.z
        R = np.array([
            [1-2*qy*qy-2*qz*qz, 2*qx*qy-2*qz*qw, 2*qx*qz+2*qy*qw],
            [2*qx*qy+2*qz*qw, 1-2*qx*qx-2*qz*qz, 2*qy*qz-2*qx*qw],
            [2*qx*qz-2*qy*qw, 2*qy*qz+2*qx*qw, 1-2*qx*qx-2*qy*qy]
        ])
        translation = np.array([t.x, t.y, t.z])

        # Transform each point
        for pt in pc2.read_points(pointCloud, field_names=["x", "y", "z"], skip_nans=True):
            p = np.array([pt[0], pt[1], pt[2]])
            p_transformed = R @ p + translation
            transformed_points.append((p_transformed[0], p_transformed[1], p_transformed[2]))

        # Build new PointCloud2
        new_cloud = pc2.create_cloud_xyz32(pointCloud.header, transformed_points)
        new_cloud.header.frame_id = target_frame
        new_cloud.header.stamp = stamp
        error_code = 1
        msg = "Success"
        return new_cloud, error_code, msg

    except Exception as e:
        print("Failed to transform pose")
        error_code = -51
        msg = "Failed to transform pose"
        return error_code, msg

def transform_bBox(tf_buffer, bBox, stamp, ref_frame):
    """Transform bounding box points safely."""
    try:
        min_pose = Pose()
        min_pose.position = bBox.min_point
        min_pose.orientation.w = 1.0

        max_pose = Pose()
        max_pose.position = bBox.max_point
        max_pose.orientation.w = 1.0

        min_trans = transform_pose(tf_buffer, min_pose, stamp, ref_frame)
        max_trans = transform_pose(tf_buffer, max_pose, stamp, ref_frame)

        bBox.min_point = min_trans.position
        bBox.max_point = max_trans.position
        error_code = 1
        msg = "Success"
        return bBox, error_code, msg

    except Exception as e:
        print("Failed to transform pose")
        error_code = -51
        msg = "Failed to transform pose"
        return error_code, msg
