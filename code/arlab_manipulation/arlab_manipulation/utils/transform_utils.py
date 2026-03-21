#!/usr/bin/env python3
"""
TF Transform Utilities for Poses, Point Clouds, and Bounding Boxes.

Provides ROS-independent functions to transform geometry messages and
point clouds to a target frame ('base_link'). Includes error handling
and status codes for use in pipelines.

Maintainer:
    Sofia Öttl <sofia.oettl@uni-a.de>

Note:
    These functions are experimental and have not been fully tested.
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
    """Transform a Pose to the target frame using a TF2 buffer.

    Converts a Pose into a PoseStamped, looks up the transform from the
    reference frame to the target frame, and applies it. Returns status
    codes and message for error handling.

    Args:
        tf_buffer: TF2 buffer used for looking up transforms.
        pose: Pose to transform.
        stamp: ROS 2 timestamp for transform lookup.
        ref_frame: Frame in which the input pose is defined.

    Returns:
        transformed_pose: Pose in the target frame, or None if failed.
        status_code: 1 for success, negative for failure.
        message: Description of the result.

    Notes:
        - Timeout is set to 1 second for transform lookup.
        - Unsuccessful transform returns None and status -51.
        - Preserves orientation unless transform fails.
    """
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
    """Transform a PointCloud to the target frame using a TF2 buffer.

    Applies rotation and translation from the transform to all points in
    the cloud. Returns a new PointCloud with updated frame and timestamp.

    Args:
        tf_buffer: TF2 buffer used for transform lookup.
        pointCloud: sensor_msgs/PointCloud2 message to transform.
        stamp: ROS 2 timestamp for transform.
        ref_frame: Frame in which the point cloud is defined.

    Returns:
        new_cloud: Transformed PointCloud2 message, or None if failed.
        status_code: 1 for success, negative for failure.
        message: Description of the result.

    Notes:
        - Constructs rotation matrix from quaternion manually.
        - Handles NaNs by skipping them.
        - Unsuccessful transform returns None and status -51.
        - Experimental: not tested in real-world scenarios.
    """
    try:
        transformed_points = []

        transform_stamped = tf_buffer.lookup_transform(target_frame, ref_frame, stamp, timeout=rclpy.duration.Duration(seconds=1.0))

        t = transform_stamped.transform.translation
        q = transform_stamped.transform.rotation

        # Rotation matrix from quaternion (w, x, y, z)
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
    """Transform a bounding box to the target frame using TF2.

    Transforms the min and max points of the bounding box individually.
    Updates the bounding box in place with transformed positions.

    Args:
        tf_buffer: TF2 buffer used for transform lookup.
        bBox: Bounding box object with min_point and max_point attributes.
        stamp: ROS 2 timestamp for transform.
        ref_frame: Frame in which the bounding box is defined.

    Returns:
        bBox: Updated bounding box in the target frame, or None if failed.
        status_code: 1 for success, negative for failure.
        message: Description of the result.

    Notes:
        - Uses transform_pose internally; failure of either corner aborts.
        - Orientation of min/max poses is identity (w=1.0) since only positions are transformed.
        - Experimental: not tested in real-world scenarios.
    """
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
