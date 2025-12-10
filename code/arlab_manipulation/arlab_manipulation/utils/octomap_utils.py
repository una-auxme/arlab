#!/usr/bin/env python3
"""
octomap_utils.py
----------------------

Author: Sofia Öttl
Date: 2025-12-10

Python-only Octomap utilities
"""

import numpy as np
from geometry_msgs.msg import Pose


def detect_shelf_floor(octo_data, resolution=0.01):
    """
    Find the lowest occupied layer in the octomap (assumed to be shelf floor)
    octo_data: numpy 3D array of occupancy (True=occupied, False=free)
    resolution: voxel size in meters
    Returns floor_z in world coordinates
    """
    if octo_data is None or octo_data.size == 0:
        return 0.75

    floor_candidates = []
    for ix in range(octo_data.shape[0]):
        for iy in range(octo_data.shape[1]):
            column = octo_data[ix, iy, :]
            occupied = np.where(column)[0]
            if occupied.size > 0:
                floor_candidates.append(occupied[0] * resolution)

    if not floor_candidates:
        return 0.75
    return np.median(floor_candidates)


def is_box_free(octo_data, center, size, resolution=0.01):
    """
    Check if a box at 'center' with 'size' (dx, dy, dz) is free
    octo_data: numpy 3D array
    center: [x, y, z] in world coordinates
    size: [dx, dy, dz] in meters
    resolution: voxel size in meters
    """
    if octo_data is None or octo_data.size == 0:
        return True

    dx, dy, dz = size
    nx = int(np.ceil(dx / resolution))
    ny = int(np.ceil(dy / resolution))
    nz = int(np.ceil(dz / resolution))

    start_x = int(np.floor(center[0] / resolution - nx / 2))
    start_y = int(np.floor(center[1] / resolution - ny / 2))
    start_z = int(np.floor(center[2] / resolution - nz / 2))

    for ix in range(start_x, start_x + nx):
        for iy in range(start_y, start_y + ny):
            for iz in range(start_z, start_z + nz):
                if ix < 0 or iy < 0 or iz < 0:
                    continue
                try:
                    if octo_data[ix, iy, iz]:
                        return False
                except IndexError:
                    continue
    return True


def find_placing_area(
    octo_data,
    bbox,
    margin=0.02,
    lift=0.01,
    offset_x=0.05,
    offset_y=0.05,
    resolution=0.01,
):
    """
    Find a free placement area above the shelf floor (Python-only version).
    octo_data: 3D numpy array
    bbox: object with size_x, size_y, size_z in meters
    Returns: geometry_msgs.msg.Pose
    """
    if bbox is None:
        print("Warning: bbox is None, returning default pose")
        pose = Pose()
        pose.position.x = 0.5
        pose.position.y = 0.5
        pose.position.z = 0.0
        pose.orientation.w = 1.0
        return pose

    floor_z = detect_shelf_floor(octo_data, resolution)

    search_size_x = bbox.size_x + margin + 2 * offset_x
    search_size_y = bbox.size_y + margin + offset_y
    search_size_z = bbox.size_z

    min_bb = [0.0, 0.0, floor_z]
    max_bb = [1.0, 1.0, floor_z + 1.0]

    x_vals = np.arange(min_bb[0], max_bb[0] - search_size_x, resolution)
    y_vals = np.arange(min_bb[1], max_bb[1] - search_size_y, resolution)
    z_center = floor_z + search_size_z / 2 + lift

    for x in x_vals:
        for y in y_vals:
            center = [x + search_size_x / 2, y + search_size_y / 2, z_center]
            if is_box_free(
                octo_data,
                center,
                [search_size_x, search_size_y, search_size_z],
                resolution,
            ):
                pose = Pose()
                pose.position.x = center[0]
                pose.position.y = center[1]
                pose.position.z = center[2] - floor_z
                pose.orientation.w = 1.0
                return pose

    print("Warning: No free area found, returning default pose")
    pose = Pose()
    pose.position.x = 0.5
    pose.position.y = 0.5
    pose.position.z = 0.0
    pose.orientation.w = 1.0
    return pose
