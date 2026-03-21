#!/usr/bin/env python3
"""
Octomap Utilities (Python-only)

This module provides ROS-independent utilities for working with 3D octomap
data represented as numpy arrays. Functions include detecting shelf floors,
checking box occupancy, and finding free placement areas.

Maintainer:
    Sofia Öttl <sofia.oettl@uni-a.de>

Note:
    These functions are experimental and have not been fully tested.
"""

import numpy as np
from geometry_msgs.msg import Pose


def detect_shelf_floor(octo_data, resolution=0.01):
    """Estimate the lowest occupied layer in the octomap as the shelf floor.

    Scans each vertical column to find the first occupied voxel. Returns the median
    of all first-occupied voxels to reduce the influence of outliers or noise.

    Args:
        octo_data: 3D numpy array representing occupancy (True=occupied, False=free)
        resolution: Voxel size in meters.

    Returns:
        floor_z: Estimated z-coordinate of the shelf floor in world coordinates.

    Notes:
        - Experimental function: **not fully tested**.
        - If no occupied voxels are found, returns a safe default floor height.
        - Median ensures robustness against isolated high/low voxels.
    """

    floor_candidates = []
    for ix in range(octo_data.shape[0]):
        for iy in range(octo_data.shape[1]):
            column = octo_data[ix, iy, :]
            occupied = np.where(column)[0]
            if occupied.size > 0:
                floor_candidates.append(occupied[0] * resolution)

    if not floor_candidates:
        # Default safe height when map is empty
        return 0.75
    return np.median(floor_candidates)


def is_box_free(octo_data, center, size, resolution=0.01):
    """Check whether a 3D box at the given location is free of obstacles.

    Iterates over voxels inside the box. Returns False if any occupied voxel
    is detected. This is used for safe collision checking before placing an object.

    Args:
        octo_data: 3D numpy array representing occupancy.
        center: [x, y, z] coordinates of the box center in world frame.
        size: [dx, dy, dz] dimensions of the box in meters.
        resolution: Voxel size in meters.

    Returns:
        True if all voxels are free, False otherwise.

    Notes:
        - Experimental function: **not fully tested**.
        - Out-of-bounds voxels are assumed free to avoid false negatives.
        - Conservative check ensures no collisions with unknown areas.
    """

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
                        # Occupied voxel found → placement not possible
                        return False
                except IndexError:
                    # Outside map bounds assumed free
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
    """Search for a free placement area above the shelf floor.

    Iterates over candidate positions and returns the first valid placement
    that fits the object's bounding box without collisions.

    Args:
        octo_data: 3D numpy array representing occupancy.
        bbox: Object with attributes size_x, size_y, size_z in meters.
        margin: Extra clearance around object for safety.
        lift: Height to lift the object above the floor to avoid collisions.
        offset_x: X-offset for search expansion.
        offset_y: Y-offset for search expansion.
        resolution: Voxel size in meters.

    Returns:
        pose: geometry_msgs.msg.Pose representing placement location and orientation.
        status_code: int (1=success, negative values indicate failure).
        message: str describing the result.

    Notes:
        - Experimental function: **not fully tested**.
        - Returns a safe default pose if bbox is None or no free space is found.
        - Placement z-coordinate is relative to the detected shelf floor.
        - Only the first valid location is returned to minimize computation.
    """
    if bbox is None:
        print("Bbox is empty")
        pose = Pose()
        return pose, -50, "BoundingBox is empty"

    # Determine the shelf floor for placement reference
    floor_z = detect_shelf_floor(octo_data, resolution)

    # Compute the total search box size including margins and offsets
    search_size_x = bbox.size_x + margin + 2 * offset_x
    search_size_y = bbox.size_y + margin + offset_y
    search_size_z = bbox.size_z

    # Define search boundaries (world coordinates)
    min_bb = [0.0, 0.0, floor_z]
    max_bb = [1.0, 1.0, floor_z + 1.0]

    # Candidate positions along x and y
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
                # Valid placement found; orientation identity (no rotation)
                pose = Pose()
                pose.position.x = center[0]
                pose.position.y = center[1]
                pose.position.z = center[2] - floor_z
                pose.orientation.w = 1.0
                return pose, 1, "Success"

    # No free placement found; return safe default
    print("No free placing area found")
    pose = Pose()
    pose.position.x = 0.5
    pose.position.y = 0.5
    pose.position.z = 0.0
    pose.orientation.w = 1.0
    return pose, -49, "No free placing area found"
