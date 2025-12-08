#!/usr/bin/env python3
"""
octomap_utils.py
----------------------

Author: Sofia Öttl
Date: 2025-12-08

"""

import numpy as np
from geometry_msgs.msg import Pose

def detect_shelf_floor(octo_tree):
    """
    Finds the lowest occupied layer in the octomap (assumed to be shelf floor)
    Returns floor_z in world coordinates
    """
    min_bb = octo_tree.getMetricMin()
    max_bb = octo_tree.getMetricMax()
    step = octo_tree.getResolution()

    x_vals = np.arange(min_bb[0], max_bb[0], step)
    y_vals = np.arange(min_bb[1], max_bb[1], step)
    floor_candidates = []

    for x in x_vals:
        for y in y_vals:
            for z in np.arange(min_bb[2], max_bb[2], step):
                node = octo_tree.search(x, y, z)
                if node is not None and node.is_occupied():
                    floor_candidates.append(z)
                    break

    if not floor_candidates:
        return min_bb[2]
    return np.median(floor_candidates)

def is_box_free(octo_tree, center, size):
    """
    Check if a box at 'center' with 'size' (dx, dy, dz) is free in the OctoMap
    center: [x, y, z] in world coordinates
    size: [dx, dy, dz] in meters
    """
    step = octo_tree.getResolution()
    dx, dy, dz = size
    nx = int(np.ceil(dx / step))
    ny = int(np.ceil(dy / step))
    nz = int(np.ceil(dz / step))

    start_x = center[0] - dx / 2
    start_y = center[1] - dy / 2
    start_z = center[2] - dz / 2

    for ix in range(nx):
        for iy in range(ny):
            for iz in range(nz):
                x = start_x + ix * step
                y = start_y + iy * step
                z = start_z + iz * step
                node = octo_tree.search(x, y, z)
                if node is not None and node.is_occupied():
                    return False
    return True

def find_placing_area(octo_tree, bbox, margin=0.02, lift=0.01, offset_x=0.05, offset_y=0.05):
    """
    Finds a free placement area directly above the shelf floor,
    taking into account the space needed for the robot gripper.

    bbox: object with size_x, size_y, size_z in meters
    margin: extra safety margin around object in XY
    lift: extra clearance above the floor in meters
    offset_x: extra space needed on the sides for the gripper (meters)
    offset_y: extra space needed in front of the object for the gripper (meters)
    Returns Pose relative to shelf floor (z=0)
    """
    floor_z = detect_shelf_floor(octo_tree)
    step = octo_tree.getResolution()

    search_size_x = bbox.size_x + margin + 2*offset_x  # offset for MIA hand
    search_size_y = bbox.size_y + margin + offset_y    # offset for MIA hand
    search_size_z = bbox.size_z

    min_bb = octo_tree.getMetricMin()
    max_bb = octo_tree.getMetricMax()

    x_vals = np.arange(min_bb[0], max_bb[0] - search_size_x, step)
    y_vals = np.arange(min_bb[1], max_bb[1] - search_size_y, step)

    z_center = floor_z + search_size_z / 2 + lift

    for x in x_vals:
        for y in y_vals:
            center = [x + search_size_x/2, y + search_size_y/2, z_center]
            if is_box_free(octo_tree, center, [search_size_x, search_size_y, search_size_z]):
                pose = Pose()
                pose.position.x = center[0]
                pose.position.y = center[1]
                pose.position.z = center[2] - floor_z  # z is relative to shelf floor
                return pose

    print("Warning: No free area found on floor for object + gripper")
    return Pose()
