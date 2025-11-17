#!/usr/bin/env python3
"""
voxel_utils.py
---------------

Author: Sofia Öttl
Date: 2025-10-22
"""

import numpy as np
import matplotlib.pyplot as plt

# Transfer pointcloud into voxel map
def pointcloud_to_voxel_map(tf_buffer, pointcloud, voxel_size=0.01):

    if pointcloud is None or len(pointcloud) == 0:
        tf_buffer.get_logger().warn("Empty pointcloud, cannot create voxel map.")
        return None

    # Move Pointcloud in positive Area
    min_coords = np.min(pointcloud, axis=0)
    shifted = pointcloud - min_coords

    # Dimension of Voxelmap
    dims = np.ceil(np.max(shifted, axis=0) / voxel_size).astype(int)
    voxel_map = np.zeros(dims, dtype=np.uint8)

    # Calculate Indices
    indices = np.floor(shifted / voxel_size).astype(int)
    voxel_map[indices[:,0], indices[:,1], indices[:,2]] = 1

    return voxel_map, min_coords, voxel_size

# Find a placing point using the voxel map
def find_placing_area(self, voxel_map, bbox, gripper_margin=0.02):
    """
    Find a free area in the voxel_map where the object + gripper fits.
    Returns the voxel coordinates (x,y,z) of the bottom-left corner, or [0,0,0] if none.
    """

    voxel_size = self.voxel_size
    shape_x = int(np.ceil((bbox.size_x + gripper_margin) / voxel_size))
    shape_y = int(np.ceil((bbox.size_y + gripper_margin) / voxel_size))
    shape_z = int(np.ceil((bbox.size_z + gripper_margin) / voxel_size))

    dims = voxel_map.shape

    # Slide a window over the voxel map
    for x in range(dims[0] - shape_x):
        for y in range(dims[1] - shape_y):
            for z in range(dims[2] - shape_z):
                sub = voxel_map[x:x+shape_x, y:y+shape_y, z:z+shape_z]
                if np.all(sub == 0):
                    # Found free area
                    return [x, y, z]

    # fallback
    self.get_logger().warn("No free area found for object + gripper")
    return [0, 0, 0]

# Show Voxel Map
def visualize_voxel_map(self, voxel_map):
    """
    Visualizes the voxel map using matplotlib 3D scatter plot.
    """
    filled = np.argwhere(voxel_map == 1)
    if filled.size == 0:
        print("Voxelmap is empty.")
        return

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(filled[:,0], filled[:,1], filled[:,2], c='blue', marker='s', s=10)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.show()
