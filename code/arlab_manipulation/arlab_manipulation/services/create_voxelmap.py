#!/usr/bin/env python3

"""
GetGrippingParameter.py
---------------------

ROS2 Node 'GetGrippingParameter' providing a service to return recommended gripping
forces for different objects.

Author: Sofia Öttl
Date: 2025-08-24

"""

import rclpy
from rclpy.node import Node

import numpy as np

from arlab_common_interfaces.srv import CreateVoxelmap

class create_voxelmap(Node):

    def __init__(self):
        super().__init__("CreateVoxelmap")
        self.srv = self.create_service(
            CreateVoxelmap, "CreateVoxelmap",
            self.callback
            )

    def callback(self, request, response):
        tf_buffer = request.tf_buffer
        pointcloud =  request.pointcloud
        voxel_size = request.voxel_size

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


def main(args=None):
    rclpy.init(args=args)
    node =  create_voxelmap()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
