#!/usr/bin/env python3
"""
Create Octomap (binary voxelmap) from PointCloud2 (Python-only)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from octomap_msgs.msg import Octomap
from arlab_common_interfaces.srv import CreateVoxelmap
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2

def pointcloud2_to_xyz_array(pointcloud_msg):
    """Convert PointCloud2 to Nx3 numpy array"""
    points = []
    for p in pc2.read_points(pointcloud_msg, skip_nans=True):
        points.append([p[0], p[1], p[2]])
    return np.array(points, dtype=np.float32)

class CreateVoxelmapNode(Node):

    def __init__(self):
        super().__init__("create_voxelmap")
        self.srv = self.create_service(
            CreateVoxelmap, "CreateVoxelmap", self.callback
        )
        self.get_logger().info("CreateVoxelmap service ready")

    def callback(self, request, response):
        voxel_size = request.voxel_size
        points = pointcloud2_to_xyz_array(request.pointcloud)

        if points.size == 0:
            self.get_logger().warn("Empty pointcloud, cannot create voxel map.")
            return response

        # Shift pointcloud to positive coordinates
        min_coords = np.min(points, axis=0)
        shifted = points - min_coords

        # Compute voxel grid dimensions
        dims = np.ceil(np.max(shifted, axis=0) / voxel_size).astype(int)

        # Create voxel grid
        voxel_map = np.zeros(dims, dtype=np.uint8)
        indices = np.floor(shifted / voxel_size).astype(int)
        voxel_map[indices[:,0], indices[:,1], indices[:,2]] = 1

        # Convert voxel grid to Octomap message
        octo_msg = Octomap()
        octo_msg.header = Header()
        octo_msg.header.stamp = self.get_clock().now().to_msg()
        octo_msg.header.frame_id = "map"
        octo_msg.binary = True
        octo_msg.id = "OcTree"
        octo_msg.resolution = voxel_size
        octo_msg.data = voxel_map.flatten().tolist()  # binary octomap (0/1 values)

        response.voxel_map = octo_msg
        self.get_logger().info(f"Voxelmap ({dims[0]}x{dims[1]}x{dims[2]}) created")
        return response

def main(args=None):
    rclpy.init(args=args)
    node = CreateVoxelmapNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
