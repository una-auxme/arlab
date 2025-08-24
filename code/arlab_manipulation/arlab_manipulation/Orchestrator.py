#!/usr/bin/env python3

"""
Orchestrator.py
---------------

ROS2 Node 'Orchestrator' that subscribes to object, point cloud, and bounding box data,
queries the gripping force service, computes gripping poses, and publishes orchestrator data.

Author: Sofia Öttl
Date: 2025-08-24

"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, PointStamped, Pose
from std_msgs.msg import Float64, String
from sensor_msgs.msg import PointCloud2
from vision_msgs.msg import BoundingBox2D
import sensor_msgs_py.point_cloud2 as pc2
import cv2
import numpy as np
import rclpy.time
import tf2_ros
import tf2_geometry_msgs

from arlab_common_interfaces.srv import GrippingForce
from arlab_common_interfaces.msg import OrchestratorData

class Orchestrator(Node):
    """ROS2 Node for orchestrating robotic manipulation.

    Subscribes to test commands, point cloud data, and bounding box data.
    Calls a GrippingForce service to obtain recommended gripping forces.
    Computes gripping poses and publishes OrchestratorData messages.

    Attributes:
        data_publisher (rclpy.Publisher): Publisher for OrchestratorData messages.
        test_sub (rclpy.Subscription): Subscription to test command strings.
        pointcloud_sub (rclpy.Subscription): Subscription to point cloud data.
        boundingbox_sub (rclpy.Subscription): Subscription to bounding box data.
        client (rclpy.ServiceClient): Client for GrippingForce service.
        tf_buffer (tf2_ros.Buffer): TF2 buffer for frame transformations.
        tf_listener (tf2_ros.TransformListener): TF2 listener for transforms.
        req (GrippingForce.Request): Service request object.
        force (float): Current gripping force in Newtons.
        objectname (str): Current object name.
        cmd (str): Current command type (pick, place, etc.).
    """


    def __init__(self):
        """Initialize the Orchestrator node, publishers, subscribers, service client, and TF2."""
        super().__init__('Orchestrator')

        # Publisher for C++ (MoveItNode)
        #self.goal_pub = self.create_publisher(Pose, '/goalpose', 10)
        #self.gripforce_pub = self.create_publisher(Float64, '/gripforce', 10)
        #self.cmd_pub = self.create_publisher(String, '/cmd', 10)

        self.data_publisher = self.create_publisher(OrchestratorData, '/orchestrator_data', 10)

        # Action Server for ManipulationCommand
        # ---- string                       command_type
        # ---- int64                        target_entityid
        # ---- geometry_msgs/Pose           target_pose

        # Service client for getShape
        # ---- bool                         has_pointcloud
        # ---- sensor_msgs/PointCloud2      pointcloud
        # ---- bool                         has_boundingbox2d
        # ---- vision_msgs/BoundingBox2D    boundingbox2d

        # Service client for getEntity
        # ---- builtin_interfaces/Time      stamp
        # ---- string                       description
        # ---- geometry_msgs/Pose           pose
        # ---- string                       pose_reference_frame
        # ---- EntityFurniture              furniture
        # ---- EntityHuman                  human
        # ---- EntityPickable               pickable

        # Dummy-subscribtion for object, pointcloud & boundingbox
        self.test_sub = self.create_subscription(String, '/testdata', self.test_callback, 10) #pickable & command_tpye
        self.pointcloud_sub = self.create_subscription(PointCloud2, '/pointcloud', self.pointcloud_callback, 10)
        self.boundingbox_sub = self.create_subscription(BoundingBox2D, '/boundingbox2d', self.bbox_callback, 10)

        # Service client for gripping force
        self.client = self.create_client(GrippingForce, 'GetGrippingForce')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for GrippingForce service...')

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Inits
        self.req = GrippingForce.Request()
        self.force = 5.0
        self.objectname = "default"
        self.cmd = "home"


    def test_callback(self, msg: String):
        """Handle incoming test command messages.

        Parses messages of the form 'command:object' and calls the GrippingForce service.

        Args:
            msg (String): Incoming ROS String message containing the command and object.
        """
        data = msg.data.strip()

        if ':' in data:
            cmd, obj = data.split(':', 1)
            self.cmd = cmd.lower().strip()
            self.objectname = obj.strip()
            self.get_logger().info(f"Object: {self.objectname} | Command: {cmd}")
        else:
            self.get_logger().warn(f"Invalid message: {data}")

        self.req.objectname = self.objectname
        future = self.client.call_async(self.req)
        future.add_done_callback(self.handle_service_response)


    def bbox_callback(self, msg: BoundingBox2D):
        """Handle incoming BoundingBox2D messages and compute orientation.

        Determines the main orientation based on the longer side of the bounding box.

        Args:
            msg (BoundingBox2D): Incoming bounding box message.
        """
        # Determine angle from bounding box dimensions
        if msg.size_x >= msg.size_y:
            angle = 0.0         # horicontal bbox
        else:
            angle = np.pi / 2   # vertical bbox

        # Orientation in quaternion (rotation around Z-axis)
        qz = np.sin(angle / 2.0)
        qw = np.cos(angle / 2.0)
        self.bounding_box_orientation = [0.0, 0.0, qz, qw]


    def pointcloud_callback(self, msg: PointCloud2):
        """Handle incoming PointCloud2 messages and compute gripping pose.

        Transforms all points to the base frame, calculates the mean position,
        and combines with bounding box orientation to define the gripping pose.

        Args:
            msg (PointCloud2): Incoming point cloud message.
        """
        base_frame = 'base_link'
        cam_frame = 'camera_link' # Change in world frame when slam is working

        points_base = []

        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            point_cam = PointStamped()
            point_cam.header.stamp = msg.header.stamp
            point_cam.header.frame_id = cam_frame
            point_cam.point.x = p[0]
            point_cam.point.y = p[1]
            point_cam.point.z = p[2]

            # Transform points to base frame
            point_base = self.tf_buffer.transform(
                point_cam,
                base_frame,
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            points_base.append([point_base.point.x, point_base.point.y, point_base.point.z])

        self.get_logger().info(f"Transformed pointcloud from '{cam_frame}' to '{base_frame}'")

        # Gripping position
        self.gripping_point_pos = np.mean(points_base, axis=0)

        # Gripping orientation
        self.gripping_point_orient = self.bounding_box_orientation #([0, 0, 0, 1])

        # Gripping pose (position&orientation)
        self.gripping_pose = np.concatenate([self.gripping_point_pos, self.gripping_point_orient])


    def handle_service_response(self, future):
        """Handle the response from the GrippingForce service.

        Updates the internal gripping force and publishes the goal.

        Args:
            future (rclpy.task.Future): Future object returned by service call.
        """
        try:
            response = future.result()
            self.force = response.gripforce
            self.get_logger().info(f"Grip force for '{self.objectname}': {self.force:.1f} N")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            self.force = 5.0

        self.publish_goal()


    def publish_goal(self):
        """Publish the gripping pose, force, and command as OrchestratorData."""
        #gripping_pose = Pose()
        #gripping_pose.position.x = self.gripping_point[0]
        #gripping_pose.position.y = self.gripping_point[1]
        #gripping_pose.position.z = self.gripping_point[2]
        #gripping_pose.orientation.x = self.gripping_point[3]
        #gripping_pose.orientation.y = self.gripping_point[4]
        #gripping_pose.orientation.z = self.gripping_point[5]
        #gripping_pose.orientation.w = self.gripping_point[6]
        #self.goal_pub.publish(gripping_pose)
        
        # Testdata
        gripping_pose = Pose()
        gripping_pose.position.x = 0.372
        gripping_pose.position.y = 0.124
        gripping_pose.position.z = 0.621
        gripping_pose.orientation.x = 0.999
        gripping_pose.orientation.y = 0.041
        gripping_pose.orientation.z = 0.006
        gripping_pose.orientation.w = 0.004

        msg = OrchestratorData()
        msg.pose = gripping_pose
        msg.grip_force = Float64()
        msg.grip_force.data = self.force
        msg.cmd = String()
        msg.cmd.data = self.cmd #pick, place, open, close, move, home

        self.data_publisher.publish(msg)

        pos = gripping_pose.position
        orient = gripping_pose.orientation
        self.get_logger().info(
            f"Published gripper goal: Position x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f} | "
            f"Orientation ox={orient.x:.3f}, oy={orient.y:.3f}, oz={orient.z:.3f}, ow={orient.w:.3f} | "
            f"Grip force: {self.force:.1f} N | Command: {self.cmd}"
        )

def main(args=None):
    """Initialize the Orchestrator node and spin."""
    rclpy.init(args=args)
    node = Orchestrator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
