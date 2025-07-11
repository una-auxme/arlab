#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, PointStamped, Pose
from std_msgs.msg import Float64, String
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import cv2
import numpy as np
import rclpy.time

from arlab_common_interfaces.srv import GrippingForce


class PosePublisher(Node):
    def __init__(self):
        super().__init__('PosePublisher')

        # Publisher: Pose für MoveIt und Greifkraft
        self.goal_pub = self.create_publisher(Pose, '/goal_pose', 10)
        self.gripforce_pub = self.create_publisher(Float64, '/grip_force', 10)
        self.cmd_pub = self.create_publisher(String, '/cmd', 10)

        # Subscriber for new object
        self.test_sub = self.create_subscription(String, '/test_data', self.test_callback, 10)
        self.knowledgebase_sub = self.create_subscription(PointCloud2, '/point_cloud', self.pointcloud_callback, 10)

        # Service Client für Greifkraft
        self.client = self.create_client(GrippingForce, 'GetGrippingForce')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for GrippingForce service...')

        self.req = GrippingForce.Request()
        self.force = 5.0
        self.object_name = "default"
        self.cmd = "home"


    def test_callback(self, msg: String):
        data = msg.data.strip()

        if ':' in data:
            cmd, obj = data.split(':', 1)
            self.cmd = cmd.lower().strip()
            self.object_name = obj.strip()
            self.get_logger().info(f"Object: {self.object_name} | Command: {cmd}")
        else:
            self.get_logger().warn(f"Invalid message: {data}")

        self.req.object_name = self.object_name
        future = self.client.call_async(self.req)
        future.add_done_callback(self.handle_service_response)
    
    
    def pointcloud_callback(self, msg: String):

        # transform
        points_base = []

        #T = ...  get from calibration (future me problem)

        T = np.array([  # dummy T
            [1.0, 0.0, 0.0, 0.0], 
            [0.0, 1.0, 0.0, 0.0],  
            [0.0, 0.0, 1.0, 0.0],  
            [0.0, 0.0, 0.0, 1.0]   
        ])

        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            point_cam = np.array([p[0], p[1], p[2]])         
            point_base = T @ point_cam                           
            points_base.append(point_base[:3]) 

        self.point_cloud_base = np.array(points_base)
        self.get_logger().info(f"Transformed {len(points_base)} cam to base points.")

        # gripping point calculation (center of object)
        self.gripping_point_pos = np.mean(self.point_cloud_base)
        self.gripping_point_orient = ([0, 0, 0, 1])
        self.gripping_pose = np.concatenate([self.gripping_point_pos, self.gripping_point_orient])#

    
    def handle_service_response(self, future):
        try:
            response = future.result()
            self.force = response.grip_force
            self.get_logger().info(f"Grip force for '{self.object_name}': {self.force:.1f} N")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            self.force = 5.0

        self.publish_goal()


    def publish_goal(self):
        """
        gripping_pose = Pose()
        gripping_pose.position.x = self.gripping_point[0]
        gripping_pose.position.y = self.gripping_point[1]
        gripping_pose.position.z = self.gripping_point[2]
        gripping_pose.orientation.x = self.gripping_point[3]
        gripping_pose.orientation.y = self.gripping_point[4]
        gripping_pose.orientation.z = self.gripping_point[5]
        gripping_pose.orientation.w = self.gripping_point[6]
        self.goal_pub.publish(gripping_pose)
        """

        gripping_pose = Pose()  #dummy pose
        gripping_pose.position.x = 0.372
        gripping_pose.position.y = 0.124
        gripping_pose.position.z = 0.621
        gripping_pose.orientation.x = 0.999
        gripping_pose.orientation.y = 0.041
        gripping_pose.orientation.z = 0.006
        gripping_pose.orientation.w = 0.004
        self.goal_pub.publish(gripping_pose)

        force_msg = Float64()
        force_msg.data = self.force
        self.gripforce_pub.publish(force_msg)

        cmd_msg = String()
        cmd_msg.data = self.cmd #pick, place, open, close, move, home
        self.cmd_pub.publish(cmd_msg)

        pos = gripping_pose.position
        orient = gripping_pose.orientation
        self.get_logger().info( 
            f"Published gripping pose: Position x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f} | Orientation: ox={orient.x:.3f}, oy={orient.y:.3f}, oz={orient.z:.3f}, ow={orient.w:.3f} | Grip force: {self.force:.1f} N | Command: {self.cmd}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = PosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
