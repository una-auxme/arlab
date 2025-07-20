#!/usr/bin/env python3
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

class Orchestrator(Node):
    def __init__(self):
        super().__init__('Orchestrator')

        # Publisher for C++ (MoveItNode)
        self.goal_pub = self.create_publisher(Pose, '/goalpose', 10)
        self.gripforce_pub = self.create_publisher(Float64, '/gripforce', 10)
        self.cmd_pub = self.create_publisher(String, '/cmd', 10)

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
        # 2D-Infos from BoundingBox2D: center (x,y), size_x, size_y

        # long side = main orientation ⇒ calculate angle
        if msg.size_x >= msg.size_y:
            angle = 0.0         # horicontal bbox
        else:
            angle = np.pi / 2   # vertical bbox

        # Orientation in quaternion (rotation around Z-axis)
        qz = np.sin(angle / 2.0)
        qw = np.cos(angle / 2.0)
        self.bounding_box_orientation = [0.0, 0.0, qz, qw]


    def pointcloud_callback(self, msg: PointCloud2):
        # goal coorinates
        base_frame = 'base_link'
        cam_frame = 'camera_link' # change in world frame when slam is working

        points_base = []

        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            # point in camera frame
            point_cam = PointStamped()
            point_cam.header.stamp = msg.header.stamp
            point_cam.header.frame_id = cam_frame
            point_cam.point.x = p[0]
            point_cam.point.y = p[1]
            point_cam.point.z = p[2]

            # transform to base frame
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
        try:
            response = future.result()
            self.force = response.gripforce
            self.get_logger().info(f"Grip force for '{self.objectname}': {self.force:.1f} N")
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
            f"Published gripping pose: Pos x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f} | Orient: ox={orient.x:.3f}, oy={orient.y:.3f}, oz={orient.z:.3f}, ow={orient.w:.3f} | Grip force: {self.force:.1f} N | Cmd: {self.cmd}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = Orchestrator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
