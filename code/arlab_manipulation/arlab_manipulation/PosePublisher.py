#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import Float64, String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

from arlab_common_interfaces.srv import GrippingForce

from tf2_ros import TransformListener, Buffer
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
import rclpy.time


class PosePublisher(Node):
    def __init__(self):
        super().__init__('PosePublisher')

        # Publisher: Pose für MoveIt und Greifkraft
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.gripforce_pub = self.create_publisher(Float64, '/grip_force', 10)

        # Subscriber: Objektname, Maske, Depth, CameraInfo
        self.object_sub = self.create_subscription(String, '/target_object', self.object_callback, 10)
        self.mask_sub = self.create_subscription(Image, '/segmentation_mask', self.mask_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.cam_info_sub = self.create_subscription(CameraInfo, '/camera/color/camera_info', self.cam_info_callback, 10)

        # Service Client für Greifkraft
        self.client = self.create_client(GrippingForce, 'GetGrippingForce')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for GrippingForce service...')

        self.req = GrippingForce.Request()
        self.force = 5.0
        self.object_name = "default"

        # TF2 setup for transformation
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # CV bridge
        self.bridge = CvBridge()
        self.depth_image = None
        self.cam_intrinsics = None
        self.latest_pose = None


    def object_callback(self, msg: String):
        self.object_name = msg.data.strip()
        self.get_logger().info(f"New object: '{self.object_name}'")

        self.req.object_name = self.object_name
        future = self.client.call_async(self.req)
        future.add_done_callback(self.handle_service_response)


    def handle_service_response(self, future):
        try:
            response = future.result()
            self.force = response.grip_force
            self.get_logger().info(f"Grip force for '{self.object_name}': {self.force:.1f} N")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            self.force = 5.0

        if self.latest_pose:
            self.publish_goal()


    def cam_info_callback(self, msg: CameraInfo):
        if self.cam_intrinsics is None:
            self.cam_intrinsics = msg
            self.get_logger().info("Camera info received.")


    def depth_callback(self, msg: Image):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')


    def mask_callback(self, msg: Image):
        if self.cam_intrinsics is None or self.depth_image is None:
            self.get_logger().warn("Waiting for camera info or depth image...")
            return

        # load mask
        mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')

        # calc centre of grafity
        moments = cv2.moments(mask)
        if moments['m00'] == 0:
            self.get_logger().warn("No segmented area found in mask.")
            return

        cX = int(moments['m10'] / moments['m00'])
        cY = int(moments['m01'] / moments['m00'])

        # read depth on centre of grafity
        depth = self.depth_image[cY, cX] / 1000.0

        if depth == 0 or np.isnan(depth):
            self.get_logger().warn("Invalid depth at centroid.")
            return

        # calc camera coordinates
        fx = self.cam_intrinsics.k[0]
        fy = self.cam_intrinsics.k[4]
        cx = self.cam_intrinsics.k[2]
        cy = self.cam_intrinsics.k[5]

        X = (cX - cx) * depth / fx
        Y = (cY - cy) * depth / fy
        Z = depth

        # PointStamped in cameraframe
        point_cam = PointStamped()
        point_cam.header = msg.header
        point_cam.point.x = X
        point_cam.point.y = Y
        point_cam.point.z = Z

        # transform in robotframe "base_link"
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame='base_link',
                source_frame=point_cam.header.frame_id,
                time=rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            transformed_point = do_transform_point(point_cam, transform)

            self.latest_pose = PoseStamped()
            self.latest_pose.header.frame_id = 'base_link'
            self.latest_pose.header.stamp = self.get_clock().now().to_msg()
            self.latest_pose.pose.position = transformed_point.point
            self.latest_pose.pose.orientation.w = 1.0

            if self.force:
                self.publish_goal()

        except Exception as e:
            self.get_logger().error(f"TF transform failed: {e}")


    def publish_goal(self):
        self.goal_pub.publish(self.latest_pose)

        goal = PoseStamped()
        goal.header.frame_id = "base_link"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = 0.4
        goal.pose.position.y = 0.0
        goal.pose.position.z = 0.4
        goal.pose.orientation.w = 1.0
        #self.goal_pub.publish(goal)

        force_msg = Float64()
        force_msg.data = self.force
        self.gripforce_pub.publish(force_msg)

        pos = self.latest_pose.pose.position
        orient = self.latest_pose.pose.orientation
        #pos = goal.pose.position
        #orient = goal.pose.orientation
        self.get_logger().info(
            f"Published goal pose in 'base_link': Orientation w={orient.w:.3f} | Position x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f} | Grip force: {self.force:.1f} N"
        )


def main(args=None):
    rclpy.init(args=args)
    node = PosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
