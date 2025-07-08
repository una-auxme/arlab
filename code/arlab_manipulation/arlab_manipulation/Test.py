import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading

class Test(Node):
    def __init__(self):
        super().__init__('Test')

        # Publisher definieren
        self.object_pub = self.create_publisher(String, '/target_object', 10)
        self.mask_pub = self.create_publisher(Image, '/segmentation_mask', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/aligned_depth_to_color/image_raw', 10)
        self.cam_info_pub = self.create_publisher(CameraInfo, '/camera/color/camera_info', 10)

        self.bridge = CvBridge()

        # Starte Thread für Nutzereingabe
        threading.Thread(target=self.wait_for_input, daemon=True).start()

    def wait_for_input(self):
        while rclpy.ok():
            input("Press [Enter] to publish test data...")
            self.publish_all()

    def publish_all(self):
        self.publish_object_name()
        self.publish_mask()
        self.publish_depth()
        self.publish_cam_info()

    def publish_object_name(self):
        msg = String()
        msg.data = "Apfel"
        self.object_pub.publish(msg)
        self.get_logger().info("✅ Published object name: 'Kuchen'")

    def publish_mask(self):
        mask = np.zeros((480, 640), dtype=np.uint8)
        cv2.circle(mask, (320, 240), 50, 255, -1)
        msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
        msg.header.frame_id = 'camera_color_frame'
        msg.header.stamp = self.get_clock().now().to_msg()
        self.mask_pub.publish(msg)
        self.get_logger().info("✅ Published segmentation mask")

    def publish_depth(self):
        depth = np.full((480, 640), 1000, dtype=np.uint16)
        msg = self.bridge.cv2_to_imgmsg(depth, encoding='passthrough')
        msg.header.frame_id = 'camera_depth_frame'
        msg.header.stamp = self.get_clock().now().to_msg()
        self.depth_pub.publish(msg)
        self.get_logger().info("✅ Published depth image")

    def publish_cam_info(self):
        msg = CameraInfo()
        msg.header.frame_id = 'camera_color_frame'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.width = 640
        msg.height = 480
        msg.k = [
            615.0, 0.0, 320.0,
            0.0, 615.0, 240.0,
            0.0, 0.0, 1.0
        ]
        self.cam_info_pub.publish(msg)
        self.get_logger().info("✅ Published camera info")

def main(args=None):
    rclpy.init(args=args)
    node = Test()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
