import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
from pyk4a import PyK4A, Config, ColorResolution, DepthMode
import threading


class KinectAzurePublisher(Node):
    def __init__(self):
        super().__init__(type(self).__name__)

        # Create a publisher for the camera image topic
        self.kinect_pub = self.create_publisher(Image, "/camera/image_raw", 10)
        self.bridge = CvBridge()

        # Configure the Azure Kinect camera
        self.config = PyK4A(
            Config(
                color_resolution=ColorResolution.RES_720P,
                depth_mode=DepthMode.OFF,
                synchronized_images_only=False,
            )
        )
        self.config.start()
        self.get_logger().info("Azure Kinect started")

        # Start the image capture loop in a separate thread
        self.running = True
        self.capture_thread = threading.Thread(target=self.capture_loop)
        self.capture_thread.start()

    def capture_loop(self):
        # Continuously capture and publish frames
        while self.running and rclpy.ok():
            try:
                capture = self.config.get_capture()
                if capture.color is not None:
                    # Remove alpha channel (convert BGRA to BGR)
                    frame = capture.color[:, :, :3]

                    # → Convert OpenCV image to ROS Image message
                    msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                    # → Add timestamp and frame ID to header
                    msg.header = Header()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = "azure_kinect"

                    # → Publish image message
                    self.kinect_pub.publish(msg)
                else:
                    self.get_logger().warn("No color frame available")
            except Exception as e:
                self.get_logger().warn(f"Error capturing frame: {e}")

    def stop(self):
        self.running = False
        self.capture_thread.join()
        self.config.stop()
        self.get_logger().info("Azure Kinect stopped")


def main(args=None):
    rclpy.init(args=args)
    my_ros2_node = KinectAzurePublisher()
    rclpy.spin(my_ros2_node)
    my_ros2_node.stop()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
