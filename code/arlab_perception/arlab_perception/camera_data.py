# ------------------------------------------------------------
# Filename: kinect_azure_publisher.py
# Description: ROS 2 Node for publishing images from the Azure Kinect camera.
# Maintainer: Max Mustermann <max@example.com>
# Created: 2025-07-10
# Last Modified: 2025-08-14
# License: MIT
# ------------------------------------------------------------
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
from pyk4a import PyK4A, Config, ColorResolution, DepthMode
import threading


class KinectAzurePublisher(Node):
    """
    ROS 2 Node that interfaces with the Azure Kinect camera and publishes color
    images to the /camera/image_raw topic. Images are captured in a separate thread.
    """
    def __init__(self):
        """
        Initializes the KinectAzurePublisher Node, sets up the image publisher,
        and configures the Azure Kinect camera.
        """
        # Initialize the ROS 2 Node with the class name as the node name
        super().__init__(type(self).__name__)

        # Create a publisher for the camera image topic (Image message type)
        self.kinect_pub = self.create_publisher(Image, "/camera/image_raw", 10)

        # Create a CvBridge to convert OpenCV images to ROS Image messages
        self.bridge = CvBridge()

        # Configure and start the Azure Kinect camera
        self.config = PyK4A(
            Config(
                color_resolution=ColorResolution.RES_720P, # Set resolution to 720P
                depth_mode=DepthMode.OFF, # Depth data is disabled
                synchronized_images_only=False, # Allow unsynced color/depth
            )
        )
        self.config.start()
        self.get_logger().info("Azure Kinect started")

        # Start the image capture loop in a separate thread to avoid blocking
        self.running = True
        self.capture_thread = threading.Thread(target=self.capture_loop)
        self.capture_thread.start()

    def capture_loop(self):
        """
        Continuously captures images from the Azure Kinect camera and publishes them
        to the /camera/image_raw topic.
        """
        # Continuously capture and publish frames as long as the node is running
        while self.running and rclpy.ok():
            try:
                # Capture a frame from the Azure Kinect camera
                capture = self.config.get_capture()
                if capture.color is not None:

                    # Remove alpha channel (convert BGRA to BGR)
                    frame = capture.color[:, :, :3]

                    # Convert the OpenCV image to a ROS Image message
                    msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")

                    # Add timestamp and frame ID to the message header
                    msg.header = Header()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = "azure_kinect"

                    # Publish the image message to the /camera/image_raw topic
                    self.kinect_pub.publish(msg)
                else:
                    # Log a warning if no color frame is available
                    self.get_logger().warn("No color frame available")
            except Exception as e:
                # Log any errors encountered while capturing frames
                self.get_logger().warn(f"Error capturing frame: {e}")

    def stop(self):
        """
        Stops the image capture loop and releases resources.
        """
        self.running = False  # Stop the capture loop
        self.capture_thread.join()  # Wait for the capture thread to finish
        self.config.stop()  # Stop the Azure Kinect camera
        self.get_logger().info("Azure Kinect stopped")


def main(args=None):
    """
    Initializes the ROS 2 client library, starts the KinectAzurePublisher Node,
    and keeps it running until shutdown.
    """
    # Initialize ROS 2
    rclpy.init(args=args)

    # Create an instance of the KinectAzurePublisher Node
    my_ros2_node = KinectAzurePublisher()

    # Keep the node running and process callbacks
    rclpy.spin(my_ros2_node)

    # Stop the node and release resources when done
    my_ros2_node.stop()
    rclpy.shutdown()


if __name__ == "__main__":
    # Execute the main function when the script is run directly
    main()
