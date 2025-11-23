"""ROS 2 publisher for Azure Kinect color images.

Subscribes to the Azure Kinect via ``pyk4a`` and publishes color frames
as ``sensor_msgs.msg.Image`` on ``/camera/image_raw``. Frames are acquired
on a dedicated thread to avoid blocking the ROS 2 executor.

Maintainer:
    Meruna Yugarajah <m.yugarajah@gmail.com>
"""

import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
from pyk4a import PyK4A, Config, ColorResolution, DepthMode


class KinectAzurePublisher(Node):
    """ROS 2 node that captures and publishes Azure Kinect color images.

    The node starts an Azure Kinect device (via ``pyk4a``) and continuously
    publishes color images to ``/camera/image_raw``. Each message contains a
    timestamped ``Header`` with ``frame_id="azure_kinect"`` and the image data
    encoded as ``bgr8``.

    Topic Interface:
    * **Output** ``/camera/image_raw`` – ``sensor_msgs.msg.Image``

    Attributes:
        kinect_pub (rclpy.publisher.Publisher): Publisher for ``Image`` messages
        on ``/camera/image_raw``.
        bridge (CvBridge): Converter between OpenCV arrays and ROS ``Image``.
        config (PyK4A): Azure Kinect device handle configured for color capture.
        running (bool): Loop control flag for the capture thread.
        capture_thread (threading.Thread): Background thread acquiring frames.

    Notes:
        - Color frames from Azure Kinect are BGRA; the alpha channel is stripped
          to BGR before conversion to a ROS ``Image`` with encoding ``bgr8``.
        - Depth capture is disabled (``DepthMode.OFF``).
        - If no color frame is available for a cycle, a warning is logged and
          no message is published.
    """

    def __init__(self):
        """Initialize the node, the Azure Kinect, and the capture thread.

        Creates the publisher and CvBridge instance, configures and starts the
        Azure Kinect device, and launches a background thread that acquires and
        publishes frames.

        Side Effects:
            Starts the Azure Kinect hardware stream and a background thread.

        Raises:
            RuntimeError: Propagated if the Azure Kinect device fails to start.
        """
        super().__init__(type(self).__name__)

        self.kinect_pub = self.create_publisher(Image, "/camera/image_raw", 10)
        self.bridge = CvBridge()

        self.config = PyK4A(
            Config(
                color_resolution=ColorResolution.RES_720P,
                depth_mode=DepthMode.OFF,
                synchronized_images_only=False,
            )
        )
        self.config.start()
        #self.get_logger().info("Azure Kinect started")

        # Start the image capture loop in a separate thread to avoid blocking
        self.running = True
        self.capture_thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.capture_thread.start()

    def capture_loop(self):
        """Continuously capture color frames and publish them.

        Grabs frames from the Azure Kinect while the node is running and ROS 2
        is active. Converts each frame to a ROS ``Image`` and publishes it on
        ``/camera/image_raw`` with a stamped header.

        Side Effects:
            Publishes messages and logs warnings on capture issues.

        Threading:
            Runs on a dedicated background thread. Uses ``self.running`` as a
            stop flag; no additional synchronization is required because only
            this thread mutates capture state.
        """
        while self.running and rclpy.ok():
            try:
                capture = self.config.get_capture()
                if capture.color is not None:
                    # Azure Kinect color is BGRA; drop alpha → BGR
                    frame = capture.color[:, :, :3]
                    msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                    msg.header = Header()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = "azure_kinect"
                    self.kinect_pub.publish(msg)
                else:
                    #self.get_logger().warning("No color frame available")
            except Exception as e:
                #self.get_logger().warning(f"Error capturing frame: {e}")

    def stop(self):
        """Stop capture and release device resources."""
        self.running = False
        if self.capture_thread.is_alive():
            self.capture_thread.join()
        self.config.stop()
        #self.get_logger().info("Azure Kinect stopped")


def main(args=None):
    rclpy.init(args=args)
    node = KinectAzurePublisher()
    rclpy.spin(node)
    node.stop()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
