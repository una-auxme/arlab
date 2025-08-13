import rclpy

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.mgs import Header
from pyk4a import PyK4A, Config, ColorResolution, DepthMode


class KinectAzurePublisher(Node):
    def __init__(self):
        super().__init__(type(self).__name__)
        self.kinect_pub = self.create_publisher(Image, "/camera/image_raw", 10)
        self.bridge = CvBridge()

        # Kamera konfigurieren
        self.config = PyK4A(
            Config(
                color_resolution=ColorResolution.RES_720P,
                depth_mode=DepthMode.OFF,
                synchronized_images_only=False,
            )
        )

        self.config.start()
        self.get_logger().info("Azure Kinect gestartet")

    def timer_callback(self):
        try:
            capture = self.config.get_capture()
            if capture.color is None:
                # Alpha Kanal wird entfernt wegen BGR
                frame = capture.color[:, :, :3]
                # OpenCV wird in ROS Image konvertiert
                msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")

                # Header für Zeitstempel
                msg.header = Header()
                msg.header.stamp = self.get_clock().now.to_msg()
                msg.header.frame_id = "Azure Kinect ID"

                # Nachricht wird veröffentlicht
                self.kinect_pub.publish(msg)
                self.get_logger().info("Bild wird veröffentlicht")
            else:
                self.get_logger().info("Kein Farb-Frame vorhanden")

        except Exception:
            self.get_logger().warn("Fehler bei der Kamera-Frame.")


def main(args=None):
    rclpy.init(args=args)
    my_ros2_node = KinectAzurePublisher()
    rclpy.spin(my_ros2_node)
    my_ros2_node.config.stop()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
