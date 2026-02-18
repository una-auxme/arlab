import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header


class VideoPublisher(Node):
    def __init__(self):
        super().__init__(type(self).__name__)
        self.video_pub = self.create_publisher(Image, "/camera/image_raw", 1)
        self.bridge = CvBridge()

        # Lokales Video laden (Pfad anpassen)
        self.video_path = (
            "/workspace/src/arlab/code/arlab_perception/test_videos/vid1.mp4"
        )
        self.cap = cv2.VideoCapture(self.video_path)

        if not self.cap.isOpened():
            self.get_logger().error(f"Konnte Video nicht öffnen: {self.video_path}")
            return

        # Timer ruft `timer_callback` alle 0.033 Sekunden auf (ca. 30 FPS)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("VideoPublisher gestartet")
        # self.timer_callback()

    def timer_callback(self):
        if self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().info("Ende des Videos erreicht")
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # Wieder von vorne starten
                return

            frame = np.array(frame, copy=False)
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")

            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "video_frame"

            self.video_pub.publish(msg)
            self.get_logger().info("Video-Frame veröffentlicht")

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VideoPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
