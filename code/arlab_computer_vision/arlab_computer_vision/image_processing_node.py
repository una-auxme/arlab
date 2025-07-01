import rclpy
import numpy as np

from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2


class ImagePreProcessing(Node):
    """Cool ros2 template node that publishes stuff to itself ;-)"""

    def __init__(self):
        super().__init__(type(self).__name__)

        self.depth_image_pub = self.create_publisher(PointCloud2, "/depth_image", 10)
        self.create_depth_camera_subscription(
            PointCloud2, "/depth_data_raw", self.process_data, 10
        )

    def pre_process_data(self, data: PointCloud2):
        """Receives messages from the /depth_camera_raw

        Args:
            msg (String): Received cool message
        """
        print(f"Received message: {String(data)}")

        ros_msg = PointCloud2()
        ros_msg.data = data
        ros_msg.header.stamp = self.get_clock().now().to_msg()
        ros_msg.header.frame_id = "camera_link"
        self.depth_image_pub.publish(ros_msg)

    def noise_filtering(self, data: PointCloud2):
        # Todo: implement noise filtering here.
        pass

    def depth_filering(self, data: PointCloud2):
        # Todo: implement logic for depth filtering here.
        pass

    def transform_data_base(self, data: PointCloud2, destination: str):
        if destination == "Movement":
            # Todo: Implement transformation to movement system.
            pass
        elif destination == "Manipulation":
            # Todo: Implement transformation to manipulation system.
            pass
        return data


def main(args=None):
    rclpy.init(args=args)

    my_ros2_node = ImagePreProcessing()

    rclpy.spin(my_ros2_node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
