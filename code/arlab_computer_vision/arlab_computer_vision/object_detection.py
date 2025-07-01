import rclpy
import numpy as np

from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2

class ObjectDetection(Node):
    """Cool ros2 template node that publishes stuff to itself ;-)"""

    def __init__(self):
        super().__init__(type(self).__name__)

        # Todo: Implement object detection model here.
        self.model = None

        self.create_camera_data_subscriber(
            PointCloud2, "/depth_data", self.process_data, 10
        )

    def detect_objects(self, data: PointCloud2):
        # Todo: Implement the obejct detection with the ML model
        pass


    def flatten_pointcloud2_to_xy(self, pc2_msg: PointCloud2) -> PointCloud2:
        points = point_cloud2.read_points(pc2_msg, field_names=('x', 'y'), skip_nans=True)
        flattened = [(x, y) for (x, y) in points]

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        ]

        return point_cloud2.create_cloud(pc2_msg.header, fields, flattened)



    def process_data(self, data: PointCloud2):
        """Receives messages from the /depth_camera_raw

        Args:
            msg (String): Received cool message
        """
        print(f"Received message: {String(data)}")

        pointcloud_2d = self.flatten_pointcloud2_to_xy(data)
        output = self.detect_objects(pointcloud_2d)


        # Todo: Save classified data in knowledge base.
        pass


def main(args=None):
    rclpy.init(args=args)

    my_ros2_node = ObjectDetection()

    rclpy.spin(my_ros2_node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
