import rclpy

from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2


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

    def process_data(self, data: PointCloud2):
        """Receives messages from the /depth_camera_raw

        Args:
            msg (String): Received cool message
        """
        print(f"Received message: {String(data)}")

        # output = self.detect_objects(data)

        # Todo: Save classified data in knowledge base.
        pass


def main(args=None):
    rclpy.init(args=args)

    my_ros2_node = ObjectDetection()

    rclpy.spin(my_ros2_node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
