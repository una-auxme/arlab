import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan



class LidarPubSub(Node):
    def __init__(self):
        super().__init__(type(self).__name__)


        # Abonniert das LIDAR-Topic
        self.lidar_sub = self.create_subscription(LaserScan, "/scan", self.lidar_callback, 10)

        # Wird benötigt, um Garbage Collection zu verhindern
        self.lidar_pub = self.create_publisher(LaserScan, "/relay_scan", 10)

    def lidar_callback(self, msg: LaserScan):
        # Es wird einfach weitergeleitet, ohne Filterung
        self.lidar_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    lidar_node = LidarPubSub()
    rclpy.spin(lidar_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()