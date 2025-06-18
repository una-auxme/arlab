from arlab_template_interfaces.srv import AddThreeInts

import rclpy
from rclpy.node import Node


class MyRos2Service(Node):
    """This node provides a service that adds two ints

    The custom service definition can be found in the
    arlab_template_interfaces.srv folder
    """

    def __init__(self):
        super().__init__(type(self).__name__)
        self.srv = self.create_service(
            AddThreeInts, "/add_three_ints", self.add_three_ints_callback
        )

    def add_three_ints_callback(self, request, response):
        response.sum = request.a + request.b + request.c
        self.get_logger().info(
            f"Incoming request\na: {request.a} b: {request.b} c: {request.c}"
        )

        return response


def main(args=None):
    rclpy.init(args=args)

    my_ros2_service = MyRos2Service()

    rclpy.spin(my_ros2_service)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
