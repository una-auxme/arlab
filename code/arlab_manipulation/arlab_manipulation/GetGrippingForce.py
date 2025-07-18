#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from arlab_common_interfaces.srv import GrippingForce

class GetGrippingForce(Node):
    def __init__(self):
        super().__init__('GetGrippingForce')
        self.srv = self.create_service(GrippingForce, 'GetGrippingForce', self.callback)

        # Example object-gripping-force table --> need to be more detailed
        self.object_force_table = {
            "banana": 2.0,
            "apple": 5.0,
            "bottle": 8.0,
            "cup": 4.0,
            "milk": 7.5,
            "beer": 3.0,
            "chipscan": 3.0,
            "chipsbag": 2.0,
            "cereals": 2.0,
            "shoppingbag": 7.0
        }

        self.default_force = 5.0

    def callback(self, request, response):
        gripforce = self.object_force_table.get(request.objectname, self.default_force)

        self.get_logger().info(
            f"Objectname='{request.objectname}' → Grippingforce={gripforce:.1f} N"
        )

        response.gripforce = gripforce
        return response

def main(args=None):
    rclpy.init(args=args)
    node = GetGrippingForce()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
