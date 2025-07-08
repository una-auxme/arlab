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
            "banane": 2.0,
            "apfel": 5.0,
            "flasche": 8.0,
            "tasse": 4.0,
            "milch": 7.5,
            "zahnpasta": 3.0,
            "pringles": 3.0,
            "joghurt": 2.0
        }

        self.default_force = 5.0

    def callback(self, request, response):
        raw_name = request.object_name
        object_name = raw_name.lower().strip()
        grip_force = self.object_force_table.get(object_name, self.default_force)
        response.grip_force = grip_force

        self.get_logger().info(
            f"Request: Original='{raw_name}' → Normalised='{object_name}' → Grippingforce={grip_force:.1f} N"
        )

        return response


def main(args=None):
    rclpy.init(args=args)
    node = GetGrippingForce()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
