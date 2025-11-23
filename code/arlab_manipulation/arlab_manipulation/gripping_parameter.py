#!/usr/bin/env python3

"""
GetGrippingParameter.py
---------------------

ROS2 Node 'GetGrippingParameter' providing a service to return recommended gripping
forces for different objects.

Author: Sofia Öttl
Date: 2025-08-24

"""

import rclpy
from rclpy.node import Node

from arlab_common_interfaces.srv import GrippingParameter


class gripping_parameter(Node):
    """ROS2 Node for providing gripping/placing parameter for different objects."""

    def __init__(self):
        """Initialize the GetGrippingParameter node, create the service and define the
        object_parameter_table"""
        super().__init__("GetGrippingParameter")
        self.srv = self.create_service(
            GrippingParameter, "GetGrippingParameter",
            self.callback
            )

        # Example objectgroup-gripping-parameter table --> need to be more detailed
        self.object_parameter_table = {
            "fruits": [2.0, 0, 0],
            "cylinder": [5.0, 0, 0],
            "default": [5.0, 0, 0]
        }

    def callback(self, request, response):
        """Service callback that computes the gripping force for a requested object."""

        grip_parameter = self.object_parameter_table.get(
            request.objectgroup,
            self.object_parameter_table["default"]
            )

        gripforce, grippos_mode, griporient_mode = grip_parameter

        self.get_logger().info(
            f"Send parameter for '{request.objectgroup}' → "
            f"Gripping force = {gripforce}N, "
            f"Gripping position mode = {grippos_mode}, "
            f"Gripping orientation mode = {griporient_mode}"
        )

        response.gripforce = gripforce
        response.grippos_mode = grippos_mode
        response.griporient_mode = griporient_mode

        return response


def main(args=None):
    """Main function to initialize and spin the GetGrippingParameter node.

    Args:
        args (list, optional): Command line arguments. Defaults to None.
    """
    rclpy.init(args=args)
    node =  gripping_parameter()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
