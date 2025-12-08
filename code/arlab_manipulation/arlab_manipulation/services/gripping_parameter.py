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
from geometry_msgs.msg import Point, Quaternion


class GrippingParameterNode(Node):
    """ROS2 Node for providing gripping/placing parameters for different objects."""

    def __init__(self):
        super().__init__("GetGrippingParameter")
        self.srv = self.create_service(
            GrippingParameter,
            "GetGrippingParameter",
            self.callback
        )

        # Object group → gripping parameters (force, pos_mode, orient_mode)
        self.group_parameter_table = {
            "fruits": [2.0, 0, 0],
            "cylinder": [5.0, 0, 0],
            "default": [5.0, 0, 0]
        }

        # Object name → weight [kg]
        self.object_weight_table = {
            "apple": 0.2,
            "banana": 0.25,
            "orange": 0.3,
            "default": 1.0
        }

    def callback(self, request, response):
        """Service callback that computes the gripping force for a requested object."""

        object_weight = self.object_weight_table.get(
            request.objectname, self.object_weight_table["default"])

        grip_parameter = self.group_parameter_table.get(
            request.objectgroup,
            self.group_parameter_table["default"]
        )

        gripforce, grippos_mode, griporient_mode = grip_parameter

        self.get_logger().info(
            f"Send parameter for object '{request.objectname}' "
            f"(group: '{request.objectgroup}') → "
            f"Gripping force = {gripforce} N, "
            f"Gripping position mode = {grippos_mode}, "
            f"Gripping orientation mode = {griporient_mode}, "
            f"Object weight = {object_weight} kg"
        )

        response.gripforce = gripforce
        response.grippos_mode = grippos_mode
        response.griporient_mode = griporient_mode
        response.object_weight = object_weight

        return response


def main(args=None):
    rclpy.init(args=args)
    node = GrippingParameterNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
