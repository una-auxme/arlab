#!/usr/bin/env python3

"""
GetGrippingForce.py
---------------------

ROS2 Node 'GetGrippingForce' providing a service to return recommended gripping
forces for different objects.

Author: Sofia Öttl
Date: 2025-08-24

"""

import rclpy
from rclpy.node import Node

from arlab_common_interfaces.srv import GrippingForce


class GetGrippingForce(Node):
    """ROS2 Node for providing gripping force for different objects.

    This node offers a ROS2 service 'GetGrippingForce' that returns the recommended
    gripping force in Newtons (N) for a given object name. The forces are based on
    a predefined object-force table. If the object is not in the table, a default
    force is used.

    Attributes:
        srv (rclpy.Service): ROS2 service for getting gripping force.
        object_force_table (dict): Mapping from object names (str) to gripping forces
        (float, N).
        default_force (float): Default gripping force for unknown objects.
    """

    def __init__(self):
        """Initialize the GetGrippingForce node, create the service and define the
        object_force_table"""
        super().__init__("GetGrippingForce")
        self.srv = self.create_service(GrippingForce, "GetGrippingForce", self.callback)

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
            "shoppingbag": 7.0,
        }

        self.default_force = 5.0

    def callback(self, request, response):
        """Service callback that computes the gripping force for a requested object.

        Args:
            request (GrippingForce.Request): The service request containing the object
            name.
            response (GrippingForce.Response): The service response where the gripping
            force is stored.

        Returns:
            GrippingForce.Response: The response containing the computed gripping force.
        """
        gripforce = self.object_force_table.get(request.objectname, self.default_force)

        self.get_logger().info(
            f"Objectname='{request.objectname}' → Grippingforce={gripforce:.1f} N"
        )

        response.gripforce = gripforce
        return response


def main(args=None):
    """Main function to initialize and spin the GetGrippingForce node.

    Args:
        args (list, optional): Command line arguments. Defaults to None.
    """
    rclpy.init(args=args)
    node = GetGrippingForce()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
