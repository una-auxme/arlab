#!/usr/bin/env python3

"""
GrippingParameterNode: Provide gripping parameters for robotic manipulation.

This module defines a ROS 2 node that maps semantic object information
(object name and group) to technical control parameters for a gripper
(grip force, position mode, orientation mode, object weight).

Maintainer:
    Sofia Öttl <sofia.oettl@uni-a.de>
    Christopher Müller <christopher.mueller@uni-a.de>
"""

import rclpy
from arlab_common_interfaces.srv import GrippingParameter
from rclpy.node import Node


class GrippingParameterNode(Node):
    """ROS2 Node that provides gripping parameters for objects.

    This node abstracts the decision of how to grasp an object by maintaining
    predefined tables for object groups and weights. Centralizing this logic
    ensures consistent behavior across the manipulation pipeline and allows
    easy updates.

    Topic Interface:
        * **Input**: objectname, objectgroup.
        * **Output**: gripforce, grippos_mode, griporient_mode, object_weight.

    Notes:
        - Unknown objects fall back to default parameters to ensure safe gripping.
        - Object grouping reduces the number of explicit entries needed and
          allows similar-shaped objects to share parameters.

    Attributes:
        group_parameter_table: Mapping from object group → [grip force, pos mode, orient mode].
        object_weight_table: Mapping from object name → weight [kg].
        grip_type_table: Mapping from object name → mia hand grip type (cylindrical, pinch, lateral, spherical, tridigital)
    """

    def __init__(self):
        """Initialize the ROS 2 node and register the service.

        The node name matches the class name for easier debugging. The service
        handle is stored on ``self`` to prevent garbage collection and ensure
        requests are properly handled.

        Side Effects:
            Registers a service callback with the executor.
        """
        super().__init__("GetGrippingParameter")

        # Register the service; keep reference to prevent garbage collection
        self.srv = self.create_service(GrippingParameter, "GetGrippingParameter", self.callback)

        # Table: Object group → gripping parameters (force [N], pos_mode, orient_mode)
        self.group_parameter_table = {
            "object_category_unknown": [5.0, 0, 0],
            "sphere": [2.0, 0, 0],  # Sphere: gentle grip, because its mostly fruits
            "cube": [2.0, 0, 0],
            "cylinder": [5.0, 0, 0],
            "cone": [2.0, 0, 0],
            "ring": [5.0, 0, 0],
            "capsule": [2.0, 0, 0],
            "default": [5.0, 0, 0],  # Default: if object group not found
        }

        # Table: Object name → weight [kg]
        self.object_weight_table = {
            "apple": 0.25,
            "banana": 0.3,
            "beer": 0.2,
            "bottle": 0.25,
            "cereals": 0.3,
            "cup": 0.2,
            "milk": 0.25,
            "chipscan": 0.3,
            "chipsbag": 0.2,
            "shoppingbag": 0.25,
            "default": 1.0,  # Default: if object name not found
        }

        # Table: Object name → mia hand grip type (cylindrical, pinch, lateral, spherical, tridigital)
        self.grip_type_table = {
            "beer" : "cylindrical",
            "chipsbag" : "pinch",
            "default" : "cylindrical",  # Default: if object name not found
        }

    def callback(self, request, response):
        """Compute gripping parameters for a requested object.

        Determines appropriate grip force, position mode, orientation mode,
        and weight. Defaults are used for unknown objects or groups to
        ensure safe handling.

        Args:
            request: Service request containing object name and group.
            response: Service response to populate with grip parameters.

        Side Effects:
            Populates the response and logs the selected parameters.

        Threading:
            Executed by the rclpy executor thread for this node.
        """

        # Look up weight; fall back to default if object name is unknown
        object_weight = self.object_weight_table.get(request.objectname, self.object_weight_table["default"])

        # Look up grip parameters; fall back to default if group is unknown
        grip_parameter = self.group_parameter_table.get(request.objectgroup, self.group_parameter_table["default"])

        # Look up grip types; fall back to default if group is unknown
        grip_type = self.grip_type_table.get(request.objectname, self.grip_type_table["default"])

        # Unpack the parameters for direct use in the service response
        gripforce, grippos_mode, griporient_mode = grip_parameter

        self.get_logger().info(
            f"Send parameter for object '{request.objectname}' "
            f"(group: '{request.objectgroup}') → "
            f"Gripping force = {gripforce} N, "
            f"Gripping type = {grip_type}, "
            f"Gripping position mode = {grippos_mode}, "
            f"Gripping orientation mode = {griporient_mode}, "
            f"Object weight = {object_weight} kg"
        )

        # Populate and return the response
        response.gripforce = gripforce
        response.grip_type = grip_type
        response.grippos_mode = grippos_mode
        response.griporient_mode = griporient_mode
        response.object_weight = object_weight

        return response


def main(args=None):
    """Start the GrippingParameterNode and spin until shutdown.

    Args:
        args: Optional command-line arguments forwarded to ``rclpy.init``.
    """
    rclpy.init(args=args)
    node = GrippingParameterNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
