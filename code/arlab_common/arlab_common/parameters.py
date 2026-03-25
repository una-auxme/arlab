"""Parameter update utilities for ROS2 nodes.

This module provides helper functions for updating node attributes from
ROS2 parameters, with type checking and error handling.

Maintainers:
    Peter Viechter <peter.viechter@uni-a.de>
"""

from typing import List

from rcl_interfaces.msg import (
    SetParametersResult,
)
from rclpy.node import Node
from rclpy.parameter import Parameter


def update_attributes(obj: Node, params: List[Parameter]) -> SetParametersResult:
    """Update node attributes from ROS2 parameters.

    Updates the attributes of a node object with values from the provided
    ROS2 parameters. Attribute names must match parameter names.

    Args:
        obj (Node): The node whose attributes should be updated.
        params (List[Parameter]): List of parameters with new values.

    Returns:
        SetParametersResult: Result object indicating success or failure.
            The successful attribute is set to True if all parameters were
            updated successfully, False otherwise. The reason attribute
            contains error messages for failed updates.
    """
    result = SetParametersResult()
    result.successful = True
    for param in params:
        error_reason = None
        if hasattr(obj, param.name):
            new_value = param.value
            orig_value = getattr(obj, param.name)
            if orig_value is not None and not isinstance(new_value, type(orig_value)) and not isinstance(orig_value, type(new_value)):
                error_reason = "type mismatch"
            else:
                setattr(obj, param.name, param.value)
                obj.get_logger().info(f"Updated parameter {param.name} to {new_value}")
        else:
            error_reason = "attribute not found"

        if error_reason is not None:
            result.successful = False
            result.reason = error_reason
            obj.get_logger().warn(f"Failed to update parameter {param.name}: {result.reason}")

    return result
