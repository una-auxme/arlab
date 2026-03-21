"""Debugging utilities for ROS2 nodes.

This module provides helper functions for starting Python debuggers and
retrieving caller information for debugging purposes.

Maintainers:
    Peter Viechter <peter.viechter@uni-a.de>
"""

import importlib.util
import inspect
from typing import Optional

import rclpy.logging
from rclpy.impl.rcutils_logger import RcutilsLogger


def get_logger() -> RcutilsLogger:
    """Get the logger for the debugger node.

    Returns:
        RcutilsLogger: The logger instance for the "debugger" node.
    """
    return rclpy.logging.get_logger("debugger")


def get_caller_file() -> str:
    """Get the filename of the caller's function.

    Uses the inspect module to retrieve the filename of the function
    that called this function.

    Returns:
        str: The filename of the caller, or "unknown" if unable to determine.
    """
    stack = inspect.stack()
    if len(stack) < 1:
        return "unknown"
    caller = stack[-1]
    return caller.filename


def start_debugger(
    node_module_name: Optional[str] = None,
    host: str = "127.0.0.1",
    port: int = 53000,
    wait_for_client: bool = False,
) -> None:
    """Start a Python debugger for the current node.

    This function starts a debugpy debugger listener on the specified host
    and port. It can optionally wait for a debugging client to attach before
    continuing execution.

    Args:
        node_module_name (str, optional): Name of the underlying node. Only used for logging.
            Defaults to the caller's filename if not provided.
        host (str, optional): Host address the debugger binds to. Defaults to "127.0.0.1".
        port (int, optional): Port number for the debugger. Defaults to 53000.
        wait_for_client (bool, optional): If True, wait for a debugging client to attach
            before continuing. Defaults to False.
    """
    debugger_spec = importlib.util.find_spec("debugpy")
    if debugger_spec is not None:
        try:
            if node_module_name is None:
                node_module_name = get_caller_file()
            import debugpy

            debugpy.listen((host, port))
            get_logger().warn(f"Started debugger on {host}:{port} for {node_module_name}")
            if wait_for_client:
                get_logger().warn("Waiting until debugging client is attached...")
                debugpy.wait_for_client()
        except Exception as error:
            # Yes, all exceptions should be catched and sent into rosconsole
            get_logger().error(f"Failed to start debugger: {error}")
    else:
        get_logger().error("debugpy module not found. Unable to start debugger")
