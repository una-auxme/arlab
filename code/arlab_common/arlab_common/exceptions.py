"""Utility functions for exception handling and error formatting.

This module provides helper functions for formatting exceptions with their
full traceback for logging and debugging purposes.

Maintainers:
    Peter Viechter <peter.viechter@uni-a.de>
"""

import traceback


def emsg_with_trace(e: Exception) -> str:
    """Format an exception with its full traceback.

    Args:
        e: The exception to format

    Returns:
        str: Formatted exception message with traceback
    """
    traceback_str_list = traceback.format_exception(e)
    traceback_str = "".join(traceback_str_list)
    return f"\n{traceback_str}"
