"""
ROS shutdown guard helpers for Python nodes.

Responsibility:
- Recognize known rclpy spin RuntimeErrors that can surface after shutdown starts.

Does not own:
- Node lifecycle or signal handling.
- Suppressing application errors while the ROS context is still active.
"""

from __future__ import annotations


def should_ignore_shutdown_runtime_error(exc: RuntimeError, *, context_ok: bool) -> bool:
    """
    Recognize one rclpy conversion error only after context shutdown.

    While the context remains active every RuntimeError is treated as an
    application failure. This narrow check avoids hiding unrelated callback or
    executor errors.
    """
    if context_ok:
        return False
    return "Unable to convert call argument" in str(exc)
