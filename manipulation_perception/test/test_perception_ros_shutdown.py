"""
Test classification of ROS shutdown RuntimeError instances.

Responsibility: Suppress only errors caused by an already-stopped rclpy context.
"""

from pathlib import Path
import sys


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from manipulation_perception.common.ros_shutdown import (  # noqa: E402,I100,I101,I201
    should_ignore_shutdown_runtime_error,
)


def test_shutdown_runtime_error_is_ignored_only_after_context_shutdown():
    exc = RuntimeError("Unable to convert call argument '0' to Python object")

    assert should_ignore_shutdown_runtime_error(exc, context_ok=False)
    assert not should_ignore_shutdown_runtime_error(exc, context_ok=True)


def test_unrelated_runtime_error_is_not_ignored():
    exc = RuntimeError("detector failed")

    assert not should_ignore_shutdown_runtime_error(exc, context_ok=False)
