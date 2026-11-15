"""
Provide steady, shutdown-aware waits for the sequence request client.

Responsibility:
- Own the process shutdown flag and interruptible ROS wait probes.
- Gate the final transition from preparation to action goal submission.

Does not own:
- Action cancellation, late response reconciliation, or result interpretation.
"""

from __future__ import annotations

import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

TRANSPORT_WAIT_SLICE_SEC = 0.05
CANCEL_TRANSPORT_PROBE_SEC = 3.0
RECONCILE_TRANSPORT_PROBE_SEC = 3.0


class ShutdownRequest:
    """Record POSIX shutdown without invalidating the rclpy context."""

    def __init__(self) -> None:
        """Create one thread-safe process-local shutdown flag."""
        self._event = threading.Event()

    @property
    def requested(self) -> bool:
        """Return whether SIGINT, SIGTERM, or a wait interruption occurred."""
        return self._event.is_set()

    def request(self, *_unused) -> None:
        """Mark shutdown from a signal handler or exception path."""
        self._event.set()


def goal_send_permitted(shutdown: ShutdownRequest) -> bool:
    """Reject the last preparation-to-side-effect transition after shutdown."""
    return not shutdown.requested


def wait_future(
    node: Node,
    future,
    *,
    timeout_sec: float,
    shutdown: ShutdownRequest,
    honor_shutdown: bool = True,
) -> bool:
    """Spin a future against a steady deadline independent of ROS time."""
    deadline = time.monotonic() + timeout_sec
    while not future.done():
        if honor_shutdown and shutdown.requested:
            return False
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            return False
        try:
            rclpy.spin_once(
                node,
                timeout_sec=min(TRANSPORT_WAIT_SLICE_SEC, remaining),
            )
        except KeyboardInterrupt:
            shutdown.request()
            if honor_shutdown:
                return False
    return True


def wait_until_future_or_shutdown(
    node: Node,
    future,
    *,
    shutdown: ShutdownRequest,
) -> bool:
    """Wait without an elapsed-time budget until a future or shutdown."""
    while not future.done():
        if shutdown.requested:
            return False
        try:
            rclpy.spin_once(node, timeout_sec=TRANSPORT_WAIT_SLICE_SEC)
        except KeyboardInterrupt:
            shutdown.request()
            return False
    return True


def wait_for_terminal_result(
    node: Node,
    future,
    *,
    shutdown: ShutdownRequest,
) -> bool:
    """Retain an accepted goal until terminal transport or shutdown."""
    return wait_until_future_or_shutdown(node, future, shutdown=shutdown)


def wait_for_server(
    client: ActionClient,
    *,
    shutdown: ShutdownRequest,
) -> bool:
    """Wait for the public sequence endpoint until discovery or shutdown."""
    while not shutdown.requested:
        if client.wait_for_server(timeout_sec=TRANSPORT_WAIT_SLICE_SEC):
            return True
    return False


def wait_for_nonzero_stamp(
    node: Node,
    *,
    shutdown: ShutdownRequest,
):
    """Wait for /clock until a valid stamp or shutdown without a task deadline."""
    while not shutdown.requested:
        stamp = node.get_clock().now().to_msg()
        if stamp.sec != 0 or stamp.nanosec != 0:
            return stamp
        rclpy.spin_once(node, timeout_sec=TRANSPORT_WAIT_SLICE_SEC)
    return None
