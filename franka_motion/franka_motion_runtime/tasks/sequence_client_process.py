"""
Own the common process lifecycle of Franka transfer-sequence clients.

Responsibility:
- Preserve the rclpy context while a client contains an accepted action goal.
- Install cooperative POSIX shutdown handlers and restore process state.

Does not own:
- Request construction, action transport, result policy, or motion execution.
"""

from __future__ import annotations

from collections.abc import Callable
import signal

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from franka_motion_runtime.tasks.sequence_client_waits import ShutdownRequest


ClientRunner = Callable[[Node, ShutdownRequest], int]


def run_client_process(
    node_name: str,
    runner: ClientRunner,
    *,
    args=None,
) -> int:
    """Run one contained client and always restore signals and ROS context."""
    shutdown = ShutdownRequest()
    previous_sigint = signal.getsignal(signal.SIGINT)
    previous_sigterm = signal.getsignal(signal.SIGTERM)
    node = None
    initialized = False
    signal.signal(signal.SIGINT, shutdown.request)
    signal.signal(signal.SIGTERM, shutdown.request)
    try:
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
        initialized = True
        node = Node(node_name)
        try:
            return runner(node, shutdown)
        except KeyboardInterrupt:
            shutdown.request()
            node.get_logger().error("Transfer sequence client interrupted")
            return 5
        except Exception as error:  # Keep CLI failures explicit and non-zero.
            node.get_logger().error(f"Transfer sequence client failed: {error}")
            return 9
    finally:
        try:
            if node is not None:
                node.destroy_node()
        finally:
            try:
                if initialized:
                    rclpy.shutdown()
            finally:
                signal.signal(signal.SIGINT, previous_sigint)
                signal.signal(signal.SIGTERM, previous_sigterm)
