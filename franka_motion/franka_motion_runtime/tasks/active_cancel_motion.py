"""
Observe fresh controller execution before canceling a transfer approach.

Responsibility:
- Correlate current transfer feedback with post-submission controller status.
- Own the status subscription and bounded, interruptible checkpoint wait.

Does not own:
- Controller execution, sequence submission, cancellation, or scenario endpoints.
"""

from __future__ import annotations

import time

from action_msgs.msg import GoalStatusArray
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_action_status_default

from franka_motion_runtime.tasks.active_cancel_validation import (
    ActiveCancelCheckpoint,
    ControllerGoalSnapshot,
)
from franka_motion_runtime.tasks.sequence_client_feedback import FeedbackLogger
from franka_motion_runtime.tasks.sequence_client_waits import (
    ShutdownRequest,
    TRANSPORT_WAIT_SLICE_SEC,
)


class _ActiveCancelFeedback:
    """Keep current task phase and fresh controller evidence under one owner."""

    def __init__(self, node: Node) -> None:
        """Create an unarmed observer without allocating ROS subscriptions."""
        self._node = node
        self._feedback_logger = FeedbackLogger(node)
        self._checkpoint = ActiveCancelCheckpoint()
        self._subscription = None

    def connect_controller(self, action_name: str) -> None:
        """Observe the configured controller action using its status QoS."""
        self._subscription = self._node.create_subscription(
            GoalStatusArray,
            action_name.rstrip("/") + "/_action/status",
            self._observe_status,
            qos_profile_action_status_default,
        )

    def mark_submission(self, stamp_ns: int) -> None:
        """Discard previous evidence immediately before sequence goal send."""
        self._checkpoint = ActiveCancelCheckpoint(stamp_ns)

    def close(self) -> None:
        """Release only this observer's subscription on every terminal path."""
        if self._subscription is not None:
            self._node.destroy_subscription(self._subscription)
            self._subscription = None

    def __call__(self, feedback_message) -> None:
        """Replace current phase while preserving the normal feedback log."""
        self._feedback_logger(feedback_message)
        feedback = feedback_message.feedback
        self._checkpoint.observe_feedback(
            sequence_step_index=feedback.sequence_step_index,
            phase=feedback.current_state.phase,
        )

    def _observe_status(self, message: GoalStatusArray) -> None:
        """Replace executing evidence when a terminal or empty snapshot arrives."""
        goals = tuple(
            ControllerGoalSnapshot(
                goal_uuid=bytes(item.goal_info.goal_id.uuid),
                accepted_stamp_ns=(
                    item.goal_info.stamp.sec * 1_000_000_000
                    + item.goal_info.stamp.nanosec
                ),
                status=item.status,
            )
            for item in message.status_list
            if item.goal_info.stamp.sec >= 0
            and 0 <= item.goal_info.stamp.nanosec < 1_000_000_000
        )
        self._checkpoint.observe_status(goals, received_at=time.monotonic())

    def execution_evidence(self) -> ControllerGoalSnapshot | None:
        """Return recent accepted/executing proof, without claiming velocity."""
        return self._checkpoint.execution_evidence(
            now=time.monotonic(), max_status_age_sec=1.0
        )

    @property
    def triggered(self) -> bool:
        """Require both current approach and current controller execution."""
        return self.execution_evidence() is not None


def _wait_for_cancel_checkpoint(
    node: Node,
    feedback: _ActiveCancelFeedback,
    result_future,
    *,
    timeout_sec: float,
    shutdown: ShutdownRequest,
) -> ControllerGoalSnapshot | None:
    """Wait for accepted controller execution or an owned terminal condition."""
    deadline = time.monotonic() + timeout_sec
    while not result_future.done():
        if shutdown.requested:
            return None
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            return None
        evidence = feedback.execution_evidence()
        if evidence is not None:
            return evidence
        rclpy.spin_once(
            node,
            timeout_sec=min(TRANSPORT_WAIT_SLICE_SEC, remaining),
        )
    return None
