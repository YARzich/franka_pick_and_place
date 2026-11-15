"""
Log meaningful feedback transitions from either transfer-sequence action.

Responsibility:
- Suppress heartbeat duplicates and present concise phase/progress changes.

Does not own:
- Action transport, goal construction, result policy, or process lifecycle.
"""

from __future__ import annotations

from rclpy.node import Node


class FeedbackLogger:
    """Log only meaningful phase changes instead of every heartbeat."""

    def __init__(self, node: Node) -> None:
        """Bind feedback display to one client logger."""
        self._logger = node.get_logger()
        self._last_key: tuple[object, ...] | None = None

    def __call__(self, feedback_message) -> None:
        """Emit concise sequence index, phase, progress, and operator detail."""
        feedback = feedback_message.feedback
        state = feedback.current_state
        key = (
            feedback.sequence_step_index,
            feedback.active_object_id,
            feedback.waiting_reason,
            state.phase,
            state.paused,
            state.pause_pending,
            state.detail,
        )
        if key == self._last_key:
            return
        self._last_key = key
        self._logger.info(
            "step %d/%d object=%s phase=%d waiting=%d progress=%.0f%%: %s"
            % (
                feedback.sequence_step_index + 1,
                feedback.sequence_step_count,
                feedback.active_object_id or "<none>",
                state.phase,
                feedback.waiting_reason,
                feedback.overall_progress * 100.0,
                state.detail,
            )
        )
