"""
Ordered publication for asynchronous pose-estimation output.

Responsibility:
- Serialize concurrent positive and invalidation publications.
- Reject output older than the latest published message.
- Keep same-stamp invalidation authoritative over a later positive result.

Does not own:
- Detection message construction or pose validation.
- ROS publisher creation and destruction.
- Inference worker lifecycle or shutdown initiation.
"""

from __future__ import annotations

from collections.abc import Callable
import threading

from vision_msgs.msg import Detection3DArray

from manipulation_perception.common.header_sync import stamp_to_nanoseconds


class OrderedPoseOutputPublisher:
    """
    Publish pose outputs monotonically across ROS and worker threads.

    Source ROS timestamps define ordering. A generation token invalidates work
    started before a clock reset, while same-stamp empty output wins over a late
    positive inference. The lock covers ordering state and the publisher call,
    making ``publish`` and ``reset`` safe across callback/worker threads.
    """

    def __init__(
        self,
        *,
        publish: Callable[[Detection3DArray], None],
        shutdown_requested: Callable[[], bool],
    ) -> None:
        self._publish = publish
        self._shutdown_requested = shutdown_requested
        self._lock = threading.Lock()
        self._generation = 0
        self._latest_stamp_ns = -1
        self._latest_is_invalidation = False

    def current_generation(self) -> int:
        """Return the clock-epoch token to attach to newly queued work."""
        with self._lock:
            return self._generation

    def reset(self) -> int:
        """Start a new clock epoch and return its generation token."""
        with self._lock:
            self._generation += 1
            self._latest_stamp_ns = -1
            self._latest_is_invalidation = False
            return self._generation

    def publish(
        self,
        message: Detection3DArray,
        *,
        invalidation: bool,
        generation: int,
    ) -> bool:
        """Publish when generation and source-stamp ordering still permit it."""
        stamp_ns = stamp_to_nanoseconds(message.header.stamp)
        if stamp_ns <= 0:
            return False
        with self._lock:
            if self._shutdown_requested():
                return False
            if generation != self._generation:
                return False
            if stamp_ns < self._latest_stamp_ns:
                return False
            if (
                stamp_ns == self._latest_stamp_ns
                and self._latest_is_invalidation
                and not invalidation
            ):
                return False

            self._publish(message)
            self._latest_stamp_ns = stamp_ns
            self._latest_is_invalidation = invalidation
            return True
