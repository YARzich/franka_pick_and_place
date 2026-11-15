"""
Pending RGB-frame coordination for pose inference.

Responsibility:
- Keep a bounded, stamp-keyed queue of RGB frames awaiting synchronized pose inputs.
- Classify queued frames as ready, expired, explicitly empty or dropped by backpressure.
- Track recent successful synchronization for timeout warning policy.

Does not own:
- ROS subscriptions, logging or detection publication.
- Frame decoding, backend selection or inference worker lifecycle.
- The single-flight inference gate.
"""

from __future__ import annotations

from collections import OrderedDict
from collections.abc import Iterable
from dataclasses import dataclass
from typing import Any, Protocol

from manipulation_perception.common.header_sync import HeaderKey


class PoseInputLookup(Protocol):
    """
    Exact-stamp lookup contract for image-associated pose inputs.

    Implementations must match camera info, bbox, mask and depth by both ROS
    timestamp and frame ID; payload arrival order is not a valid fallback.
    """

    def has_empty_bbox_response_for_image(self, image_msg: Any) -> bool:
        ...

    def context_for_image(
        self,
        image_msg: Any,
        *,
        require_depth: bool,
        report_missing: bool,
    ) -> Any | None:
        ...


@dataclass(frozen=True)
class ReadyPoseFrame:
    """One fully synchronized RGB sample admitted for backend inference."""

    key: HeaderKey
    image: Any
    context: Any
    generation: int


@dataclass(frozen=True)
class DroppedPoseFrame:
    """Image generation that must receive an ordered empty output."""

    image: Any
    generation: int


@dataclass(frozen=True)
class PendingPoseFramesResult:
    """Deterministic queue transitions produced by one evaluation pass."""

    ready: ReadyPoseFrame | None
    expired_frames: tuple[DroppedPoseFrame, ...]
    no_detection_frames: tuple[DroppedPoseFrame, ...]
    backpressured_count: int
    should_warn_sync_timeout: bool


class PendingPoseFrames:
    """
    Own bounded pending-frame state and synchronization decisions.

    Sample identity is exact ``stamp + frame_id``. Queue age and sync-health
    windows use caller-supplied monotonic time, so ROS time jumps do not expire
    data unpredictably. The mutable queue assumes serialized node-callback
    access and never starts inference itself.
    """

    def __init__(
        self,
        *,
        input_lookup: PoseInputLookup,
        timeout_sec: float,
        max_size: int = 20,
    ) -> None:
        """Initialize an empty queue with monotonic timeout and capacity policy."""
        self._input_lookup = input_lookup
        self._timeout_sec = max(0.0, float(timeout_sec))
        self._max_size = max(1, int(max_size))
        self._frames: OrderedDict[HeaderKey, tuple[Any, float, int]] = OrderedDict()
        self._last_ready_input_monotonic: float | None = None

    def add(
        self,
        key: HeaderKey,
        image: Any,
        received_at: float,
        generation: int,
    ) -> None:
        """Insert or replace a sample and evict the oldest entry at capacity."""
        self._frames[key] = (image, float(received_at), int(generation))
        while len(self._frames) > self._max_size:
            self._frames.popitem(last=False)

    def evaluate(
        self,
        *,
        now_monotonic: float,
        require_depth: bool,
        worker_running: bool,
    ) -> PendingPoseFramesResult:
        """
        Classify pending samples without blocking for missing inputs.

        At most one ready frame is selected when the worker is idle. Fully
        synchronized frames observed while it is busy are dropped as
        backpressure instead of accumulating stale inference latency.
        """
        expired: list[tuple[HeaderKey, Any, int]] = []
        no_detection: list[tuple[HeaderKey, Any, int]] = []
        backpressured_keys: list[HeaderKey] = []
        ready: ReadyPoseFrame | None = None

        for key, (image, received_at, generation) in list(self._frames.items()):
            if now_monotonic - received_at > self._timeout_sec:
                expired.append((key, image, generation))
                continue
            if self._input_lookup.has_empty_bbox_response_for_image(image):
                no_detection.append((key, image, generation))
                continue

            if ready is not None and not worker_running:
                continue
            context = self._input_lookup.context_for_image(
                image,
                require_depth=require_depth,
                report_missing=False,
            )
            if context is None:
                continue

            self._last_ready_input_monotonic = now_monotonic
            if worker_running:
                backpressured_keys.append(key)
                continue
            ready = ReadyPoseFrame(
                key=key,
                image=image,
                context=context,
                generation=generation,
            )

        self._remove_many(key for key, _, _ in expired)
        self._remove_many(key for key, _, _ in no_detection)
        self._remove_many(backpressured_keys)

        sync_stalled = (
            self._last_ready_input_monotonic is None
            or now_monotonic - self._last_ready_input_monotonic
            > max(10.0, 3.0 * self._timeout_sec)
        )
        return PendingPoseFramesResult(
            ready=ready,
            expired_frames=tuple(
                DroppedPoseFrame(image=image, generation=generation)
                for _, image, generation in expired
            ),
            no_detection_frames=tuple(
                DroppedPoseFrame(image=image, generation=generation)
                for _, image, generation in no_detection
            ),
            backpressured_count=len(backpressured_keys),
            should_warn_sync_timeout=bool(expired) and not worker_running and sync_stalled,
        )

    def remove(self, key: HeaderKey) -> None:
        """Forget one sample after its inference slot has been claimed."""
        self._frames.pop(key, None)

    def keys(self) -> tuple[HeaderKey, ...]:
        """Return sample identities in deterministic insertion order."""
        return tuple(self._frames)

    def clear(self) -> None:
        """Discard all pending samples and reset synchronization health state."""
        self._frames.clear()
        self._last_ready_input_monotonic = None

    def __len__(self) -> int:
        """Return the number of queued RGB frames."""
        return len(self._frames)

    def _remove_many(self, keys: Iterable[HeaderKey]) -> None:
        for key in keys:
            self._frames.pop(key, None)
