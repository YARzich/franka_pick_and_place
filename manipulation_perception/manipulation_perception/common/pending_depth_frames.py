"""
Bounded queue for depth frames awaiting stamped detection responses.

Responsibility:
- Keep depth frames keyed by exact ``stamp + frame_id``.
- Keep each frame's resolved calibration immutable while it waits.
- Release frames when a matching detector response arrives or waiting expires.
- Apply deterministic backpressure while preserving insertion order.

Does not own:
- ROS clocks, subscriptions, publishers or logging.
- Detection bbox storage and freshness decisions.
- Depth masking, point cloud generation or frame transforms.
"""

from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass
from typing import Any, Protocol

import numpy as np

from manipulation_perception.common.header_sync import header_key, HeaderKey


Intrinsics = tuple[float | None, float | None, float | None, float | None]
_MISSING_INTRINSICS: Intrinsics = (None, None, None, None)


class DetectionResponseLookup(Protocol):
    """
    Read-only detector-response view used by the pending-frame queue.

    Implementations must match a response by the source header's exact ROS
    timestamp and frame ID. ``now_sec`` is ROS time and is used only for the
    implementation's response-freshness policy.
    """

    @property
    def enabled(self) -> bool:
        ...

    def has_fresh_response_for_header(self, source_header: Any, now_sec: float) -> bool:
        ...


@dataclass(frozen=True)
class PendingDepthFrame:
    """
    One depth sample and calibration retained until detection catches up.

    ``received_at_monotonic`` is a steady-clock value used for queue expiry;
    the ROS sample identity remains in ``source_header``.
    """

    depth: np.ndarray
    source_header: Any
    intrinsics: Intrinsics
    received_at_monotonic: float


@dataclass(frozen=True)
class PendingDepthAddResult:
    """Outcome of applying queue admission and bounded backpressure."""

    queued: bool
    dropped: tuple[PendingDepthFrame, ...]


class PendingDepthFrames:
    """Own bounded depth-frame state while the detector catches up."""

    def __init__(
        self,
        *,
        response_lookup: DetectionResponseLookup,
        timeout_sec: float,
        max_size: int = 20,
    ) -> None:
        self._response_lookup = response_lookup
        self._timeout_sec = max(0.0, float(timeout_sec))
        self._max_size = max(1, int(max_size))
        self._frames: OrderedDict[HeaderKey, PendingDepthFrame] = OrderedDict()

    def add_if_waiting(
        self,
        depth: np.ndarray,
        source_header: Any,
        *,
        now_ros_sec: float,
        received_at_monotonic: float,
        intrinsics: Intrinsics = _MISSING_INTRINSICS,
    ) -> PendingDepthAddResult:
        """Queue a frame and its immutable sample calibration while detection waits."""
        if not self._response_lookup.enabled:
            return PendingDepthAddResult(queued=False, dropped=())

        key = header_key(source_header)
        if key is None:
            return PendingDepthAddResult(queued=False, dropped=())
        if self._response_lookup.has_fresh_response_for_header(source_header, now_ros_sec):
            return PendingDepthAddResult(queued=False, dropped=())

        self._frames[key] = PendingDepthFrame(
            depth=depth,
            source_header=source_header,
            intrinsics=intrinsics,
            received_at_monotonic=float(received_at_monotonic),
        )
        self._frames.move_to_end(key)
        dropped: list[PendingDepthFrame] = []
        while len(self._frames) > self._max_size:
            _, dropped_frame = self._frames.popitem(last=False)
            dropped.append(dropped_frame)
        return PendingDepthAddResult(queued=True, dropped=tuple(dropped))

    def release_ready(
        self,
        *,
        now_ros_sec: float,
        now_monotonic: float,
    ) -> tuple[PendingDepthFrame, ...]:
        """Release frames matched in ROS time or expired in monotonic time."""
        ready: list[PendingDepthFrame] = []
        for key, frame in list(self._frames.items()):
            expired = (
                float(now_monotonic) - frame.received_at_monotonic > self._timeout_sec
            )
            has_matching_response = self._response_lookup.has_fresh_response_for_header(
                frame.source_header,
                now_ros_sec,
            )
            if not has_matching_response and not expired:
                continue
            del self._frames[key]
            ready.append(frame)
        return tuple(ready)

    def keys(self) -> tuple[HeaderKey, ...]:
        """Return queued sample identities in deterministic insertion order."""
        return tuple(self._frames)

    def clear(self) -> None:
        """Discard all borrowed depth samples without publishing them."""
        self._frames.clear()

    def __len__(self) -> int:
        """Return the number of queued depth frames."""
        return len(self._frames)
