"""
Bounded state for exact stamped pose-input association.

Responsibility:
    Retain bounded RGB, calibration, bbox, and depth messages by `(stamp,
    frame_id)` and select complete snapshots under explicit freshness rules.

Does not own:
    ROS subscriptions, payload decoding, or inference scheduling.

Message identity always comes from the sensor header `(stamp, frame_id)`.
Receipt age uses the injected ROS clock, so the owning coordinator must clear
this cache when that clock jumps. The cache owns references, not ROS entities
or decoded image arrays.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import auto, Enum
from typing import Any

from manipulation_perception.common.bbox_wire import (
    bbox_from_detection2d_array,
)
from manipulation_perception.common.header_sync import header_key, HeaderKey
from manipulation_perception.pose.pose_input_state import active_scaled_bbox


DEFAULT_MAX_CACHE_SIZE = 20


@dataclass(frozen=True)
class PoseFrameContext:
    """Calibration and half-open detector ROI matched to one exact RGB header."""

    camera_info: Any
    bbox: tuple[int, int, int, int]


@dataclass(frozen=True)
class ActiveStampedMessage:
    """A matching message plus its receipt age at lookup time."""

    message: Any
    age_sec: float


class PoseContextStatus(Enum):
    """Reason why a synchronized inference context is or is not available."""

    READY = auto()
    INVALID_IMAGE_HEADER = auto()
    MISSING_CAMERA_INFO = auto()
    MISSING_BBOX = auto()
    MISSING_DEPTH = auto()


@dataclass(frozen=True)
class PoseContextLookup:
    """Typed synchronization outcome and optional ready inference context."""

    status: PoseContextStatus
    context: PoseFrameContext | None = None


@dataclass(frozen=True)
class _StampedValue:
    value: Any
    received_at: Any


@dataclass(frozen=True)
class _BBoxRecord:
    bbox_xyxy: tuple[int, int, int, int] | None
    source_wh: tuple[int, int] | None
    received_at: Any


class PoseStampedInputCache:
    """
    Own bounded sensor state indexed exclusively by valid header keys.

    The callback-serialized owner provides thread safety; this state object
    intentionally performs no locking of its own.
    """

    def __init__(
        self,
        *,
        clock: Any,
        bbox_timeout_sec: float,
        use_mask: bool,
        depth_enabled: bool,
        max_size: int = DEFAULT_MAX_CACHE_SIZE,
    ) -> None:
        self._clock = clock
        self._bbox_timeout_sec = bbox_timeout_sec
        self._use_mask = use_mask
        self._depth_enabled = depth_enabled
        self._max_size = max(1, int(max_size))
        self._camera_info_by_key: dict[HeaderKey, _StampedValue] = {}
        self._depth_by_key: dict[HeaderKey, _StampedValue] = {}
        self._mask_by_key: dict[HeaderKey, _StampedValue] = {}
        self._bbox_by_key: dict[HeaderKey, _BBoxRecord] = {}

    def clear(self) -> None:
        """Discard every cached input, for example after a ROS clock jump."""
        self._camera_info_by_key.clear()
        self._depth_by_key.clear()
        self._mask_by_key.clear()
        self._bbox_by_key.clear()

    def update_camera_info(self, message: Any) -> None:
        """Store calibration under its exact positive stamp and frame ID."""
        self._store_message(self._camera_info_by_key, message)

    def update_depth(self, message: Any) -> None:
        """Store depth under its exact positive stamp and frame ID."""
        self._store_message(self._depth_by_key, message)

    def update_mask(self, message: Any) -> None:
        """Store an optional mask under its exact positive stamp and frame ID."""
        self._store_message(self._mask_by_key, message)

    def update_bbox_detections(self, message: Any) -> None:
        """Store a stamped bbox response, including explicit empty detection."""
        key = self.message_key(message)
        if key is None:
            return
        bbox = bbox_from_detection2d_array(message)
        self._bbox_by_key[key] = _BBoxRecord(
            bbox_xyxy=bbox.xyxy if bbox is not None else None,
            source_wh=bbox.source_size if bbox is not None else None,
            received_at=self._clock.now(),
        )
        self._prune(self._bbox_by_key)

    def lookup_context(
        self,
        image_message: Any,
        *,
        require_depth: bool,
    ) -> PoseContextLookup:
        """Match required inputs by exact nonzero stamp and nonempty frame id."""
        image_key = self.message_key(image_message)
        if image_key is None:
            return PoseContextLookup(PoseContextStatus.INVALID_IMAGE_HEADER)

        camera_info = self._camera_info_by_key.get(image_key)
        if camera_info is None:
            return PoseContextLookup(PoseContextStatus.MISSING_CAMERA_INFO)

        bbox = self._active_bbox(
            image_message.height,
            image_message.width,
            image_key,
        )
        if bbox is None:
            return PoseContextLookup(PoseContextStatus.MISSING_BBOX)
        if require_depth and self.active_depth(image_key) is None:
            return PoseContextLookup(PoseContextStatus.MISSING_DEPTH)
        return PoseContextLookup(
            PoseContextStatus.READY,
            PoseFrameContext(camera_info=camera_info.value, bbox=bbox),
        )

    def active_depth(self, image_key: HeaderKey) -> ActiveStampedMessage | None:
        """Return fresh depth for the exact image identity when depth is enabled."""
        if not self._depth_enabled:
            return None
        return self._active_message(self._depth_by_key, image_key)

    def active_mask(self, image_key: HeaderKey) -> ActiveStampedMessage | None:
        """Return a fresh mask for the exact image identity when masks are enabled."""
        if not self._use_mask:
            return None
        return self._active_message(self._mask_by_key, image_key)

    def has_empty_bbox_response(self, image_key: HeaderKey) -> bool:
        """Return whether the exact image has a fresh no-detection response."""
        record = self._bbox_by_key.get(image_key)
        return bool(
            record is not None
            and record.bbox_xyxy is None
            and self._age_sec(record.received_at) <= self._bbox_timeout_sec
        )

    @staticmethod
    def message_key(message: Any) -> HeaderKey | None:
        """Extract strict ``stamp + frame_id`` identity from a stamped message."""
        return header_key(getattr(message, "header", None))

    def _store_message(
        self,
        cache: dict[HeaderKey, _StampedValue],
        message: Any,
    ) -> None:
        key = self.message_key(message)
        if key is None:
            return
        cache[key] = _StampedValue(message, self._clock.now())
        self._prune(cache)

    def _active_bbox(
        self,
        frame_height: int,
        frame_width: int,
        image_key: HeaderKey,
    ) -> tuple[int, int, int, int] | None:
        record = self._bbox_by_key.get(image_key)
        if record is None:
            return None
        return active_scaled_bbox(
            record.bbox_xyxy,
            record.source_wh,
            self._age_sec(record.received_at),
            self._bbox_timeout_sec,
            frame_height,
            frame_width,
        )

    def _active_message(
        self,
        cache: dict[HeaderKey, _StampedValue],
        image_key: HeaderKey,
    ) -> ActiveStampedMessage | None:
        record = cache.get(image_key)
        if record is None:
            return None
        age_sec = self._age_sec(record.received_at)
        if age_sec > self._bbox_timeout_sec:
            return None
        return ActiveStampedMessage(record.value, age_sec)

    def _age_sec(self, stamp: Any) -> float:
        return (self._clock.now() - stamp).nanoseconds * 1.0e-9

    def _prune(self, cache: dict[HeaderKey, Any]) -> None:
        while len(cache) > self._max_size:
            del cache[next(iter(cache))]


__all__ = [
    "ActiveStampedMessage",
    "DEFAULT_MAX_CACHE_SIZE",
    "PoseContextLookup",
    "PoseContextStatus",
    "PoseFrameContext",
    "PoseStampedInputCache",
]
