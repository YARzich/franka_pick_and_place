"""
Stamped detection bbox ROI state for the stereo perception node.

Responsibility:
- Keep stamped detector bbox responses with receive times.
- Check stamped bbox/depth frame consistency before applying ROI.
- Check bbox freshness against the configured timeout.
- Apply the active bbox as a NaN mask to depth frames.

Does not own:
- ROS subscriptions, clocks or logging.
- Detector inference or bbox message publication.
- Point cloud projection after ROI masking.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from manipulation_perception.common.bbox_geometry import (
    apply_bbox_roi,
    DetectionBBox,
    scale_and_clip_bbox,
)
from manipulation_perception.common.bbox_wire import bbox_from_detection2d_array
from manipulation_perception.common.header_sync import header_key, HeaderKey


@dataclass(frozen=True)
class DetectionRoiResult:
    """ROI-masked depth plus optional integer half-open bounds."""

    depth: np.ndarray
    bbox: tuple[int, int, int, int] | None
    missing_required_bbox: bool


@dataclass(frozen=True)
class _BBoxRecord:
    bbox: DetectionBBox | None
    received_sec: float


class DetectionRoiFilter:
    """
    Own bounded ROI responses and fail closed for unmatched depth frames.

    Stamped responses are keyed by exact ROS ``stamp + frame_id``. Freshness
    uses the caller-provided ROS time; the class is mutable and assumes
    serialized access from the owning node callback group.
    """

    def __init__(self, enabled: bool, timeout_sec: float) -> None:
        self._enabled = bool(enabled)
        self._timeout_sec = max(0.05, float(timeout_sec))
        self._bbox_by_key: dict[HeaderKey, _BBoxRecord] = {}

    @property
    def enabled(self) -> bool:
        """Return whether depth output requires a matching detector response."""
        return self._enabled

    def update_detection_array(self, msg, received_sec: float) -> None:
        """Store one stamped detector response, including an explicit no-detection."""
        key = header_key(getattr(msg, "header", None))
        if key is None:
            return
        self._bbox_by_key[key] = _BBoxRecord(
            bbox=bbox_from_detection2d_array(msg),
            received_sec=float(received_sec),
        )
        while len(self._bbox_by_key) > 20:
            oldest_key = next(iter(self._bbox_by_key))
            del self._bbox_by_key[oldest_key]

    def active_bbox(
        self,
        frame_height: int,
        frame_width: int,
        now_sec: float,
        source_header=None,
    ) -> tuple[int, int, int, int] | None:
        """Return fresh half-open pixel bounds for the requested depth sample."""
        if not self._enabled:
            return None

        record = self._record_for_header(source_header)
        if record is None or record.bbox is None:
            return None

        age_sec = float(now_sec) - record.received_sec
        if age_sec > self._timeout_sec:
            return None

        return scale_and_clip_bbox(
            record.bbox,
            frame_width,
            frame_height,
        )

    def has_fresh_bbox_for_header(self, source_header, now_sec: float) -> bool:
        """Return whether the exact sample has a fresh, non-empty bbox."""
        if not self._enabled:
            return True
        record = self._record_for_header(source_header)
        return (
            record is not None
            and record.bbox is not None
            and float(now_sec) - record.received_sec <= self._timeout_sec
        )

    def has_fresh_response_for_header(self, source_header, now_sec: float) -> bool:
        """Return whether the exact sample has any fresh detector response."""
        if not self._enabled:
            return True
        record = self._record_for_header(source_header)
        return record is not None and float(now_sec) - record.received_sec <= self._timeout_sec

    def apply(self, depth: np.ndarray, now_sec: float, source_header=None) -> DetectionRoiResult:
        """
        Mask depth outside the matched bbox, or all depth when none is valid.

        The fail-closed all-``NaN`` result prevents an unmatched full-frame
        cloud from entering Octomap when ROI filtering is enabled.
        """
        if not self._enabled:
            return DetectionRoiResult(depth, None, False)

        frame_height, frame_width = depth.shape[:2]
        bbox = self.active_bbox(frame_height, frame_width, now_sec, source_header)
        if bbox is None:
            empty_depth = np.full_like(depth, np.nan, dtype=np.float32)
            return DetectionRoiResult(empty_depth, None, True)

        return DetectionRoiResult(apply_bbox_roi(depth, bbox), bbox, False)

    def _record_for_header(self, source_header) -> _BBoxRecord | None:
        if source_header is None:
            return None

        key = header_key(source_header)
        if key is None:
            return None
        return self._bbox_by_key.get(key)
