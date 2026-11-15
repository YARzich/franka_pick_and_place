"""
Adapters between perception detections and bounding-box wire representations.

Responsibility:
- Decode stamped ``Detection2DArray``-like messages into typed half-open bounds.
- Isolate the historical inclusive ``Int32MultiArray`` endpoint convention.
- Encode detector candidates into the established compact legacy payload.

Does not own:
- Half-open clipping, scaling, or depth ROI operations.
- ROS subscriptions, publishers, and message synchronization.
- Detector inference or mask rasterization.
"""

from __future__ import annotations

import math

from manipulation_perception.common.bbox_geometry import DetectionBBox
from manipulation_perception.common.detection_candidate import DetectionCandidate


def bbox_from_detection2d_array(msg) -> DetectionBBox | None:
    """Rasterize the best continuous Detection2D bbox as half-open slice bounds."""
    detections = list(getattr(msg, "detections", []))
    if not detections:
        return None

    def detection_score(detection) -> float:
        results = list(getattr(detection, "results", []))
        if not results:
            return 0.0
        return max(float(getattr(result.hypothesis, "score", 0.0)) for result in results)

    detection = max(detections, key=detection_score)
    bbox = detection.bbox
    center_x = float(bbox.center.position.x)
    center_y = float(bbox.center.position.y)
    half_width = 0.5 * float(bbox.size_x)
    half_height = 0.5 * float(bbox.size_y)
    if not all(math.isfinite(value) for value in (center_x, center_y, half_width, half_height)):
        return None
    if half_width <= 0.0 or half_height <= 0.0:
        return None

    x1 = math.floor(center_x - half_width)
    y1 = math.floor(center_y - half_height)
    x2 = math.ceil(center_x + half_width)
    y2 = math.ceil(center_y + half_height)
    if x2 <= x1 or y2 <= y1:
        return None

    class_id = None
    score = None
    results = list(getattr(detection, "results", []))
    if results:
        best_result = max(results, key=lambda result: float(getattr(result.hypothesis, "score", 0.0)))
        class_text = str(getattr(best_result.hypothesis, "class_id", "")).strip()
        if class_text:
            try:
                class_id = int(class_text)
            except ValueError:
                class_id = None
        score = float(getattr(best_result.hypothesis, "score", 0.0))
    return DetectionBBox((x1, y1, x2, y2), None, class_id, score)


def legacy_inclusive_bbox_from_detection(
    detection: DetectionCandidate,
    frame_shape: tuple[int, int] | tuple[int, int, int],
) -> tuple[int, int, int, int] | None:
    """
    Reproduce the historical numeric endpoint conversion for the wire API.

    The legacy `Int32MultiArray` contract copied/truncated detector maximum
    coordinates and older consumers interpreted them as inclusive indices. That
    one-pixel ambiguity is deliberately isolated here instead of leaking into
    stamped messages, masks, pose crops or depth ROI geometry.
    """
    frame_h, frame_w = frame_shape[:2]
    if frame_w <= 0 or frame_h <= 0:
        return None

    x1, y1, x2, y2 = (int(value) for value in detection.xyxy)
    x1 = max(0, min(frame_w - 1, x1))
    y1 = max(0, min(frame_h - 1, y1))
    x2 = max(0, min(frame_w - 1, x2))
    y2 = max(0, min(frame_h - 1, y2))
    if x2 <= x1 or y2 <= y1:
        return None
    return x1, y1, x2, y2


def bbox_payload_from_detection(
    detection: DetectionCandidate,
    frame_shape: tuple[int, int] | tuple[int, int, int],
) -> list[int] | None:
    """
    Encode one candidate with the historical inclusive integer endpoint.

    ``DetectionCandidate.xyxy`` is half-open continuous geometry. This adapter
    intentionally preserves the existing legacy payload conversion: endpoint
    values are truncated/clipped to valid pixel indices and consumed as
    inclusive bounds by older ROI subscribers.
    """
    frame_h, frame_w = frame_shape[:2]
    clipped = legacy_inclusive_bbox_from_detection(detection, frame_shape)
    if clipped is None:
        return None

    x1, y1, x2, y2 = clipped
    return [
        x1,
        y1,
        x2,
        y2,
        int(frame_w),
        int(frame_h),
        detection.class_id,
        int(round(detection.confidence * 1000.0)),
    ]
