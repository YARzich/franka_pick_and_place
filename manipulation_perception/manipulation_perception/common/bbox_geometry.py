"""
Pure half-open bounding-box geometry and depth ROI helpers.

Responsibility:
- Represent typed integer half-open image bounds.
- Scale, rasterize, and clip rectangular geometry to image slice bounds.
- Apply already validated slice bounds to depth arrays.

Does not own:
- ROS or legacy wire-message conversion.
- Detector masks, candidate selection, or image resizing.
- Detection freshness and frame/timestamp correlation.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np


@dataclass(frozen=True)
class DetectionBBox:
    """Integer half-open image bounds plus optional source-image metadata."""

    xyxy: tuple[int, int, int, int]
    source_size: tuple[int, int] | None = None
    class_id: int | None = None
    score: float | None = None


def scale_and_clip_bbox(
    bbox: DetectionBBox | Sequence[int | float],
    frame_width: int,
    frame_height: int,
    source_size: tuple[int, int] | None = None,
) -> tuple[int, int, int, int] | None:
    """
    Scale and clip internal half-open bbox geometry to image slice bounds.

    `DetectionBBox` values have already crossed their wire adapter. When a
    source size is present, continuous edges are scaled before rasterization so
    the same covered source pixels remain represented in the target image.
    """
    if frame_width <= 0 or frame_height <= 0:
        return None

    if isinstance(bbox, DetectionBBox):
        x1, y1, x2, y2 = bbox.xyxy
        source_size = bbox.source_size
    else:
        x1, y1, x2, y2 = bbox

    if source_size is not None:
        src_w, src_h = source_size
        if src_w > 0 and src_h > 0 and (src_w != frame_width or src_h != frame_height):
            scale_x = float(frame_width) / float(src_w)
            scale_y = float(frame_height) / float(src_h)
            x1 = float(x1) * scale_x
            x2 = float(x2) * scale_x
            y1 = float(y1) * scale_y
            y2 = float(y2) * scale_y

    return clip_half_open_bbox((x1, y1, x2, y2), frame_width, frame_height)


def clip_half_open_bbox(
    bbox: Sequence[int | float],
    frame_width: int,
    frame_height: int,
) -> tuple[int, int, int, int] | None:
    """
    Rasterize and clip continuous half-open geometry to image slice bounds.

    Minimum edges use ``floor`` and maximum edges use ``ceil`` so every pixel
    touched by the continuous rectangle is retained. Returned maximum edges may
    equal ``frame_width``/``frame_height`` and are intended for NumPy slices.
    """
    if frame_width <= 0 or frame_height <= 0 or len(bbox) != 4:
        return None
    x1_raw, y1_raw, x2_raw, y2_raw = (float(value) for value in bbox)
    if not all(math.isfinite(value) for value in (x1_raw, y1_raw, x2_raw, y2_raw)):
        return None
    x1 = max(0, min(frame_width, math.floor(x1_raw)))
    y1 = max(0, min(frame_height, math.floor(y1_raw)))
    x2 = max(0, min(frame_width, math.ceil(x2_raw)))
    y2 = max(0, min(frame_height, math.ceil(y2_raw)))
    if x2 <= x1 or y2 <= y1:
        return None
    return x1, y1, x2, y2


def apply_bbox_roi(depth: np.ndarray, bbox: tuple[int, int, int, int] | None) -> np.ndarray:
    """Mask depth outside integer half-open slice bounds."""
    roi_depth = np.full_like(depth, np.nan, dtype=np.float32)
    if bbox is None:
        return roi_depth

    x1, y1, x2, y2 = bbox
    roi_depth[y1:y2, x1:x2] = depth[y1:y2, x1:x2]
    return roi_depth
