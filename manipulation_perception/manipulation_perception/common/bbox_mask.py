"""
Mask-derived bounding boxes and detector-mask rasterization.

Responsibility:
- Derive inclusive or half-open image bounds from binary masks.
- Resize and threshold model masks using the established image semantics.
- Rasterize the best candidate's half-open bbox when no model mask exists.

Does not own:
- ROS message conversion or legacy bbox payload encoding.
- Candidate production, tracking, or freshness state.
- Depth ROI application and general bounding-box scaling.
"""

from __future__ import annotations

from typing import Sequence

import cv2
import numpy as np

from manipulation_perception.common.bbox_geometry import clip_half_open_bbox
from manipulation_perception.common.detection_candidate import DetectionCandidate


def bbox_from_mask(
    mask: np.ndarray | None,
    image_shape: tuple[int, int],
    min_pixels: int = 16,
    end_exclusive: bool = False,
) -> tuple[int, int, int, int] | None:
    """
    Derive an image bbox using the requested endpoint convention.

    With `end_exclusive=True`, maximum edges may equal the image dimensions and
    can be passed directly to NumPy slices. The default preserves legacy
    inclusive payload semantics.
    """
    frame_height, frame_width = image_shape
    if mask is None or frame_height <= 0 or frame_width <= 0:
        return None

    mask_array = np.asarray(mask)
    if mask_array.ndim > 2:
        mask_array = np.squeeze(mask_array)
    if mask_array.ndim != 2:
        return None
    if mask_array.shape[:2] != (frame_height, frame_width):
        mask_array = cv2.resize(
            mask_array.astype(np.uint8, copy=False),
            (frame_width, frame_height),
            interpolation=cv2.INTER_NEAREST,
        )

    ys, xs = np.where(mask_array > 0)
    if len(xs) < max(1, int(min_pixels)):
        return None

    x1 = int(xs.min())
    y1 = int(ys.min())
    x2 = int(xs.max()) + (1 if end_exclusive else 0)
    y2 = int(ys.max()) + (1 if end_exclusive else 0)
    if end_exclusive:
        x1 = max(0, min(frame_width - 1, x1))
        y1 = max(0, min(frame_height - 1, y1))
        x2 = max(x1 + 1, min(frame_width, x2))
        y2 = max(y1 + 1, min(frame_height, y2))
        return x1, y1, x2, y2

    if x2 <= x1 or y2 <= y1:
        return None
    return x1, y1, x2, y2


def mask_from_best_detection(
    detections: Sequence[DetectionCandidate],
    frame_shape: tuple[int, int] | tuple[int, int, int],
    fill_value: int,
) -> np.ndarray:
    """
    Rasterize the highest-confidence candidate using existing mask semantics.

    A model mask is resized to the source image and thresholded at ``0.5``. The
    bbox fallback rasterizes the typed half-open geometry directly; only the
    separate legacy bbox topic retains inclusive endpoint behavior.
    """
    frame_h, frame_w = frame_shape[:2]
    mask = np.zeros((frame_h, frame_w), dtype=np.uint8)
    if not detections:
        return mask

    best = max(detections, key=lambda candidate: candidate.confidence)
    best_mask = best.mask
    if best_mask is not None:
        mask_array = np.asarray(best_mask, dtype=np.float32)
        if mask_array.ndim > 2:
            mask_array = np.squeeze(mask_array)
        if mask_array.shape[:2] != (frame_h, frame_w):
            mask_array = cv2.resize(mask_array, (frame_w, frame_h), interpolation=cv2.INTER_LINEAR)
        mask[mask_array > 0.5] = fill_value
        return mask

    clipped = clip_half_open_bbox(best.xyxy, frame_w, frame_h)
    if clipped is None:
        return mask

    x1, y1, x2, y2 = clipped
    mask[y1:y2, x1:x2] = fill_value
    return mask
