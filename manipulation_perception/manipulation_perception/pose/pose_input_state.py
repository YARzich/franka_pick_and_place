"""
Input freshness and ROI helpers for pose-estimation nodes.

Responsibility:
- Apply the current bbox/mask freshness policy without depending on rclpy.
- Normalize mask arrays into the binary shape used by GigaPose.
- Select the GigaPose crop bbox while preserving detector fallback behavior.

Does not own:
- ROS message subscriptions or CvBridge decoding.
- Backend worker execution.
- Image/depth file generation.
"""

from __future__ import annotations

import cv2
import numpy as np

from manipulation_perception.common.bbox_geometry import scale_and_clip_bbox
from manipulation_perception.common.bbox_mask import bbox_from_mask


def active_scaled_bbox(
    bbox_xyxy: tuple[int, int, int, int] | None,
    source_wh: tuple[int, int] | None,
    age_sec: float,
    timeout_sec: float,
    frame_height: int,
    frame_width: int,
) -> tuple[int, int, int, int] | None:
    """Return fresh integer half-open crop bounds in the current image size."""
    if bbox_xyxy is None or age_sec > timeout_sec:
        return None
    return scale_and_clip_bbox(bbox_xyxy, frame_width, frame_height, source_wh)


def normalized_binary_mask(mask: np.ndarray, frame_height: int, frame_width: int) -> np.ndarray:
    """Resize a mask with nearest-neighbour sampling and return binary uint8 data."""
    mask_array = np.asarray(mask)
    if mask_array.ndim > 2:
        mask_array = np.squeeze(mask_array)
    if mask_array.shape[:2] != (frame_height, frame_width):
        mask_array = cv2.resize(
            mask_array.astype(np.uint8, copy=False),
            (frame_width, frame_height),
            interpolation=cv2.INTER_NEAREST,
        )
    return (mask_array > 0).astype(np.uint8)


def active_binary_mask(
    mask: np.ndarray | None,
    enabled: bool,
    age_sec: float,
    timeout_sec: float,
    frame_height: int,
    frame_width: int,
) -> np.ndarray | None:
    """Return a fresh normalized mask, or ``None`` when disabled or expired."""
    if not enabled or mask is None or age_sec > timeout_sec:
        return None
    return normalized_binary_mask(mask, frame_height, frame_width)


def select_gigapose_bbox(
    detector_bbox: tuple[int, int, int, int],
    mask: np.ndarray | None,
    image_shape: tuple[int, int],
    *,
    min_pixels: int = 16,
) -> tuple[int, int, int, int]:
    """Prefer mask-derived half-open bounds over the detector crop fallback."""
    mask_bbox = bbox_from_mask(mask, image_shape, min_pixels=min_pixels, end_exclusive=True)
    return detector_bbox if mask_bbox is None else mask_bbox
