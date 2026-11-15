"""
Stable public facade for bounding-box and mask helpers.

Responsibility:
- Preserve the established ``common.bbox`` import surface.
- Re-export geometry, wire-adapter, and mask helpers from their owner modules.

Does not own:
- Bounding-box calculations or rasterization algorithms.
- ROS graph entities, detector inference, or temporal state.
"""

from manipulation_perception.common.bbox_geometry import (
    apply_bbox_roi,
    clip_half_open_bbox,
    DetectionBBox,
    scale_and_clip_bbox,
)
from manipulation_perception.common.bbox_mask import (
    bbox_from_mask,
    mask_from_best_detection,
)
from manipulation_perception.common.bbox_wire import (
    bbox_from_detection2d_array,
    bbox_payload_from_detection,
    legacy_inclusive_bbox_from_detection,
)


__all__ = (
    "DetectionBBox",
    "apply_bbox_roi",
    "bbox_from_detection2d_array",
    "bbox_from_mask",
    "bbox_payload_from_detection",
    "clip_half_open_bbox",
    "legacy_inclusive_bbox_from_detection",
    "mask_from_best_detection",
    "scale_and_clip_bbox",
)
