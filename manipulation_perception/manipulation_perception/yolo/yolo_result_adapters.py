"""
Normalize YOLO and SAHI result objects into the detector contract.

Responsibility:
    Extract class, confidence, bbox, mask, and timing data from backend-specific
    result objects into stable `DetectionCandidate` values.

Does not own:
    Inference, threshold policy, or ROS message publication.
"""

from __future__ import annotations

from typing import Any

import numpy as np

from manipulation_perception.common.detection_candidate import DetectionCandidate


def class_name_from_names(names: Any, cls_id: int) -> str:
    """Resolve a class label from mapping/list model metadata."""
    if isinstance(names, dict):
        return str(names.get(cls_id, f"class_{cls_id}"))
    if isinstance(names, (list, tuple)) and 0 <= cls_id < len(names):
        return str(names[cls_id])
    return str(cls_id)


def mask_array_from_tensor(mask_tensor: Any) -> np.ndarray:
    """Expose a supported tensor-like mask as a CPU float32 array."""
    if hasattr(mask_tensor, "detach"):
        mask_tensor = mask_tensor.detach()
    if hasattr(mask_tensor, "cpu"):
        mask_tensor = mask_tensor.cpu()
    if hasattr(mask_tensor, "numpy"):
        return mask_tensor.numpy().astype(np.float32, copy=False)
    return np.asarray(mask_tensor, dtype=np.float32)


def detections_from_yolo_result(
    result: Any,
    offset_x: int = 0,
    offset_y: int = 0,
) -> list[DetectionCandidate]:
    """Convert one Ultralytics result to typed half-open image candidates."""
    names = result.names if hasattr(result, "names") else {}
    boxes = getattr(result, "boxes", None)
    if boxes is None:
        return []

    masks = getattr(result, "masks", None)
    mask_data = getattr(masks, "data", None) if masks is not None else None
    detections: list[DetectionCandidate] = []
    for det_index, box in enumerate(boxes):
        xyxy = box.xyxy[0].tolist()
        cls_id = int(box.cls[0]) if box.cls is not None else -1
        mask_array = None
        if mask_data is not None and det_index < len(mask_data):
            mask_array = mask_array_from_tensor(mask_data[det_index])
        detections.append(
            DetectionCandidate(
                xyxy=(
                    float(xyxy[0]) + float(offset_x),
                    float(xyxy[1]) + float(offset_y),
                    float(xyxy[2]) + float(offset_x),
                    float(xyxy[3]) + float(offset_y),
                ),
                confidence=float(box.conf[0]) if box.conf is not None else 0.0,
                class_id=cls_id,
                class_name=class_name_from_names(names, cls_id),
                mask=mask_array,
            )
        )
    return detections


def detections_from_sahi_result(
    prediction_result: Any,
    active_class_ids_filter: set[int] | list[int] | None,
    max_det: int,
) -> list[DetectionCandidate]:
    """Convert a SAHI prediction while enforcing class/count limits."""
    max_detections = int(max_det)
    if max_detections <= 0:
        return []

    class_filter = set(active_class_ids_filter) if active_class_ids_filter is not None else None
    detections: list[DetectionCandidate] = []
    for obj_prediction in prediction_result.object_prediction_list:
        cls_id = int(obj_prediction.category.id)
        if class_filter is not None and cls_id not in class_filter:
            continue
        x1, y1, x2, y2 = obj_prediction.bbox.to_xyxy()
        detections.append(
            DetectionCandidate(
                xyxy=(float(x1), float(y1), float(x2), float(y2)),
                confidence=float(obj_prediction.score.value),
                class_id=cls_id,
                class_name=str(obj_prediction.category.name),
            )
        )
    detections.sort(key=lambda detection: detection.confidence, reverse=True)
    return detections[:max_detections]
