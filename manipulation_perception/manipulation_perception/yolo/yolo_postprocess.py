"""
YOLO detection post-processing helpers.

Responsibility:
- Apply filtering, NMS and temporal confirmation rules.
- Preserve historical result-adapter imports for internal callers.

Does not own:
- Model loading or inference.
- YOLO/SAHI object normalization, owned by `yolo_result_adapters`.
- OpenCV windows, mouse controls or debug rendering lifecycle.
- ROS image subscriptions or bbox/mask publication.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, replace

import cv2
import numpy as np

from manipulation_perception.common.detection_candidate import DetectionCandidate
from manipulation_perception.yolo.yolo_result_adapters import (
    class_name_from_names,
    detections_from_sahi_result,
    detections_from_yolo_result,
    mask_array_from_tensor,
)


__all__ = [
    "apply_temporal_filter",
    "class_name_from_names",
    "detections_from_sahi_result",
    "detections_from_yolo_result",
    "filter_by_box_area",
    "iou_xyxy",
    "mask_array_from_tensor",
    "nms_detections",
    "normalize_indices",
    "tile_starts",
    "TemporalDetectionTrack",
]


def iou_xyxy(box_a: Sequence[float], box_b: Sequence[float]) -> float:
    """Return IoU for two continuous half-open ``xyxy`` rectangles."""
    ax1, ay1, ax2, ay2 = box_a
    bx1, by1, bx2, by2 = box_b
    inter_x1 = max(ax1, bx1)
    inter_y1 = max(ay1, by1)
    inter_x2 = min(ax2, bx2)
    inter_y2 = min(ay2, by2)
    inter_w = max(0.0, inter_x2 - inter_x1)
    inter_h = max(0.0, inter_y2 - inter_y1)
    inter_area = inter_w * inter_h
    area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    union = area_a + area_b - inter_area
    if union <= 1.0e-9:
        return 0.0
    return inter_area / union


def normalize_indices(indices) -> list[int]:
    """Normalize OpenCV NMS index container variants to scalar integers."""
    if indices is None:
        return []
    if len(indices) == 0:
        return []
    normalized: list[int] = []
    for idx in indices:
        if isinstance(idx, (list, tuple, np.ndarray)):
            normalized.append(int(idx[0]))
        else:
            normalized.append(int(idx))
    return normalized


def tile_starts(full_size: int, win_size: int, step: int) -> list[int]:
    """Return tile origins that always cover the final image boundary."""
    if win_size >= full_size:
        return [0]
    starts = list(range(0, max(1, full_size - win_size + 1), max(1, step)))
    last_start = full_size - win_size
    if starts[-1] != last_start:
        starts.append(last_start)
    return starts


def filter_by_box_area(
    detections: list[DetectionCandidate],
    frame_shape: tuple[int, int] | tuple[int, int, int],
    min_box_area_ratio: float,
    max_box_area_ratio: float,
) -> list[DetectionCandidate]:
    """Keep candidates whose half-open area lies inside configured ratios."""
    h, w = frame_shape[:2]
    frame_area = float(max(1, h * w))
    filtered: list[DetectionCandidate] = []
    for det in detections:
        x1, y1, x2, y2 = det.xyxy
        box_w = max(0.0, x2 - x1)
        box_h = max(0.0, y2 - y1)
        area_ratio = (box_w * box_h) / frame_area
        if area_ratio < min_box_area_ratio:
            continue
        if area_ratio > max_box_area_ratio:
            continue
        filtered.append(det)
    return filtered


def nms_detections(
    detections: list[DetectionCandidate],
    iou_threshold: float,
    max_det: int,
    agnostic_nms: bool,
) -> list[DetectionCandidate]:
    """Apply OpenCV NMS without losing typed candidate metadata."""
    max_detections = int(max_det)
    if max_detections <= 0:
        return []
    if len(detections) <= 1:
        return detections[:max_detections]

    def run_nms(det_subset: list[DetectionCandidate]) -> list[DetectionCandidate]:
        boxes_xywh = []
        scores = []
        for det in det_subset:
            x1, y1, x2, y2 = det.xyxy
            boxes_xywh.append(
                [
                    int(round(x1)),
                    int(round(y1)),
                    int(round(max(1.0, x2 - x1))),
                    int(round(max(1.0, y2 - y1))),
                ]
            )
            scores.append(det.confidence)
        indices = cv2.dnn.NMSBoxes(
            boxes_xywh,
            scores,
            score_threshold=0.0,
            nms_threshold=float(iou_threshold),
            top_k=max_detections,
        )
        kept = normalize_indices(indices)
        return [det_subset[i] for i in kept]

    if agnostic_nms:
        return run_nms(detections)

    by_class: dict[int, list[DetectionCandidate]] = {}
    for det in detections:
        by_class.setdefault(det.class_id, []).append(det)
    merged: list[DetectionCandidate] = []
    for class_dets in by_class.values():
        merged.extend(run_nms(class_dets))
    merged.sort(key=lambda det: det.confidence, reverse=True)
    return merged[:max_detections]


@dataclass(frozen=True, slots=True)
class TemporalDetectionTrack:
    """Immutable temporal state for one class-consistent candidate track."""

    candidate: DetectionCandidate
    hits: int
    age: int


def apply_temporal_filter(
    tracks: list[TemporalDetectionTrack],
    detections: list[DetectionCandidate],
    confirm_frames: int,
    iou_threshold: float,
    max_age: int,
    max_det: int,
) -> tuple[list[TemporalDetectionTrack], list[DetectionCandidate]]:
    """Update immutable tracks and return candidates confirmed this frame."""
    max_detections = int(max_det)
    if max_detections <= 0:
        return tracks, []
    if confirm_frames <= 1:
        if len(detections) <= max_detections:
            return [], detections
        return [], detections[:max_detections]

    tracks = [replace(track, age=track.age + 1) for track in tracks]

    used_tracks: set[int] = set()
    detections_sorted = sorted(detections, key=lambda det: det.confidence, reverse=True)
    for det in detections_sorted:
        best_idx = -1
        best_iou = 0.0
        for idx, track in enumerate(tracks):
            if idx in used_tracks:
                continue
            if track.candidate.class_id != det.class_id:
                continue
            iou_val = iou_xyxy(track.candidate.xyxy, det.xyxy)
            if iou_val >= iou_threshold and iou_val > best_iou:
                best_iou = iou_val
                best_idx = idx

        if best_idx >= 0:
            track = tracks[best_idx]
            tracks[best_idx] = TemporalDetectionTrack(
                candidate=det,
                hits=min(track.hits + 1, confirm_frames),
                age=0,
            )
            used_tracks.add(best_idx)
        else:
            tracks.append(
                TemporalDetectionTrack(candidate=det, hits=1, age=0)
            )
            used_tracks.add(len(tracks) - 1)

    tracks = [track for track in tracks if track.age <= max_age]
    confirmed = [
        track.candidate
        for track in tracks
        if track.hits >= confirm_frames and track.age == 0
    ]
    confirmed.sort(key=lambda det: det.confidence, reverse=True)
    return tracks, confirmed[:max_detections]
