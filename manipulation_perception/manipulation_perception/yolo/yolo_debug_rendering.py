"""
OpenCV drawing helpers for YOLO debug visualization.

Responsibility:
- Draw detections, masks, FPS text and runtime controls onto BGR frames.
- Keep OpenCV drawing details out of the ROS node class.

Does not own:
- OpenCV window creation, imshow/waitKey or mouse callback registration.
- Runtime control value synchronization.
- YOLO/SAHI inference or ROS message publication.
"""

from __future__ import annotations

import cv2
import numpy as np

from manipulation_perception.common.bbox_geometry import clip_half_open_bbox
from manipulation_perception.common.detection_candidate import DetectionCandidate


def draw_mask_overlay(frame_bgr: np.ndarray, mask: np.ndarray, alpha: float) -> None:
    """Blend a resized binary mask and contour into a BGR frame in place."""
    frame_h, frame_w = frame_bgr.shape[:2]
    mask_array = np.asarray(mask, dtype=np.float32)
    if mask_array.ndim > 2:
        mask_array = np.squeeze(mask_array)
    if mask_array.shape[:2] != (frame_h, frame_w):
        mask_array = cv2.resize(mask_array, (frame_w, frame_h), interpolation=cv2.INTER_LINEAR)
    mask_bool = mask_array > 0.5
    if not np.any(mask_bool):
        return

    color = np.array([0, 180, 255], dtype=np.float32)
    overlay_alpha = max(0.05, min(0.95, float(alpha)))
    frame_bgr[mask_bool] = (
        frame_bgr[mask_bool].astype(np.float32) * (1.0 - overlay_alpha) + color * overlay_alpha
    ).astype(np.uint8)
    contours, _ = cv2.findContours(
        mask_bool.astype(np.uint8),
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )
    cv2.drawContours(frame_bgr, contours, -1, (0, 220, 255), 2)


def draw_detections(
    frame_bgr: np.ndarray,
    detections: list[DetectionCandidate],
    *,
    show_masks: bool,
    mask_overlay_alpha: float,
) -> None:
    """Render typed candidates without modifying their masks or metadata."""
    frame_h, frame_w = frame_bgr.shape[:2]
    for det in detections:
        clipped = clip_half_open_bbox(det.xyxy, frame_w, frame_h)
        if clipped is None:
            continue
        x1_i, y1_i, x2_exclusive, y2_exclusive = clipped
        label = f"{det.class_name}: {det.confidence:.2f}"

        has_visible_mask = show_masks and det.mask is not None
        if has_visible_mask:
            draw_mask_overlay(frame_bgr, det.mask, mask_overlay_alpha)
        else:
            cv2.rectangle(
                frame_bgr,
                (x1_i, y1_i),
                (x2_exclusive - 1, y2_exclusive - 1),
                (0, 255, 0),
                2,
            )
        cv2.putText(
            frame_bgr,
            label,
            (x1_i, max(20, y1_i - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )


def draw_fps(frame_bgr: np.ndarray, fps: float) -> None:
    """Draw the current processing rate into a BGR frame in place."""
    cv2.putText(
        frame_bgr,
        f"FPS: {fps:.1f}",
        (10, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )


def render_controls_panel(
    frame_bgr: np.ndarray,
    controls: list[dict],
    controls_layout: list[dict],
    controls_panel_height: int,
) -> np.ndarray:
    """Append a rendered slider panel while leaving control values unchanged."""
    if not controls:
        return frame_bgr

    frame_w = frame_bgr.shape[1]
    panel = np.full((controls_panel_height, frame_w, 3), 36, dtype=np.uint8)

    for row in controls_layout:
        control = controls[int(row["index"])]
        min_value = int(control["min"])
        max_value = int(control["max"])
        value = int(control["value"])
        label = str(control["label"])
        suffix = str(control.get("suffix", "")).strip()
        value_text = f"{value}"
        if suffix:
            value_text += f" {suffix}"
        if control["key"] == "model_variant":
            value_text = "mask" if bool(value) else "bbox"
        text = f"{label}: {value_text}"

        cv2.putText(
            panel,
            text,
            (int(row["label_x"]), int(row["label_y"])),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            (235, 235, 235),
            1,
            cv2.LINE_AA,
        )

        x0 = int(row["track_x0"])
        x1 = int(row["track_x1"])
        y = int(row["track_y"])
        cv2.line(panel, (x0, y), (x1, y), (130, 130, 130), 2)

        ratio = (
            0.0
            if max_value == min_value
            else float(value - min_value) / float(max_value - min_value)
        )
        knob_x = int(round(x0 + ratio * (x1 - x0)))
        cv2.circle(panel, (knob_x, y), 7, (0, 190, 255), -1)
        cv2.circle(panel, (knob_x, y), 7, (0, 90, 120), 1)

    return np.vstack((frame_bgr, panel))
