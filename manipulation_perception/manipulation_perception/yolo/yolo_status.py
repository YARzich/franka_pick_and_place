"""
Operator-facing status formatting for the YOLO ROS adapter.

Responsibility:
    Format deterministic startup and runtime summaries from normalized YOLO
    configuration and control state.

Does not own:
    Logging, model state, or runtime controls.
"""

from __future__ import annotations

from typing import Any


def format_yolo_startup_status(
    *,
    parameters: Any,
    state: Any,
    detector: Any,
    class_ids_filter: list[int] | None,
    controls_enabled: bool,
    mask_model_available: bool,
) -> str:
    """Build the single startup record used to audit active YOLO settings."""
    return (
        "YOLO visualizer запущен: "
        f"image={parameters.image_topic}, model={parameters.model_path}, "
        f"conf={state.confidence}, iou={state.iou}, device={parameters.device}, "
        f"imgsz={state.imgsz}, class_ids_filter={class_ids_filter}, "
        f"agnostic_nms={state.agnostic_nms}, augment={state.augment}, "
        f"sahi_slice={state.sahi_slice_width}x{state.sahi_slice_height}, "
        f"sahi_overlap={state.sahi_overlap_ratio:.2f}, "
        f"sahi_official={detector.has_sahi_model}, "
        f"sahi_postprocess={parameters.sahi_postprocess_type}/"
        f"{parameters.sahi_postprocess_match_metric}:"
        f"{parameters.sahi_postprocess_match_threshold:.2f}, "
        f"sahi_standard_pred={parameters.sahi_perform_standard_pred}, "
        f"box_area=[{state.min_box_area_ratio:.4f}, {state.max_box_area_ratio:.4f}], "
        f"temporal_confirm_frames={state.temporal_confirm_frames}, "
        f"temporal_iou={state.temporal_iou_threshold:.2f}, "
        f"publish_detection_bbox={parameters.publish_detection_bbox}, "
        f"detection_bbox_topic={parameters.detection_bbox_topic}, "
        f"detection_bbox_stamped_topic={parameters.detection_bbox_stamped_topic}, "
        f"detection_bbox_frame_id={parameters.detection_bbox_frame_id or '<source>'}, "
        f"publish_detection_mask={parameters.publish_detection_mask}, "
        f"detection_mask_topic={parameters.detection_mask_topic}, "
        f"runtime_controls={controls_enabled}, "
        f"use_yolo_world={parameters.use_yolo_world}, "
        f"open_vocab_classes={parameters.open_vocab_classes}, "
        f"mask_model_available={mask_model_available}, "
        f"mask_model_path={parameters.mask_model_path}"
    )


__all__ = ["format_yolo_startup_status"]
