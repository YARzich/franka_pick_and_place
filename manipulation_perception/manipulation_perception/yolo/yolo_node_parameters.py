"""
ROS parameter declaration and reading for yolo_visualizer_node.

Responsibility:
- Declare the public ROS parameter API of the YOLO visualizer node.
- Read and normalize raw parameter values into a small immutable object.
- Preserve current launch/YAML override names and fallback defaults.

Does not own:
- YOLO/SAHI model loading or inference.
- Runtime OpenCV controls.
- ROS publishers, subscriptions or callbacks.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from manipulation_perception.common.config_validation import (
    require_frame_id,
    require_non_empty,
)
from manipulation_perception.common.parameter_utils import bool_value


@dataclass(frozen=True)
class YoloVisualizerParameters:
    """
    Normalized inference, transport and optional debug-window configuration.

    Thresholds and ratios are unitless; slice/image dimensions and bboxes use
    pixels. ``detection_bbox_frame_id`` is metadata only and must identify the
    actual source image frame, not a frame requiring TF transformation.
    """

    image_topic: str
    model_path: str
    mask_model_path: str
    confidence: float
    iou: float
    device: str
    imgsz: int
    max_det: int
    run_every_n_frames: int
    window_name: str
    show_fps: bool
    enable_window: bool
    enable_runtime_controls: bool
    agnostic_nms: bool
    augment: bool
    sahi_slice_width: int
    sahi_slice_height: int
    sahi_overlap_ratio: float
    sahi_perform_standard_pred: bool
    sahi_auto_slice_resolution: bool
    sahi_postprocess_type: str
    sahi_postprocess_match_metric: str
    sahi_postprocess_match_threshold: float
    sahi_postprocess_class_agnostic: bool
    min_box_area_ratio: float
    max_box_area_ratio: float
    temporal_confirm_frames: int
    temporal_iou_threshold: float
    class_whitelist: list[str]
    publish_detection_bbox: bool
    detection_bbox_topic: str
    detection_bbox_stamped_topic: str
    detection_bbox_frame_id: str
    publish_detection_mask: bool
    detection_mask_topic: str
    detection_mask_fill_value: int
    use_yolo_world: bool
    open_vocab_classes: list[str]
    show_masks: bool
    mask_overlay_alpha: float


def declare_yolo_visualizer_parameters(node: Any) -> None:
    """Declare backend-neutral defaults and empty scenario-owned values."""
    node.declare_parameter("image_topic", "")
    node.declare_parameter("model_path", "")
    node.declare_parameter("mask_model_path", "")
    node.declare_parameter("confidence_threshold", 0.35)
    node.declare_parameter("iou_threshold", 0.01)
    node.declare_parameter("device", "0")
    node.declare_parameter("imgsz", 640)
    node.declare_parameter("max_det", 30)
    node.declare_parameter("run_every_n_frames", 1)
    node.declare_parameter("window_name", "YOLO Camera Debug")
    node.declare_parameter("show_fps", True)
    node.declare_parameter("enable_window", True)
    node.declare_parameter("enable_runtime_controls", True)
    node.declare_parameter("agnostic_nms", False)
    node.declare_parameter("augment", False)
    node.declare_parameter("sahi_slice_width", 224)
    node.declare_parameter("sahi_slice_height", 256)
    node.declare_parameter("sahi_overlap_ratio", 0.20)
    node.declare_parameter("sahi_perform_standard_pred", True)
    node.declare_parameter("sahi_auto_slice_resolution", False)
    node.declare_parameter("sahi_postprocess_type", "GREEDYNMM")
    node.declare_parameter("sahi_postprocess_match_metric", "IOS")
    node.declare_parameter("sahi_postprocess_match_threshold", 0.5)
    node.declare_parameter("sahi_postprocess_class_agnostic", False)
    node.declare_parameter("min_box_area_ratio", 0.0001)
    node.declare_parameter("max_box_area_ratio", 0.35)
    node.declare_parameter("temporal_confirm_frames", 1)
    node.declare_parameter("temporal_iou_threshold", 0.5)
    node.declare_parameter("class_whitelist", "")
    node.declare_parameter("publish_detection_bbox", True)
    node.declare_parameter("detection_bbox_topic", "")
    node.declare_parameter("detection_bbox_stamped_topic", "")
    node.declare_parameter("detection_bbox_frame_id", "")
    node.declare_parameter("publish_detection_mask", False)
    node.declare_parameter("detection_mask_topic", "")
    node.declare_parameter("detection_mask_fill_value", 255)
    node.declare_parameter("use_yolo_world", False)
    node.declare_parameter("open_vocab_classes", "")
    node.declare_parameter("show_masks", True)
    node.declare_parameter("mask_overlay_alpha", 0.45)


def _parameter_value(node: Any, name: str) -> Any:
    return node.get_parameter(name).value


def _comma_tokens(value: str) -> list[str]:
    return [item.strip() for item in str(value).split(",") if item.strip()]


def _bool_parameter(node: Any, name: str) -> bool:
    return bool_value(_parameter_value(node, name), name)


def read_yolo_visualizer_parameters(node: Any) -> YoloVisualizerParameters:
    """Read one immutable snapshot and clamp all public numeric bounds."""
    confidence = max(0.01, min(1.0, float(_parameter_value(node, "confidence_threshold"))))
    iou = max(0.01, min(1.0, float(_parameter_value(node, "iou_threshold"))))
    imgsz = max(32, int(_parameter_value(node, "imgsz")))
    max_det = max(1, int(_parameter_value(node, "max_det")))
    sahi_overlap_ratio = max(0.0, min(0.8, float(_parameter_value(node, "sahi_overlap_ratio"))))
    sahi_postprocess_type = str(_parameter_value(node, "sahi_postprocess_type")).strip().upper()
    if sahi_postprocess_type not in {"GREEDYNMM", "NMS", "NMM", "LSNMS"}:
        sahi_postprocess_type = "GREEDYNMM"

    sahi_postprocess_match_metric = (
        str(_parameter_value(node, "sahi_postprocess_match_metric")).strip().upper()
    )
    if sahi_postprocess_match_metric not in {"IOS", "IOU"}:
        sahi_postprocess_match_metric = "IOS"

    sahi_postprocess_match_threshold = max(
        0.01,
        min(0.99, float(_parameter_value(node, "sahi_postprocess_match_threshold"))),
    )
    min_box_area_ratio = max(0.0, min(1.0, float(_parameter_value(node, "min_box_area_ratio"))))
    max_box_area_ratio = max(
        min_box_area_ratio,
        min(1.0, float(_parameter_value(node, "max_box_area_ratio"))),
    )
    temporal_iou_threshold = max(
        0.05,
        min(0.95, float(_parameter_value(node, "temporal_iou_threshold"))),
    )
    mask_overlay_alpha = max(0.05, min(0.95, float(_parameter_value(node, "mask_overlay_alpha"))))

    return YoloVisualizerParameters(
        image_topic=str(_parameter_value(node, "image_topic")),
        model_path=str(_parameter_value(node, "model_path")),
        mask_model_path=str(_parameter_value(node, "mask_model_path")).strip(),
        confidence=confidence,
        iou=iou,
        device=str(_parameter_value(node, "device")).strip(),
        imgsz=imgsz,
        max_det=max_det,
        run_every_n_frames=max(1, int(_parameter_value(node, "run_every_n_frames"))),
        window_name=str(_parameter_value(node, "window_name")),
        show_fps=_bool_parameter(node, "show_fps"),
        enable_window=_bool_parameter(node, "enable_window"),
        enable_runtime_controls=_bool_parameter(node, "enable_runtime_controls"),
        agnostic_nms=_bool_parameter(node, "agnostic_nms"),
        augment=_bool_parameter(node, "augment"),
        sahi_slice_width=max(64, int(_parameter_value(node, "sahi_slice_width"))),
        sahi_slice_height=max(64, int(_parameter_value(node, "sahi_slice_height"))),
        sahi_overlap_ratio=sahi_overlap_ratio,
        sahi_perform_standard_pred=_bool_parameter(node, "sahi_perform_standard_pred"),
        sahi_auto_slice_resolution=_bool_parameter(node, "sahi_auto_slice_resolution"),
        sahi_postprocess_type=sahi_postprocess_type,
        sahi_postprocess_match_metric=sahi_postprocess_match_metric,
        sahi_postprocess_match_threshold=sahi_postprocess_match_threshold,
        sahi_postprocess_class_agnostic=_bool_parameter(node, "sahi_postprocess_class_agnostic"),
        min_box_area_ratio=min_box_area_ratio,
        max_box_area_ratio=max_box_area_ratio,
        temporal_confirm_frames=max(1, int(_parameter_value(node, "temporal_confirm_frames"))),
        temporal_iou_threshold=temporal_iou_threshold,
        class_whitelist=_comma_tokens(str(_parameter_value(node, "class_whitelist")).strip()),
        publish_detection_bbox=_bool_parameter(node, "publish_detection_bbox"),
        detection_bbox_topic=str(_parameter_value(node, "detection_bbox_topic")),
        detection_bbox_stamped_topic=str(_parameter_value(node, "detection_bbox_stamped_topic")),
        detection_bbox_frame_id=str(_parameter_value(node, "detection_bbox_frame_id")).strip(),
        publish_detection_mask=_bool_parameter(node, "publish_detection_mask"),
        detection_mask_topic=str(_parameter_value(node, "detection_mask_topic")),
        detection_mask_fill_value=int(
            np.clip(int(_parameter_value(node, "detection_mask_fill_value")), 1, 255)
        ),
        use_yolo_world=_bool_parameter(node, "use_yolo_world"),
        open_vocab_classes=_comma_tokens(
            str(_parameter_value(node, "open_vocab_classes")).strip()
        ),
        show_masks=_bool_parameter(node, "show_masks"),
        mask_overlay_alpha=mask_overlay_alpha,
    )


def validate_yolo_visualizer_parameters(params: YoloVisualizerParameters) -> None:
    """Require the selected model and every enabled ROS output contract."""
    required = {
        "image_topic": params.image_topic,
        "model_path": params.model_path,
    }
    if params.publish_detection_bbox:
        required.update(
            {
                "detection_bbox_topic": params.detection_bbox_topic,
                "detection_bbox_stamped_topic": params.detection_bbox_stamped_topic,
            }
        )
    if params.publish_detection_mask:
        required["detection_mask_topic"] = params.detection_mask_topic
    require_non_empty(required, context="YOLO perception")
    if not Path(params.model_path).is_file():
        raise FileNotFoundError(f"YOLO model_path does not exist: {params.model_path}")
    if params.mask_model_path and not Path(params.mask_model_path).is_file():
        raise FileNotFoundError(
            f"YOLO mask_model_path does not exist: {params.mask_model_path}"
        )
    if params.detection_bbox_frame_id:
        require_frame_id(
            params.detection_bbox_frame_id,
            "detection_bbox_frame_id",
            context="YOLO perception",
        )
