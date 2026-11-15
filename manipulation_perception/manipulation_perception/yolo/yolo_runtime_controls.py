"""
Runtime control state for the YOLO visualizer.

Responsibility:
- Create the debug UI control descriptors used by yolo_visualizer_node.
- Convert control slider values into normalized runtime settings.
- Provide stable state tuples for change detection/log throttling.

Does not own:
- OpenCV window rendering or mouse callbacks.
- YOLO/SAHI model loading or inference.
- ROS parameters, publishers or subscriptions.
"""

from __future__ import annotations

from manipulation_perception.yolo.yolo_runtime_values import (
    Control,
    get_control,
    runtime_values_from_controls,
    YoloRuntimeValues,
)


def _bounded_int(value: float | int, min_value: int, max_value: int) -> int:
    return int(max(min_value, min(max_value, round(float(value)))))


def _control(
    key: str,
    label: str,
    min_value: int,
    max_value: int,
    value: int,
    suffix: str,
) -> Control:
    return {
        "key": key,
        "label": label,
        "min": min_value,
        "max": max_value,
        "value": int(value),
        "suffix": suffix,
    }


def create_runtime_controls(
    values: YoloRuntimeValues,
    mask_model_available: bool,
    has_class_filter: bool,
) -> list[Control]:
    """Build slider descriptors, omitting controls for unavailable capabilities."""
    controls = [
        _control(
            "model_variant",
            "Model variant",
            0,
            1,
            1 if values.use_mask_model else 0,
            "bbox/mask",
        ),
        _control(
            "confidence",
            "Confidence threshold",
            1,
            100,
            _bounded_int(values.confidence * 100.0, 1, 100),
            "%",
        ),
        _control(
            "iou",
            "SAHI match threshold",
            1,
            100,
            _bounded_int(values.iou * 100.0, 1, 100),
            "%",
        ),
        _control("imgsz", "Input size", 10, 60, _bounded_int(values.imgsz / 32.0, 10, 60), "x32"),
        _control("max_det", "Max detections", 1, 200, _bounded_int(values.max_det, 1, 200), ""),
        _control(
            "run_every_n_frames",
            "Run every N frames",
            1,
            30,
            _bounded_int(values.run_every_n_frames, 1, 30),
            "",
        ),
        _control("agnostic_nms", "Agnostic NMS", 0, 1, 1 if values.agnostic_nms else 0, "0/1"),
        _control("augment", "TTA augment", 0, 1, 1 if values.augment else 0, "0/1"),
        _control(
            "sahi_slice_width",
            "SAHI slice width",
            6,
            40,
            _bounded_int(values.sahi_slice_width / 32.0, 6, 40),
            "x32",
        ),
        _control(
            "sahi_slice_height",
            "SAHI slice height",
            6,
            40,
            _bounded_int(values.sahi_slice_height / 32.0, 6, 40),
            "x32",
        ),
        _control(
            "sahi_overlap_ratio",
            "SAHI overlap",
            0,
            80,
            _bounded_int(values.sahi_overlap_ratio * 100.0, 0, 80),
            "%",
        ),
        _control(
            "min_box_area_ratio",
            "Min box area",
            0,
            300,
            _bounded_int(values.min_box_area_ratio * 10000.0, 0, 300),
            "x1e-4",
        ),
        _control(
            "max_box_area_ratio",
            "Max box area",
            100,
            10000,
            _bounded_int(values.max_box_area_ratio * 10000.0, 100, 10000),
            "x1e-4",
        ),
        _control(
            "temporal_confirm_frames",
            "Temporal confirm frames",
            1,
            10,
            _bounded_int(values.temporal_confirm_frames, 1, 10),
            "",
        ),
        _control(
            "temporal_iou_threshold",
            "Temporal IoU",
            5,
            95,
            _bounded_int(values.temporal_iou_threshold * 100.0, 5, 95),
            "%",
        ),
        _control("show_masks", "Show masks", 0, 1, 1 if values.show_masks else 0, "0/1"),
        _control(
            "mask_overlay_alpha",
            "Mask alpha",
            5,
            95,
            _bounded_int(values.mask_overlay_alpha * 100.0, 5, 95),
            "%",
        ),
    ]

    if not mask_model_available:
        controls = [
            control
            for control in controls
            if control["key"] not in {"model_variant", "show_masks", "mask_overlay_alpha"}
        ]
    if has_class_filter:
        controls.append(
            _control(
                "use_whitelist",
                "Use class whitelist",
                0,
                1,
                1 if values.whitelist_enabled else 0,
                "0/1",
            )
        )
    return controls


def build_controls_layout(
    controls: list[Control],
    frame_width: int,
) -> tuple[list[Control], int]:
    """Compute deterministic panel hit regions for the current frame width."""
    row_h = 28
    pad_top = 12
    pad_bottom = 12
    label_x = 12
    label_width = 300
    track_x0 = label_x + label_width
    track_x1 = frame_width - 120
    if track_x1 <= track_x0 + 40:
        track_x1 = track_x0 + 40

    layout: list[Control] = []
    for idx, _control_item in enumerate(controls):
        y = pad_top + idx * row_h + 14
        layout.append(
            {
                "index": idx,
                "label_x": label_x,
                "label_y": y + 5,
                "track_x0": track_x0,
                "track_x1": track_x1,
                "track_y": y,
                "hit_y0": y - 10,
                "hit_y1": y + 10,
            }
        )
    panel_height = pad_top + len(controls) * row_h + pad_bottom
    return layout, panel_height


def set_control_value_from_x(
    controls: list[Control],
    layout: list[Control],
    control_index: int,
    x: int,
) -> None:
    """Map a mouse x-coordinate to one clamped discrete control value."""
    if control_index < 0 or control_index >= len(layout):
        return
    control = controls[control_index]
    row = layout[control_index]
    x0 = int(row["track_x0"])
    x1 = int(row["track_x1"])
    if x1 <= x0:
        return
    ratio = float(x - x0) / float(x1 - x0)
    ratio = max(0.0, min(1.0, ratio))
    min_value = int(control["min"])
    max_value = int(control["max"])
    raw_value = min_value + ratio * (max_value - min_value)
    if max_value - min_value <= 1:
        value = 1 if raw_value >= 0.5 else 0
    else:
        value = int(round(raw_value))
    control["value"] = max(min_value, min(max_value, value))


__all__ = [
    "Control",
    "YoloRuntimeValues",
    "build_controls_layout",
    "create_runtime_controls",
    "get_control",
    "runtime_values_from_controls",
    "set_control_value_from_x",
]
