"""
Normalized YOLO runtime values and control-to-state conversion.

Responsibility:
- Hold the immutable detector/debug state applied to one image callback.
- Convert generic UI control dictionaries into validated runtime values.

Does not own control layout, OpenCV events, ROS parameters or model loading.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


Control = dict[str, Any]


@dataclass(frozen=True)
class YoloRuntimeValues:
    """Immutable detector and debug settings for an image callback."""

    confidence: float
    iou: float
    imgsz: int
    max_det: int
    run_every_n_frames: int
    agnostic_nms: bool
    augment: bool
    sahi_slice_width: int
    sahi_slice_height: int
    sahi_overlap_ratio: float
    min_box_area_ratio: float
    max_box_area_ratio: float
    temporal_confirm_frames: int
    temporal_iou_threshold: float
    use_mask_model: bool
    show_masks: bool
    mask_overlay_alpha: float
    whitelist_enabled: bool

    def state_tuple(self) -> tuple:
        """Return a stable, rounded representation for change detection."""
        return (
            round(self.confidence, 2),
            round(self.iou, 2),
            self.imgsz,
            self.max_det,
            self.run_every_n_frames,
            self.agnostic_nms,
            self.augment,
            self.whitelist_enabled,
            self.sahi_slice_width,
            self.sahi_slice_height,
            round(self.sahi_overlap_ratio, 2),
            round(self.min_box_area_ratio, 4),
            round(self.max_box_area_ratio, 4),
            self.temporal_confirm_frames,
            round(self.temporal_iou_threshold, 2),
            self.use_mask_model,
            self.show_masks,
            round(self.mask_overlay_alpha, 2),
        )


def get_control(controls: list[Control], key: str) -> Control | None:
    """Return the control descriptor matching `key`, if present."""
    for control in controls:
        if control["key"] == key:
            return control
    return None


def _required_control_values(
    controls: list[Control],
    keys: tuple[str, ...],
) -> dict[str, Control] | None:
    values: dict[str, Control] = {}
    for key in keys:
        control = get_control(controls, key)
        if control is None:
            return None
        values[key] = control
    return values


def runtime_values_from_controls(
    controls: list[Control],
    *,
    mask_model_available: bool,
    has_class_filter: bool,
    current_use_mask_model: bool,
    current_show_masks: bool,
    current_mask_overlay_alpha: float,
) -> YoloRuntimeValues | None:
    """Validate UI controls and return one normalized runtime snapshot."""
    required = _required_control_values(
        controls,
        (
            "confidence",
            "iou",
            "imgsz",
            "max_det",
            "run_every_n_frames",
            "agnostic_nms",
            "augment",
            "sahi_slice_width",
            "sahi_slice_height",
            "sahi_overlap_ratio",
            "min_box_area_ratio",
            "max_box_area_ratio",
            "temporal_confirm_frames",
            "temporal_iou_threshold",
        ),
    )
    if required is None:
        return None

    min_box_area_ratio = max(
        0.0,
        min(1.0, float(required["min_box_area_ratio"]["value"]) / 10000.0),
    )
    max_box_area_ratio = max(
        min_box_area_ratio,
        min(1.0, float(required["max_box_area_ratio"]["value"]) / 10000.0),
    )

    model_variant = get_control(controls, "model_variant")
    use_mask_model = current_use_mask_model
    if model_variant is not None:
        use_mask_model = mask_model_available and bool(int(model_variant["value"]))

    show_masks_control = get_control(controls, "show_masks")
    show_masks = current_show_masks
    if show_masks_control is not None:
        show_masks = bool(int(show_masks_control["value"]))

    mask_alpha_control = get_control(controls, "mask_overlay_alpha")
    mask_overlay_alpha = current_mask_overlay_alpha
    if mask_alpha_control is not None:
        mask_overlay_alpha = max(
            0.05,
            min(0.95, float(mask_alpha_control["value"]) / 100.0),
        )

    whitelist_control = get_control(controls, "use_whitelist")
    whitelist_enabled = bool(int(whitelist_control["value"])) if whitelist_control else False
    whitelist_enabled = has_class_filter and whitelist_enabled

    return YoloRuntimeValues(
        confidence=max(0.01, min(1.0, float(required["confidence"]["value"]) / 100.0)),
        iou=max(0.01, min(1.0, float(required["iou"]["value"]) / 100.0)),
        imgsz=int(max(10, int(required["imgsz"]["value"])) * 32),
        max_det=max(1, int(required["max_det"]["value"])),
        run_every_n_frames=max(1, int(required["run_every_n_frames"]["value"])),
        agnostic_nms=bool(int(required["agnostic_nms"]["value"])),
        augment=bool(int(required["augment"]["value"])),
        sahi_slice_width=int(max(6, int(required["sahi_slice_width"]["value"])) * 32),
        sahi_slice_height=int(max(6, int(required["sahi_slice_height"]["value"])) * 32),
        sahi_overlap_ratio=max(
            0.0,
            min(0.8, float(required["sahi_overlap_ratio"]["value"]) / 100.0),
        ),
        min_box_area_ratio=min_box_area_ratio,
        max_box_area_ratio=max_box_area_ratio,
        temporal_confirm_frames=max(1, int(required["temporal_confirm_frames"]["value"])),
        temporal_iou_threshold=max(
            0.05,
            min(0.95, float(required["temporal_iou_threshold"]["value"]) / 100.0),
        ),
        use_mask_model=use_mask_model,
        show_masks=show_masks,
        mask_overlay_alpha=mask_overlay_alpha,
        whitelist_enabled=whitelist_enabled,
    )


__all__ = [
    "Control",
    "YoloRuntimeValues",
    "get_control",
    "runtime_values_from_controls",
]
