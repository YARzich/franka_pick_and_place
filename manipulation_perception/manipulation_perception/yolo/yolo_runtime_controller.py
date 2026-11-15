"""
Typed mutable runtime state for the YOLO visualizer.

Responsibility:
- Own the current values controlled by the OpenCV debug panel.
- Apply normalized control values and report meaningful state transitions.
- Build the detector runtime config consumed by the headless YOLO backend.

Does not own:
- ROS parameters, subscriptions, publishers or logging.
- OpenCV rendering, mouse handling or model lifecycle side effects.
"""

from __future__ import annotations

from dataclasses import dataclass, fields, replace

from manipulation_perception.yolo.yolo_detector import YoloDetectorRuntimeConfig
from manipulation_perception.yolo.yolo_node_parameters import YoloVisualizerParameters
from manipulation_perception.yolo.yolo_runtime_controls import (
    Control,
    create_runtime_controls,
    runtime_values_from_controls,
    YoloRuntimeValues,
)


@dataclass(frozen=True)
class YoloDetectorSettings:
    """Detector options that are fixed while the node is running."""

    device: str
    use_yolo_world: bool
    sahi_perform_standard_pred: bool
    sahi_auto_slice_resolution: bool
    sahi_postprocess_type: str
    sahi_postprocess_match_metric: str
    sahi_postprocess_class_agnostic: bool
    track_max_age: int = 3


@dataclass(frozen=True)
class YoloRuntimeUpdate:
    """Result of applying one OpenCV control-panel snapshot."""

    controls_applied: bool
    model_changed: bool
    state_changed: bool
    state: YoloRuntimeValues


class YoloRuntimeController:
    """Own and update the mutable detector/UI state without ROS dependencies."""

    def __init__(
        self,
        *,
        initial_state: YoloRuntimeValues,
        detector_settings: YoloDetectorSettings,
        available_class_ids_filter: list[int] | None,
        mask_model_available: bool,
        initial_sahi_postprocess_match_threshold: float,
    ) -> None:
        self._state = initial_state
        self._detector_settings = detector_settings
        self._available_class_ids_filter = (
            tuple(available_class_ids_filter) if available_class_ids_filter is not None else None
        )
        self._mask_model_available = mask_model_available
        self._sahi_postprocess_match_threshold = initial_sahi_postprocess_match_threshold
        self.create_controls()

    @classmethod
    def from_parameters(
        cls,
        parameters: YoloVisualizerParameters,
        *,
        available_class_ids_filter: list[int] | None,
        mask_model_available: bool,
    ) -> "YoloRuntimeController":
        """Create mutable runtime state from one validated ROS parameter snapshot."""
        initial_state = YoloRuntimeValues(
            confidence=parameters.confidence,
            iou=parameters.iou,
            imgsz=parameters.imgsz,
            max_det=parameters.max_det,
            run_every_n_frames=parameters.run_every_n_frames,
            agnostic_nms=parameters.agnostic_nms,
            augment=parameters.augment,
            sahi_slice_width=parameters.sahi_slice_width,
            sahi_slice_height=parameters.sahi_slice_height,
            sahi_overlap_ratio=parameters.sahi_overlap_ratio,
            min_box_area_ratio=parameters.min_box_area_ratio,
            max_box_area_ratio=parameters.max_box_area_ratio,
            temporal_confirm_frames=parameters.temporal_confirm_frames,
            temporal_iou_threshold=parameters.temporal_iou_threshold,
            use_mask_model=False,
            show_masks=parameters.show_masks,
            mask_overlay_alpha=parameters.mask_overlay_alpha,
            whitelist_enabled=available_class_ids_filter is not None,
        )
        detector_settings = YoloDetectorSettings(
            device=parameters.device,
            use_yolo_world=parameters.use_yolo_world,
            sahi_perform_standard_pred=parameters.sahi_perform_standard_pred,
            sahi_auto_slice_resolution=parameters.sahi_auto_slice_resolution,
            sahi_postprocess_type=parameters.sahi_postprocess_type,
            sahi_postprocess_match_metric=parameters.sahi_postprocess_match_metric,
            sahi_postprocess_class_agnostic=parameters.sahi_postprocess_class_agnostic,
        )
        return cls(
            initial_state=initial_state,
            detector_settings=detector_settings,
            available_class_ids_filter=available_class_ids_filter,
            mask_model_available=mask_model_available,
            initial_sahi_postprocess_match_threshold=parameters.sahi_postprocess_match_threshold,
        )

    @property
    def state(self) -> YoloRuntimeValues:
        return self._state

    @property
    def sahi_postprocess_match_threshold(self) -> float:
        return self._sahi_postprocess_match_threshold

    def create_controls(self) -> list[Control]:
        controls = create_runtime_controls(
            self._state,
            mask_model_available=self._mask_model_available,
            has_class_filter=self._available_class_ids_filter is not None,
        )
        self._last_control_values = {control["key"]: control["value"] for control in controls}
        return controls

    def apply_controls(self, controls: list[Control]) -> YoloRuntimeUpdate:
        """
        Apply a UI snapshot and report model/state transitions to the node.

        Invalid or incomplete control sets preserve the previous state. The SAHI
        matching threshold follows the shared IoU slider only after an explicit
        user change, preserving its independently configured startup value.
        """
        next_state = runtime_values_from_controls(
            controls,
            mask_model_available=self._mask_model_available,
            has_class_filter=self._available_class_ids_filter is not None,
            current_use_mask_model=self._state.use_mask_model,
            current_show_masks=self._state.show_masks,
            current_mask_overlay_alpha=self._state.mask_overlay_alpha,
        )
        if next_state is None:
            return YoloRuntimeUpdate(False, False, False, self._state)

        control_values = {control["key"]: control["value"] for control in controls}
        changed_keys = {
            key for key, value in control_values.items()
            if value != self._last_control_values.get(key)
        }
        control_names = {"use_mask_model": "model_variant", "whitelist_enabled": "use_whitelist"}
        # The panel quantizes/clamps its display. Only an edited control may
        # replace the exact configured value; unrelated edits preserve it too.
        next_state = replace(self._state, **{
            field.name: getattr(next_state, field.name)
            for field in fields(YoloRuntimeValues)
            if control_names.get(field.name, field.name) in changed_keys
        })
        if next_state.max_box_area_ratio < next_state.min_box_area_ratio:
            next_state = replace(next_state, max_box_area_ratio=next_state.min_box_area_ratio)
        model_changed = next_state.use_mask_model != self._state.use_mask_model
        iou_changed = "iou" in changed_keys
        state_changed = next_state != self._state

        self._state = next_state
        # The existing UI shares its IoU slider with SAHI only after the user changes it.
        if iou_changed:
            self._sahi_postprocess_match_threshold = next_state.iou
        self._last_control_values = control_values
        return YoloRuntimeUpdate(True, model_changed, state_changed, self._state)

    def build_detector_config(self) -> YoloDetectorRuntimeConfig:
        """Freeze current UI values and fixed settings for one inference call."""
        state = self._state
        settings = self._detector_settings
        active_class_ids_filter = None
        if state.whitelist_enabled and self._available_class_ids_filter is not None:
            active_class_ids_filter = list(self._available_class_ids_filter)

        return YoloDetectorRuntimeConfig(
            confidence=state.confidence,
            iou=state.iou,
            device=settings.device,
            imgsz=state.imgsz,
            max_det=state.max_det,
            active_class_ids_filter=active_class_ids_filter,
            agnostic_nms=state.agnostic_nms,
            augment=state.augment,
            use_yolo_world=settings.use_yolo_world,
            use_mask_model=state.use_mask_model,
            sahi_slice_width=state.sahi_slice_width,
            sahi_slice_height=state.sahi_slice_height,
            sahi_overlap_ratio=state.sahi_overlap_ratio,
            sahi_perform_standard_pred=settings.sahi_perform_standard_pred,
            sahi_auto_slice_resolution=settings.sahi_auto_slice_resolution,
            sahi_postprocess_type=settings.sahi_postprocess_type,
            sahi_postprocess_match_metric=settings.sahi_postprocess_match_metric,
            sahi_postprocess_match_threshold=self._sahi_postprocess_match_threshold,
            sahi_postprocess_class_agnostic=settings.sahi_postprocess_class_agnostic,
            min_box_area_ratio=state.min_box_area_ratio,
            max_box_area_ratio=state.max_box_area_ratio,
            temporal_confirm_frames=state.temporal_confirm_frames,
            temporal_iou_threshold=state.temporal_iou_threshold,
            track_max_age=settings.track_max_age,
        )
