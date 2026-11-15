"""
Test creation and normalization of YOLO debug runtime controls.

Responsibility: Verify optional mask controls, slider units, required values, and mask state.
"""

from pathlib import Path
import sys


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from manipulation_perception.yolo.yolo_runtime_controls import (  # noqa: E402,I100,I101,I201
    YoloRuntimeValues,
    create_runtime_controls,
    get_control,
    runtime_values_from_controls,
)


def _values() -> YoloRuntimeValues:
    return YoloRuntimeValues(
        confidence=0.35,
        iou=0.01,
        imgsz=640,
        max_det=30,
        run_every_n_frames=1,
        agnostic_nms=False,
        augment=False,
        sahi_slice_width=224,
        sahi_slice_height=256,
        sahi_overlap_ratio=0.20,
        min_box_area_ratio=0.0001,
        max_box_area_ratio=0.35,
        temporal_confirm_frames=1,
        temporal_iou_threshold=0.5,
        use_mask_model=False,
        show_masks=True,
        mask_overlay_alpha=0.45,
        whitelist_enabled=True,
    )


def test_create_runtime_controls_hides_mask_controls_without_mask_model():
    controls = create_runtime_controls(
        _values(),
        mask_model_available=False,
        has_class_filter=True,
    )

    assert get_control(controls, 'model_variant') is None
    assert get_control(controls, 'show_masks') is None
    assert get_control(controls, 'mask_overlay_alpha') is None
    assert get_control(controls, 'use_whitelist') is not None
    assert get_control(controls, 'confidence')['value'] == 35
    assert get_control(controls, 'imgsz')['value'] == 20


def test_runtime_values_from_controls_normalizes_slider_units_and_mask_state():
    controls = create_runtime_controls(_values(), mask_model_available=True, has_class_filter=True)
    get_control(controls, 'confidence')['value'] = 120
    get_control(controls, 'iou')['value'] = 0
    get_control(controls, 'imgsz')['value'] = 10
    get_control(controls, 'sahi_overlap_ratio')['value'] = 95
    get_control(controls, 'min_box_area_ratio')['value'] = 500
    get_control(controls, 'max_box_area_ratio')['value'] = 100
    get_control(controls, 'model_variant')['value'] = 1
    get_control(controls, 'show_masks')['value'] = 0
    get_control(controls, 'mask_overlay_alpha')['value'] = 1
    get_control(controls, 'use_whitelist')['value'] = 0

    values = runtime_values_from_controls(
        controls,
        mask_model_available=True,
        has_class_filter=True,
        current_use_mask_model=False,
        current_show_masks=True,
        current_mask_overlay_alpha=0.45,
    )

    assert values.confidence == 1.0
    assert values.iou == 0.01
    assert values.imgsz == 320
    assert values.sahi_overlap_ratio == 0.8
    assert values.min_box_area_ratio == 0.05
    assert values.max_box_area_ratio == 0.05
    assert values.use_mask_model is True
    assert values.show_masks is False
    assert values.mask_overlay_alpha == 0.05
    assert values.whitelist_enabled is False


def test_runtime_values_returns_none_when_required_control_is_missing():
    controls = create_runtime_controls(
        _values(),
        mask_model_available=False,
        has_class_filter=False,
    )
    controls = [control for control in controls if control['key'] != 'confidence']

    assert runtime_values_from_controls(
        controls,
        mask_model_available=False,
        has_class_filter=False,
        current_use_mask_model=False,
        current_show_masks=True,
        current_mask_overlay_alpha=0.45,
    ) is None
