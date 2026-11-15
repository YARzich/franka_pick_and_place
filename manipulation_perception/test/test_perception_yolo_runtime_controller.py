"""
Test application of live YOLO controls to detector configuration.

Responsibility: Verify typed state transitions, validation, logging, and atomic updates.
"""

from pathlib import Path
import sys
from types import SimpleNamespace


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from manipulation_perception.yolo.yolo_runtime_controls import (  # noqa: E402,I100,I101,I201
    get_control,
    YoloRuntimeValues,
)
from manipulation_perception.yolo.yolo_runtime_controller import (  # noqa: E402,I100,I101,I201
    YoloDetectorSettings,
    YoloRuntimeController,
)


def _state(**overrides) -> YoloRuntimeValues:
    values = {
        'confidence': 0.35,
        'iou': 0.2,
        'imgsz': 640,
        'max_det': 30,
        'run_every_n_frames': 1,
        'agnostic_nms': False,
        'augment': False,
        'sahi_slice_width': 224,
        'sahi_slice_height': 256,
        'sahi_overlap_ratio': 0.2,
        'min_box_area_ratio': 0.0001,
        'max_box_area_ratio': 0.35,
        'temporal_confirm_frames': 1,
        'temporal_iou_threshold': 0.5,
        'use_mask_model': False,
        'show_masks': True,
        'mask_overlay_alpha': 0.45,
        'whitelist_enabled': True,
    }
    values.update(overrides)
    return YoloRuntimeValues(**values)


def _controller(
    *,
    initial_state: YoloRuntimeValues | None = None,
    mask_model_available: bool = True,
    class_ids_filter: list[int] | None = None,
) -> YoloRuntimeController:
    return YoloRuntimeController(
        initial_state=initial_state or _state(),
        detector_settings=YoloDetectorSettings(
            device='0',
            use_yolo_world=False,
            sahi_perform_standard_pred=True,
            sahi_auto_slice_resolution=False,
            sahi_postprocess_type='GREEDYNMM',
            sahi_postprocess_match_metric='IOS',
            sahi_postprocess_class_agnostic=False,
        ),
        available_class_ids_filter=class_ids_filter if class_ids_filter is not None else [1, 3],
        mask_model_available=mask_model_available,
        initial_sahi_postprocess_match_threshold=0.5,
    )


def test_detector_config_is_built_from_current_typed_state():
    class_ids = [1, 3]
    controller = _controller(class_ids_filter=class_ids)
    class_ids.append(9)

    config = controller.build_detector_config()

    assert config.confidence == 0.35
    assert config.device == '0'
    assert config.active_class_ids_filter == [1, 3]
    assert config.sahi_postprocess_match_threshold == 0.5
    assert config.track_max_age == 3


def test_apply_controls_reports_transitions_and_updates_detector_config():
    controller = _controller()
    controls = controller.create_controls()

    first_update = controller.apply_controls(controls)
    unchanged_update = controller.apply_controls(controls)

    assert first_update.controls_applied is True
    assert first_update.model_changed is False
    assert first_update.state_changed is False
    assert unchanged_update.state_changed is False
    assert controller.sahi_postprocess_match_threshold == 0.5

    get_control(controls, 'confidence')['value'] = 73
    get_control(controls, 'iou')['value'] = 61
    get_control(controls, 'model_variant')['value'] = 1
    get_control(controls, 'use_whitelist')['value'] = 0

    changed_update = controller.apply_controls(controls)
    config = controller.build_detector_config()

    assert changed_update.model_changed is True
    assert changed_update.state_changed is True
    assert changed_update.state.confidence == 0.73
    assert config.iou == 0.61
    assert config.use_mask_model is True
    assert config.active_class_ids_filter is None
    assert config.sahi_postprocess_match_threshold == 0.61


def test_invalid_control_snapshot_does_not_change_state():
    controller = _controller()
    controls = [
        control
        for control in controller.create_controls()
        if control['key'] != 'confidence'
    ]
    initial_state = controller.state

    update = controller.apply_controls(controls)

    assert update.controls_applied is False
    assert update.model_changed is False
    assert update.state_changed is False
    assert controller.state == initial_state


def test_unchanged_controls_preserve_exact_non_slider_values():
    controller = _controller(initial_state=_state(
        imgsz=64, sahi_slice_width=64, sahi_slice_height=64,
        temporal_confirm_frames=20, iou=0.015, confidence=0.357,
    ))
    original = controller.build_detector_config()
    controls = controller.create_controls()

    update = controller.apply_controls(controls)

    assert controller.build_detector_config() == original
    assert not update.state_changed


def test_editing_one_control_preserves_other_exact_parameters():
    controller = _controller(initial_state=_state(imgsz=64, iou=0.015, confidence=0.357))
    controls = controller.create_controls()
    get_control(controls, 'confidence')['value'] = 73

    controller.apply_controls(controls)
    config = controller.build_detector_config()

    assert config.confidence == 0.73
    assert config.imgsz == 64
    assert config.iou == 0.015
    assert config.sahi_postprocess_match_threshold == 0.5
    get_control(controls, 'iou')['value'] = 61
    controller.apply_controls(controls)
    assert controller.build_detector_config().iou == 0.61
    assert controller.sahi_postprocess_match_threshold == 0.61


def test_from_parameters_preserves_initial_detector_and_ui_values():
    parameters = SimpleNamespace(
        confidence=0.4,
        iou=0.25,
        imgsz=960,
        max_det=12,
        run_every_n_frames=3,
        agnostic_nms=True,
        augment=True,
        sahi_slice_width=320,
        sahi_slice_height=352,
        sahi_overlap_ratio=0.3,
        min_box_area_ratio=0.001,
        max_box_area_ratio=0.4,
        temporal_confirm_frames=2,
        temporal_iou_threshold=0.6,
        show_masks=False,
        mask_overlay_alpha=0.7,
        device='cpu',
        use_yolo_world=True,
        sahi_perform_standard_pred=False,
        sahi_auto_slice_resolution=True,
        sahi_postprocess_type='NMS',
        sahi_postprocess_match_metric='IOU',
        sahi_postprocess_class_agnostic=True,
        sahi_postprocess_match_threshold=0.45,
    )

    controller = YoloRuntimeController.from_parameters(
        parameters,
        available_class_ids_filter=None,
        mask_model_available=False,
    )
    config = controller.build_detector_config()

    assert controller.state.run_every_n_frames == 3
    assert controller.state.whitelist_enabled is False
    assert config.device == 'cpu'
    assert config.use_yolo_world is True
    assert config.sahi_postprocess_type == 'NMS'
    assert config.sahi_postprocess_match_threshold == 0.45
