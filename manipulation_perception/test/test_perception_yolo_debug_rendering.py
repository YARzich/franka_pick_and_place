"""
Test YOLO debug overlays and interactive control-panel geometry.

Responsibility: Verify drawing, half-open boxes, control mapping, and input-frame immutability.
"""

from pathlib import Path
import sys

import cv2
import numpy as np


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from manipulation_perception.common.detection_candidate import (  # noqa: E402,I100,I101,I201
    DetectionCandidate,
)
from manipulation_perception.yolo.yolo_debug_rendering import (  # noqa: E402
    draw_detections,
    draw_mask_overlay,
    render_controls_panel,
)
from manipulation_perception.yolo.yolo_debug_view import (  # noqa: E402,I100,I101,I201
    YoloDebugControlPanel,
)
from manipulation_perception.yolo.yolo_runtime_controls import (  # noqa: E402,I100,I101,I201
    YoloRuntimeValues,
    build_controls_layout,
    create_runtime_controls,
    set_control_value_from_x,
)


def _runtime_values() -> YoloRuntimeValues:
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
        whitelist_enabled=False,
    )


def test_control_layout_and_value_from_x_clamp_and_boolean_threshold():
    controls = create_runtime_controls(
        _runtime_values(),
        mask_model_available=True,
        has_class_filter=False,
    )
    layout, panel_height = build_controls_layout(controls, frame_width=500)
    confidence = next(
        idx for idx, control in enumerate(controls)
        if control['key'] == 'confidence'
    )
    model_variant = next(
        idx for idx, control in enumerate(controls)
        if control['key'] == 'model_variant'
    )

    assert panel_height == 12 + len(controls) * 28 + 12
    assert layout[0]['track_x1'] > layout[0]['track_x0']

    set_control_value_from_x(controls, layout, confidence, x=-999)
    assert controls[confidence]['value'] == controls[confidence]['min']

    set_control_value_from_x(controls, layout, confidence, x=9999)
    assert controls[confidence]['value'] == controls[confidence]['max']

    set_control_value_from_x(
        controls,
        layout,
        model_variant,
        x=int(layout[model_variant]['track_x0']),
    )
    assert controls[model_variant]['value'] == 0
    set_control_value_from_x(
        controls,
        layout,
        model_variant,
        x=int(layout[model_variant]['track_x1']),
    )
    assert controls[model_variant]['value'] == 1


def test_draw_mask_overlay_changes_only_masked_pixels():
    frame = np.zeros((3, 3, 3), dtype=np.uint8)
    mask = np.zeros((3, 3), dtype=np.float32)
    mask[1, 1] = 1.0

    draw_mask_overlay(frame, mask, alpha=0.5)

    assert frame[1, 1].sum() > 0
    assert int(frame[0, 0].sum()) == 0


def test_draw_detections_converts_half_open_bbox_for_opencv_rectangle(monkeypatch):
    frame = np.zeros((20, 20, 3), dtype=np.uint8)
    rectangle_calls = []
    monkeypatch.setattr(
        cv2,
        'rectangle',
        lambda _frame, start, end, _color, _thickness: rectangle_calls.append(
            (start, end)
        ),
    )
    detections = [
        DetectionCandidate(
            (2, 2, 10, 10),
            0.8,
            1,
            np.ones((20, 20)),
            class_name='cube',
        )
    ]

    draw_detections(frame, detections, show_masks=False, mask_overlay_alpha=0.5)

    assert rectangle_calls == [((2, 2), (9, 9))]


def test_render_controls_panel_appends_panel_without_modifying_input_frame():
    controls = create_runtime_controls(
        _runtime_values(),
        mask_model_available=False,
        has_class_filter=False,
    )
    layout, panel_height = build_controls_layout(controls, frame_width=320)
    frame = np.zeros((40, 320, 3), dtype=np.uint8)
    before = frame.copy()

    rendered = render_controls_panel(frame, controls, layout, panel_height)

    assert rendered.shape == (40 + panel_height, 320, 3)
    np.testing.assert_array_equal(frame, before)
    assert rendered[40:].sum() > 0


def test_debug_control_panel_updates_control_from_mouse_without_window():
    controls = create_runtime_controls(
        _runtime_values(),
        mask_model_available=True,
        has_class_filter=False,
    )
    panel = YoloDebugControlPanel(controls)
    frame = np.zeros((40, 320, 3), dtype=np.uint8)

    panel.render(frame)
    confidence_index = next(
        idx for idx, control in enumerate(controls)
        if control['key'] == 'confidence'
    )
    confidence_row = panel.layout[confidence_index]
    panel_y = int(confidence_row['track_y'])

    panel.on_mouse(
        cv2.EVENT_LBUTTONDOWN,
        int(confidence_row['track_x1']),
        40 + panel_y,
    )
    assert controls[confidence_index]['value'] == controls[confidence_index]['max']
    assert panel.drag_control_index == confidence_index

    panel.on_mouse(
        cv2.EVENT_LBUTTONUP,
        int(confidence_row['track_x1']),
        40 + panel_y,
    )
    assert panel.drag_control_index is None
