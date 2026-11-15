"""
Test typed YOLO-node parameter loading and normalization.

Responsibility: Verify injected graph/model values, list parsing, validation, and limits.
"""

from pathlib import Path
import sys
from types import SimpleNamespace

import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from manipulation_perception.yolo.yolo_node_parameters import (  # noqa: E402,I100,I101,I201
    declare_yolo_visualizer_parameters,
    read_yolo_visualizer_parameters,
    validate_yolo_visualizer_parameters,
)


class _FakeParameterNode:
    def __init__(self, overrides: dict | None = None) -> None:
        self.values = dict(overrides or {})
        self.declared: list[tuple[str, object]] = []

    def declare_parameter(self, name: str, default_value: object) -> None:
        self.declared.append((name, default_value))
        self.values.setdefault(name, default_value)

    def get_parameter(self, name: str) -> SimpleNamespace:
        return SimpleNamespace(value=self.values[name])


def test_yolo_parameters_keep_public_defaults_and_parse_lists(tmp_path):
    model_path = tmp_path / 'detector.pt'
    model_path.write_text('test model')
    node = _FakeParameterNode(
        {
            'image_topic': '/camera/image',
            'model_path': str(model_path),
            'class_whitelist': 'cube, 1',
            'open_vocab_classes': 'cube, mug',
            'detection_bbox_topic': '/detections/bbox',
            'detection_bbox_stamped_topic': '/detections/stamped',
            'detection_bbox_frame_id': 'camera_optical_frame',
        }
    )

    declare_yolo_visualizer_parameters(node)
    params = read_yolo_visualizer_parameters(node)
    validate_yolo_visualizer_parameters(params)

    assert ('image_topic', '') in node.declared
    assert params.image_topic == '/camera/image'
    assert params.model_path == str(model_path)
    assert params.mask_model_path == ''
    assert params.detection_bbox_topic == '/detections/bbox'
    assert params.detection_bbox_stamped_topic == '/detections/stamped'
    assert params.detection_bbox_frame_id == 'camera_optical_frame'
    assert params.detection_mask_topic == ''
    assert params.class_whitelist == ['cube', '1']
    assert params.open_vocab_classes == ['cube', 'mug']


def test_yolo_parameters_normalize_current_runtime_limits():
    node = _FakeParameterNode(
        {
            'sahi_slice_width': 10,
            'sahi_slice_height': 20,
            'confidence_threshold': 2.0,
            'iou_threshold': 0.0,
            'imgsz': 0,
            'max_det': 0,
            'sahi_overlap_ratio': 2.0,
            'sahi_postprocess_type': 'bad',
            'sahi_postprocess_match_metric': 'bad',
            'sahi_postprocess_match_threshold': 2.0,
            'min_box_area_ratio': -1.0,
            'max_box_area_ratio': -0.5,
            'temporal_confirm_frames': 0,
            'temporal_iou_threshold': 2.0,
            'detection_mask_fill_value': 999,
            'mask_overlay_alpha': 0.0,
            'run_every_n_frames': 0,
        }
    )

    declare_yolo_visualizer_parameters(node)
    params = read_yolo_visualizer_parameters(node)

    assert params.confidence == 1.0
    assert params.iou == 0.01
    assert params.imgsz == 32
    assert params.max_det == 1
    assert params.sahi_slice_width == 64
    assert params.sahi_slice_height == 64
    assert params.sahi_overlap_ratio == 0.8
    assert params.sahi_postprocess_type == 'GREEDYNMM'
    assert params.sahi_postprocess_match_metric == 'IOS'
    assert params.sahi_postprocess_match_threshold == 0.99
    assert params.min_box_area_ratio == 0.0
    assert params.max_box_area_ratio == 0.0
    assert params.temporal_confirm_frames == 1
    assert params.temporal_iou_threshold == 0.95
    assert params.detection_mask_fill_value == 255
    assert params.mask_overlay_alpha == 0.05
    assert params.run_every_n_frames == 1


def test_yolo_parameters_fail_fast_without_scenario_contract():
    node = _FakeParameterNode()
    declare_yolo_visualizer_parameters(node)

    with pytest.raises(ValueError, match='image_topic.*model_path'):
        validate_yolo_visualizer_parameters(read_yolo_visualizer_parameters(node))
