"""
Test YOLO detector model selection, tiling, filtering, and temporal state.

Responsibility: Verify full-frame, mask, SAHI, and fallback paths produce normalized detections.
"""

from pathlib import Path
import sys

import numpy as np


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from manipulation_perception.common.detection_candidate import (  # noqa: E402,I100,I101,I201
    DetectionCandidate,
)
from manipulation_perception.yolo.yolo_detector import (  # noqa: E402,I100,I101,I201
    HeadlessYoloDetector,
    YoloDetectorRuntimeConfig,
)


class _FakeLogger:
    def __init__(self) -> None:
        self.info_messages: list[str] = []
        self.warn_messages: list[str] = []

    def info(self, message: str) -> None:
        self.info_messages.append(message)

    def warn(self, message: str) -> None:
        self.warn_messages.append(message)


class _FakeBox:
    def __init__(self, xyxy, conf=0.9, cls=1):
        self.xyxy = np.asarray([xyxy], dtype=np.float32)
        self.conf = [conf]
        self.cls = [cls]


class _FakeYoloResult:
    def __init__(self, boxes, names=None, masks=None):
        self.boxes = boxes
        self.masks = masks
        self.names = names if names is not None else {1: 'cube'}


class _FakeModel:
    names = {1: 'cube'}

    def __init__(self, *, task='detect', results=None):
        self.task = task
        self.results = list(results or [])
        self.calls: list[dict] = []

    def predict(self, **kwargs):
        self.calls.append(kwargs)
        if self.results:
            return [self.results.pop(0)]
        return []


class _FakeMasks:
    def __init__(self, data):
        self.data = data


class _FakeSahiCategory:
    def __init__(self, cls_id, name):
        self.id = cls_id
        self.name = name


class _FakeSahiScore:
    def __init__(self, value):
        self.value = value


class _FakeSahiBbox:
    def __init__(self, xyxy):
        self._xyxy = xyxy

    def to_xyxy(self):
        return self._xyxy


class _FakeSahiPrediction:
    def __init__(self, xyxy, conf, cls_id, cls_name):
        self.bbox = _FakeSahiBbox(xyxy)
        self.score = _FakeSahiScore(conf)
        self.category = _FakeSahiCategory(cls_id, cls_name)


class _FakeSahiResult:
    def __init__(self, predictions):
        self.object_prediction_list = predictions


class _FakeSahiModel:
    confidence_threshold = 0.0
    image_size = 0


def _config(**overrides) -> YoloDetectorRuntimeConfig:
    values = {
        'confidence': 0.35,
        'iou': 0.5,
        'device': '0',
        'imgsz': 640,
        'max_det': 30,
        'active_class_ids_filter': [1],
        'agnostic_nms': False,
        'augment': False,
        'use_yolo_world': False,
        'use_mask_model': False,
        'sahi_slice_width': 64,
        'sahi_slice_height': 64,
        'sahi_overlap_ratio': 0.0,
        'sahi_perform_standard_pred': True,
        'sahi_auto_slice_resolution': False,
        'sahi_postprocess_type': 'GREEDYNMM',
        'sahi_postprocess_match_metric': 'IOS',
        'sahi_postprocess_match_threshold': 0.5,
        'sahi_postprocess_class_agnostic': False,
        'min_box_area_ratio': 0.0,
        'max_box_area_ratio': 1.0,
        'temporal_confirm_frames': 1,
        'temporal_iou_threshold': 0.5,
        'track_max_age': 3,
    }
    values.update(overrides)
    return YoloDetectorRuntimeConfig(**values)


def test_detector_uses_active_segment_model_for_full_frame_prediction():
    result = _FakeYoloResult([_FakeBox([1, 2, 3, 4], conf=0.8, cls=1)])
    model = _FakeModel(task='segment', results=[result])
    detector = HeadlessYoloDetector(
        model=model,
        model_path='base.pt',
        mask_model=None,
        mask_model_path='mask.pt',
        sahi_model=None,
        get_sliced_prediction=None,
    )

    detections = detector.predict(np.zeros((10, 10, 3), dtype=np.uint8), _config())

    assert isinstance(detections[0], DetectionCandidate)
    assert detections[0].xyxy == (1.0, 2.0, 3.0, 4.0)
    assert detections[0].class_name == 'cube'
    assert model.calls[0]['source'].shape == (10, 10, 3)
    assert model.calls[0]['conf'] == 0.35
    assert model.calls[0]['classes'] == [1]
    assert model.calls[0]['verbose'] is False


def test_detector_uses_mask_model_when_runtime_selects_mask_variant():
    mask = np.array([[0.0, 1.0], [1.0, 0.0]], dtype=np.float32)
    result = _FakeYoloResult(
        [_FakeBox([1, 2, 3, 4], conf=0.8, cls=1)],
        masks=_FakeMasks([mask]),
    )
    model = _FakeModel(task='detect')
    mask_model = _FakeModel(task='segment', results=[result])
    detector = HeadlessYoloDetector(
        model=model,
        model_path='base.pt',
        mask_model=mask_model,
        mask_model_path='mask.pt',
        sahi_model=None,
        get_sliced_prediction=None,
    )

    detections = detector.predict(
        np.zeros((10, 10, 3), dtype=np.uint8),
        _config(use_mask_model=True),
    )

    assert model.calls == []
    assert mask_model.calls
    np.testing.assert_array_equal(detections[0].mask, mask)


def test_detector_uses_sahi_model_when_available_for_detect_model():
    model = _FakeModel(task='detect')
    sahi_model = _FakeSahiModel()
    logger = _FakeLogger()
    sliced_calls: list[dict] = []

    def _get_sliced_prediction(**kwargs):
        sliced_calls.append(kwargs)
        return _FakeSahiResult(
            [
                _FakeSahiPrediction([0, 0, 1, 1], 0.4, 2, 'skip'),
                _FakeSahiPrediction([1, 2, 3, 4], 0.9, 1, 'cube'),
            ]
        )

    detector = HeadlessYoloDetector(
        model=model,
        model_path='base.pt',
        mask_model=None,
        mask_model_path='mask.pt',
        sahi_model=sahi_model,
        get_sliced_prediction=_get_sliced_prediction,
        logger=logger,
    )

    detections = detector.predict(np.zeros((80, 90, 3), dtype=np.uint8), _config(max_det=5))

    assert detections == [
        DetectionCandidate((1.0, 2.0, 3.0, 4.0), 0.9, 1, class_name='cube')
    ]
    assert model.calls == []
    assert sahi_model.confidence_threshold == 0.35
    assert sahi_model.image_size == 640
    assert sliced_calls[0]['slice_height'] == 64
    assert sliced_calls[0]['slice_width'] == 64
    assert sliced_calls[0]['overlap_height_ratio'] == 0.0
    assert sliced_calls[0]['overlap_width_ratio'] == 0.0
    assert sliced_calls[0]['perform_standard_pred'] is True
    assert sliced_calls[0]['postprocess_type'] == 'GREEDYNMM'
    assert sliced_calls[0]['postprocess_match_metric'] == 'IOS'
    assert sliced_calls[0]['postprocess_match_threshold'] == 0.5
    assert sliced_calls[0]['postprocess_class_agnostic'] is False
    assert sliced_calls[0]['auto_slice_resolution'] is False
    assert sliced_calls[0]['verbose'] == 0
    assert len(logger.info_messages) == 1


def test_detector_falls_back_to_legacy_tiles_when_sahi_fails():
    tile_result = _FakeYoloResult([_FakeBox([0, 0, 10, 10], conf=0.9, cls=1)])
    model = _FakeModel(task='detect', results=[tile_result, tile_result, tile_result, tile_result])
    logger = _FakeLogger()

    def _failing_sliced_prediction(**_kwargs):
        raise RuntimeError('boom')

    detector = HeadlessYoloDetector(
        model=model,
        model_path='base.pt',
        mask_model=None,
        mask_model_path='mask.pt',
        sahi_model=_FakeSahiModel(),
        get_sliced_prediction=_failing_sliced_prediction,
        logger=logger,
    )

    detections = detector.predict(np.zeros((100, 100, 3), dtype=np.uint8), _config(max_det=10))

    assert len(model.calls) == 4
    assert [call['source'].shape[:2] for call in model.calls] == [(64, 64)] * 4
    assert sorted(det.xyxy[0] for det in detections) == [0.0, 0.0, 36.0, 36.0]
    assert logger.warn_messages


def test_detector_legacy_tiles_cover_edges_and_apply_offsets_before_nms():
    tile_results = [
        _FakeYoloResult([_FakeBox([1, 2, 11, 12], conf=0.9, cls=1)])
        for _ in range(9)
    ]
    model = _FakeModel(task='detect', results=tile_results)
    detector = HeadlessYoloDetector(
        model=model,
        model_path='base.pt',
        mask_model=None,
        mask_model_path='mask.pt',
        sahi_model=None,
        get_sliced_prediction=None,
    )

    detections = detector.predict(
        np.zeros((128, 128, 3), dtype=np.uint8),
        _config(sahi_overlap_ratio=0.5, max_det=20),
    )

    assert len(model.calls) == 9
    assert [call['source'].shape[:2] for call in model.calls] == [(64, 64)] * 9
    assert [det.xyxy[0] for det in detections] == [
        1.0, 33.0, 65.0, 1.0, 33.0, 65.0, 1.0, 33.0, 65.0
    ]
    assert [det.xyxy[1] for det in detections] == [
        2.0, 2.0, 2.0, 34.0, 34.0, 34.0, 66.0, 66.0, 66.0
    ]


def test_detector_legacy_tiles_uses_single_tile_for_small_frame():
    result = _FakeYoloResult([_FakeBox([1, 2, 11, 12], conf=0.9, cls=1)])
    model = _FakeModel(task='detect', results=[result])
    detector = HeadlessYoloDetector(
        model=model,
        model_path='base.pt',
        mask_model=None,
        mask_model_path='mask.pt',
        sahi_model=None,
        get_sliced_prediction=None,
    )

    detections = detector.predict(np.zeros((40, 50, 3), dtype=np.uint8), _config())

    assert len(model.calls) == 1
    assert model.calls[0]['source'].shape[:2] == (40, 50)
    assert detections[0].xyxy == (1.0, 2.0, 11.0, 12.0)


def test_detector_active_model_path_uses_mask_model_only_when_selected():
    model = _FakeModel()
    mask_model = _FakeModel(task='segment')
    detector = HeadlessYoloDetector(
        model=model,
        model_path='base.pt',
        mask_model=mask_model,
        mask_model_path='mask.pt',
        sahi_model=None,
        get_sliced_prediction=None,
    )

    assert detector.active_model_path(False) == 'base.pt'
    assert detector.active_model_path(True) == 'mask.pt'


def test_detector_detect_applies_area_filter_and_temporal_confirmation():
    first = _FakeYoloResult([_FakeBox([0, 0, 10, 10], conf=0.9, cls=1)])
    second = _FakeYoloResult([_FakeBox([1, 1, 11, 11], conf=0.8, cls=1)])
    model = _FakeModel(task='segment', results=[first, second])
    detector = HeadlessYoloDetector(
        model=model,
        model_path='base.pt',
        mask_model=None,
        mask_model_path='mask.pt',
        sahi_model=None,
        get_sliced_prediction=None,
    )
    frame = np.zeros((100, 100, 3), dtype=np.uint8)

    config = _config(
        temporal_confirm_frames=2,
        min_box_area_ratio=0.005,
    )
    assert detector.detect(frame, config) == []
    confirmed = detector.detect(frame, config)

    assert confirmed == [
        DetectionCandidate((1.0, 1.0, 11.0, 11.0), 0.8, 1, class_name='cube')
    ]


def test_detector_reset_temporal_tracks_clears_pending_confirmation():
    result = _FakeYoloResult([_FakeBox([0, 0, 10, 10], conf=0.9, cls=1)])
    model = _FakeModel(task='segment', results=[result, result])
    detector = HeadlessYoloDetector(
        model=model,
        model_path='base.pt',
        mask_model=None,
        mask_model_path='mask.pt',
        sahi_model=None,
        get_sliced_prediction=None,
    )
    frame = np.zeros((100, 100, 3), dtype=np.uint8)

    assert detector.detect(frame, _config(temporal_confirm_frames=2)) == []
    detector.reset_temporal_tracks()
    assert detector.detect(frame, _config(temporal_confirm_frames=2)) == []
