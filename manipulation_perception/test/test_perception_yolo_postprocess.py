"""
Test backend-neutral YOLO postprocessing and result adapters.

Responsibility: Verify IoU, tiling, NMS, limits, offsets, names, and optional masks.
"""

from pathlib import Path
import sys

import numpy as np


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from manipulation_perception.common.detection_candidate import (  # noqa: E402,I100,I101,I201
    DetectionCandidate,
)
from manipulation_perception.yolo.yolo_postprocess import (  # noqa: E402,I100,I101,I201
    apply_temporal_filter,
    detections_from_sahi_result,
    detections_from_yolo_result,
    filter_by_box_area,
    iou_xyxy,
    nms_detections,
    normalize_indices,
    TemporalDetectionTrack,
    tile_starts,
)


class _FakeBox:
    def __init__(self, xyxy, conf=None, cls=None):
        self.xyxy = np.asarray([xyxy], dtype=np.float32)
        self.conf = None if conf is None else [conf]
        self.cls = None if cls is None else [cls]


class _FakeMasks:
    def __init__(self, data):
        self.data = data


class _FakeMaskTensor:
    def __init__(self, data):
        self._data = np.asarray(data, dtype=np.float64)

    def detach(self):
        return self

    def cpu(self):
        return self

    def numpy(self):
        return self._data


class _FakeYoloResult:
    def __init__(self, boxes=None, masks=None, names=None):
        self.boxes = boxes
        self.masks = masks
        self.names = names if names is not None else {}


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


def _candidate(
    xyxy,
    confidence,
    class_id=1,
    class_name='cube',
    mask=None,
) -> DetectionCandidate:
    return DetectionCandidate(
        tuple(xyxy),
        confidence,
        class_id,
        mask,
        class_name=class_name,
    )


def _track(candidate, *, hits=1, age=0) -> TemporalDetectionTrack:
    return TemporalDetectionTrack(candidate=candidate, hits=hits, age=age)


def test_iou_xyxy_handles_overlap_and_degenerate_union():
    assert iou_xyxy([0.0, 0.0, 2.0, 2.0], [1.0, 1.0, 3.0, 3.0]) == 1.0 / 7.0
    assert iou_xyxy([0.0, 0.0, 0.0, 2.0], [1.0, 1.0, 3.0, 3.0]) == 0.0


def test_normalize_indices_accepts_opencv_shapes():
    assert normalize_indices(None) == []
    assert normalize_indices([]) == []
    assert normalize_indices(np.array([[2], [4]])) == [2, 4]
    assert normalize_indices([1, [3], np.array([5])]) == [1, 3, 5]


def test_tile_starts_cover_last_window_without_duplicates():
    assert tile_starts(100, 120, 30) == [0]
    assert tile_starts(100, 40, 30) == [0, 30, 60]
    assert tile_starts(100, 40, 1000) == [0, 60]


def test_filter_by_box_area_keeps_current_ratio_rules():
    detections = [
        _candidate((0.0, 0.0, 1.0, 1.0), 0.9),
        _candidate((0.0, 0.0, 10.0, 10.0), 0.8),
        _candidate((0.0, 0.0, 100.0, 100.0), 0.7),
    ]

    filtered = filter_by_box_area(
        detections,
        frame_shape=(100, 100, 3),
        min_box_area_ratio=0.005,
        max_box_area_ratio=0.5,
    )

    assert filtered == [detections[1]]


def test_nms_detections_respects_class_aware_mode():
    detections = [
        _candidate((0.0, 0.0, 10.0, 10.0), 0.9, class_id=1),
        _candidate((1.0, 1.0, 11.0, 11.0), 0.8, class_id=1),
        _candidate((1.0, 1.0, 11.0, 11.0), 0.7, class_id=2),
    ]

    class_aware = nms_detections(detections, iou_threshold=0.5, max_det=10, agnostic_nms=False)
    agnostic = nms_detections(detections, iou_threshold=0.5, max_det=10, agnostic_nms=True)

    assert class_aware == [detections[0], detections[2]]
    assert agnostic == [detections[0]]


def test_nms_and_sahi_adapters_respect_zero_and_single_max_det():
    single = [_candidate((0.0, 0.0, 10.0, 10.0), 0.9)]
    assert nms_detections(single, iou_threshold=0.5, max_det=0, agnostic_nms=False) == []

    detections = [
        _candidate((0.0, 0.0, 10.0, 10.0), 0.9, class_id=1),
        _candidate((20.0, 20.0, 30.0, 30.0), 0.8, class_id=2),
    ]
    assert nms_detections(
        detections,
        iou_threshold=0.5,
        max_det=1,
        agnostic_nms=False,
    ) == [detections[0]]

    result = _FakeSahiResult([_FakeSahiPrediction([0, 0, 1, 1], 0.9, 1, 'cube')])
    assert detections_from_sahi_result(result, active_class_ids_filter=None, max_det=0) == []


def test_yolo_result_adapter_handles_no_boxes():
    assert detections_from_yolo_result(_FakeYoloResult(boxes=None)) == []


def test_yolo_result_adapter_applies_offset_names_and_masks():
    result = _FakeYoloResult(
        boxes=[
            _FakeBox([1.0, 2.0, 3.0, 4.0], conf=0.75, cls=2),
            _FakeBox([5.0, 6.0, 7.0, 8.0], conf=0.50, cls=3),
        ],
        masks=_FakeMasks([_FakeMaskTensor([[0.0, 1.0], [1.0, 0.0]])]),
        names={2: 'cube', 3: 'table'},
    )

    detections = detections_from_yolo_result(result, offset_x=10, offset_y=20)

    assert detections[0].xyxy == (11.0, 22.0, 13.0, 24.0)
    assert detections[0].confidence == 0.75
    assert detections[0].class_id == 2
    assert detections[0].class_name == 'cube'
    assert detections[0].mask.dtype == np.float32
    np.testing.assert_allclose(detections[0].mask, [[0.0, 1.0], [1.0, 0.0]])
    assert detections[1].mask is None


def test_yolo_result_adapter_supports_names_list_and_missing_conf_cls():
    result = _FakeYoloResult(
        boxes=[_FakeBox([1.0, 2.0, 3.0, 4.0], conf=None, cls=None)],
        names=['cube'],
    )

    assert detections_from_yolo_result(result) == [
        _candidate((1.0, 2.0, 3.0, 4.0), 0.0, class_id=-1, class_name='-1')
    ]


def test_sahi_result_adapter_filters_sorts_and_limits():
    result = _FakeSahiResult(
        [
            _FakeSahiPrediction([0, 0, 1, 1], 0.2, 1, 'low'),
            _FakeSahiPrediction([1, 1, 2, 2], 0.9, 2, 'skip'),
            _FakeSahiPrediction([2, 2, 3, 3], 0.8, 1, 'high'),
            _FakeSahiPrediction([3, 3, 4, 4], 0.7, 1, 'mid'),
        ]
    )

    detections = detections_from_sahi_result(
        result,
        active_class_ids_filter={1},
        max_det=2,
    )

    assert detections == [
        _candidate((2.0, 2.0, 3.0, 3.0), 0.8, class_name='high'),
        _candidate((3.0, 3.0, 4.0, 4.0), 0.7, class_name='mid'),
    ]


def test_temporal_filter_bypasses_when_confirm_frames_is_one():
    detections = [_candidate((0, 0, 10, 10), 0.9)]
    tracks, confirmed = apply_temporal_filter(
        [_track(_candidate((0, 0, 10, 10), 0.5), hits=2)],
        detections,
        confirm_frames=1,
        iou_threshold=0.5,
        max_age=3,
        max_det=10,
    )

    assert tracks == []
    assert confirmed is detections


def test_temporal_filter_bypass_still_respects_max_det():
    detections = [
        _candidate((0, 0, 10, 10), 0.9, class_name='a'),
        _candidate((20, 20, 30, 30), 0.8, class_name='b'),
    ]

    tracks, confirmed = apply_temporal_filter([], detections, 1, 0.5, 3, 1)
    assert tracks == []
    assert confirmed == [detections[0]]

    tracks, confirmed = apply_temporal_filter([], detections, 1, 0.5, 3, 0)
    assert tracks == []
    assert confirmed == []


def test_temporal_filter_confirms_after_matching_frames_and_updates_payload():
    first = _candidate((0, 0, 10, 10), 0.5, class_name='old')
    second = _candidate(
        (1, 1, 11, 11),
        0.9,
        class_name='cube',
        mask=np.ones((2, 2), dtype=np.float32),
    )

    tracks, confirmed = apply_temporal_filter([], [first], 2, 0.5, 3, 10)
    assert confirmed == []
    tracks, confirmed = apply_temporal_filter(tracks, [second], 2, 0.5, 3, 10)

    assert len(tracks) == 1
    assert confirmed[0] is second
    assert tracks[0].candidate is second
    assert tracks[0].hits == 2
    assert tracks[0].age == 0


def test_temporal_filter_counts_overlapping_detections_once_per_frame():
    detections = nms_detections(
        [_candidate((0, 0, 10, 10), 0.9), _candidate((5, 0, 15, 10), 0.8)],
        iou_threshold=0.5,
        max_det=10,
        agnostic_nms=False,
    )
    assert len(detections) == 2

    tracks, confirmed = apply_temporal_filter([], detections, 2, 0.3, 3, 10)

    assert confirmed == []
    assert [track.hits for track in tracks] == [1, 1]

    tracks, confirmed = apply_temporal_filter(tracks, detections, 2, 0.3, 3, 10)

    assert confirmed == detections
    assert [track.hits for track in tracks] == [2, 2]


def test_temporal_filter_does_not_match_different_class_or_low_iou():
    base_track = _track(_candidate((0, 0, 10, 10), 0.8))
    different_class = _candidate(
        (0, 0, 10, 10),
        0.9,
        class_id=2,
        class_name='other',
    )
    far_bbox = _candidate((20, 20, 30, 30), 0.7)

    tracks, confirmed = apply_temporal_filter(
        [base_track],
        [different_class],
        2,
        0.5,
        3,
        10,
    )
    assert confirmed == []
    assert len(tracks) == 2

    tracks, confirmed = apply_temporal_filter(
        [base_track],
        [far_bbox],
        2,
        0.5,
        3,
        10,
    )
    assert confirmed == []
    assert len(tracks) == 2


def test_temporal_filter_ages_out_missing_tracks():
    track = _track(_candidate((0, 0, 10, 10), 0.8))

    tracks, confirmed = apply_temporal_filter([track], [], 2, 0.5, 1, 10)
    assert confirmed == []
    assert len(tracks) == 1
    tracks, confirmed = apply_temporal_filter(tracks, [], 2, 0.5, 1, 10)

    assert tracks == []
    assert confirmed == []


def test_temporal_filter_sorts_and_limits_confirmed_by_conf():
    tracks = [
        _track(_candidate((0, 0, 10, 10), 0.1, class_name='a')),
        _track(_candidate((20, 20, 30, 30), 0.1, class_name='b')),
    ]
    detections = [
        _candidate((20, 20, 30, 30), 0.7, class_name='b'),
        _candidate((0, 0, 10, 10), 0.9, class_name='a'),
    ]

    _, confirmed = apply_temporal_filter(tracks, detections, 2, 0.5, 3, 1)

    assert confirmed == [detections[1]]
