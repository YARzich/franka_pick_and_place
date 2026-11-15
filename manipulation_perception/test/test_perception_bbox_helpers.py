"""
Test bounding-box conversion, scaling, ROI masking, and stamped matching.

Responsibility: Preserve half-open geometry and require exact frame and timestamp correlation.
"""

from pathlib import Path
import sys

import numpy as np
from std_msgs.msg import Header
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from manipulation_perception.common.bbox import (  # noqa: E402,I100,I101,I201
    apply_bbox_roi,
    bbox_from_detection2d_array,
    bbox_from_mask,
    bbox_payload_from_detection,
    clip_half_open_bbox,
    DetectionBBox,
    mask_from_best_detection,
    scale_and_clip_bbox,
)
from manipulation_perception.common import bbox_geometry, bbox_mask, bbox_wire  # noqa: E402,I100,I101,I201
from manipulation_perception.common.detection_candidate import (  # noqa: E402,I100,I101,I201
    DetectionCandidate,
)
from manipulation_perception.common.stereo_detection_roi import DetectionRoiFilter  # noqa: E402,I100,I101,I201


def test_bbox_facade_preserves_the_established_public_surface():
    from manipulation_perception.common import bbox

    assert bbox.__all__ == (
        "DetectionBBox",
        "apply_bbox_roi",
        "bbox_from_detection2d_array",
        "bbox_from_mask",
        "bbox_payload_from_detection",
        "clip_half_open_bbox",
        "legacy_inclusive_bbox_from_detection",
        "mask_from_best_detection",
        "scale_and_clip_bbox",
    )
    assert bbox.DetectionBBox is bbox_geometry.DetectionBBox
    assert bbox.apply_bbox_roi is bbox_geometry.apply_bbox_roi
    assert bbox.bbox_from_detection2d_array is bbox_wire.bbox_from_detection2d_array
    assert bbox.bbox_payload_from_detection is bbox_wire.bbox_payload_from_detection
    assert bbox.bbox_from_mask is bbox_mask.bbox_from_mask
    assert bbox.mask_from_best_detection is bbox_mask.mask_from_best_detection


def _header(sec: int = 10, nanosec: int = 0, frame_id: str = 'camera') -> Header:
    header = Header()
    header.stamp.sec = sec
    header.stamp.nanosec = nanosec
    header.frame_id = frame_id
    return header


def _detection_bbox_array_message(detections, frame_shape, header):
    """Build the stable Detection2DArray contract without a detector backend."""
    del frame_shape
    message = Detection2DArray()
    message.header = header
    if not detections:
        return message

    best = max(detections, key=lambda detection: detection.confidence)
    x1, y1, x2, y2 = best.xyxy
    detection = Detection2D()
    detection.header = header
    detection.bbox.center.position.x = 0.5 * float(x1 + x2)
    detection.bbox.center.position.y = 0.5 * float(y1 + y2)
    detection.bbox.size_x = float(x2 - x1)
    detection.bbox.size_y = float(y2 - y1)
    result = ObjectHypothesisWithPose()
    result.hypothesis.class_id = str(best.class_id)
    result.hypothesis.score = best.confidence
    detection.results.append(result)
    message.detections.append(detection)
    return message


def test_bbox_from_detection2d_array_matches_stamped_yolo_output():
    header = _header()
    msg = _detection_bbox_array_message(
        [DetectionCandidate((2, 3, 7, 8), 0.9, 3)],
        frame_shape=(10, 20, 3),
        header=header,
    )

    bbox = bbox_from_detection2d_array(msg)

    assert bbox is not None
    assert bbox.xyxy == (2, 3, 7, 8)
    assert bbox.source_size is None
    assert bbox.class_id == 3
    assert bbox.score == 0.9
    assert bbox_from_detection2d_array(_detection_bbox_array_message([], (10, 20, 3), header)) is None


def test_bbox_from_detection2d_array_rejects_nonfinite_geometry():
    message = _detection_bbox_array_message(
        [DetectionCandidate((2, 3, 7, 8), 0.9, 3)],
        frame_shape=(10, 20, 3),
        header=_header(),
    )
    message.detections[0].bbox.center.position.x = float('nan')

    assert bbox_from_detection2d_array(message) is None


def test_scale_and_clip_bbox_matches_stereo_roi_scaling():
    bbox = DetectionBBox((10, 20, 111, 221), source_size=(640, 480))

    assert scale_and_clip_bbox(bbox, frame_width=320, frame_height=240) == (5, 10, 56, 111)
    assert scale_and_clip_bbox((-10, -20, 400, 300), frame_width=320, frame_height=240) == (
        0,
        0,
        320,
        240,
    )
    assert scale_and_clip_bbox((10, 10, 10, 20), frame_width=320, frame_height=240) is None


def test_apply_bbox_roi_uses_half_open_bounds_and_masks_outside_with_nan():
    depth = np.arange(20, dtype=np.float32).reshape(4, 5)

    roi = apply_bbox_roi(depth, (1, 1, 4, 3))

    assert np.isnan(roi[0, 0])
    np.testing.assert_array_equal(roi[1:3, 1:4], depth[1:3, 1:4])
    assert np.isnan(roi[3, 4])


def test_detection_roi_filter_tracks_timeout_clipping_and_disabled_passthrough():
    depth = np.arange(12, dtype=np.float32).reshape(3, 4)

    disabled = DetectionRoiFilter(enabled=False, timeout_sec=0.5)
    disabled_result = disabled.apply(depth, now_sec=10.0)

    assert disabled_result.depth is depth
    assert disabled_result.bbox is None
    assert disabled_result.missing_required_bbox is False

    enabled = DetectionRoiFilter(enabled=True, timeout_sec=0.5)
    missing_result = enabled.apply(depth, now_sec=10.0)
    assert missing_result.missing_required_bbox is True
    assert np.isnan(missing_result.depth).all()

    header = _header(sec=10, frame_id='camera')
    enabled.update_detection_array(
        _detection_bbox_array_message(
            [DetectionCandidate((2, 2, 7, 5), 0.9, 3)],
            frame_shape=(6, 8, 3),
            header=header,
        ),
        received_sec=10.0,
    )
    active_result = enabled.apply(depth, now_sec=10.2, source_header=header)

    assert active_result.bbox == (2, 2, 4, 3)
    assert active_result.missing_required_bbox is False
    np.testing.assert_array_equal(active_result.depth[2:3, 2:4], depth[2:3, 2:4])
    assert np.isnan(active_result.depth[0, 0])

    expired_result = enabled.apply(depth, now_sec=10.6, source_header=header)
    assert expired_result.missing_required_bbox is True
    assert np.isnan(expired_result.depth).all()


def test_detection_roi_filter_requires_matching_stamped_bbox_for_depth_frame():
    depth = np.arange(200, dtype=np.float32).reshape(10, 20)
    header = _header(sec=20, nanosec=30, frame_id='camera')
    msg = _detection_bbox_array_message(
        [DetectionCandidate((2, 3, 7, 8), 0.9, 3)],
        frame_shape=(10, 20, 3),
        header=header,
    )
    roi_filter = DetectionRoiFilter(enabled=True, timeout_sec=0.5)

    roi_filter.update_detection_array(msg, received_sec=50.0)
    assert roi_filter.has_fresh_response_for_header(header, now_sec=50.2) is True
    assert roi_filter.has_fresh_bbox_for_header(header, now_sec=50.2) is True
    active_result = roi_filter.apply(depth, now_sec=50.2, source_header=header)

    assert active_result.bbox == (2, 3, 7, 8)
    assert active_result.missing_required_bbox is False
    np.testing.assert_array_equal(active_result.depth[3:8, 2:7], depth[3:8, 2:7])
    assert np.isnan(active_result.depth[0, 0])

    stamp_mismatch = roi_filter.apply(depth, now_sec=50.2, source_header=_header(sec=20, nanosec=31))
    assert stamp_mismatch.missing_required_bbox is True
    assert np.isnan(stamp_mismatch.depth).all()

    frame_mismatch = roi_filter.apply(depth, now_sec=50.2, source_header=_header(sec=20, nanosec=30, frame_id='other'))
    assert frame_mismatch.missing_required_bbox is True
    assert np.isnan(frame_mismatch.depth).all()

    expired_result = roi_filter.apply(depth, now_sec=50.6, source_header=header)
    assert expired_result.missing_required_bbox is True
    assert np.isnan(expired_result.depth).all()


def test_detection_roi_filter_uses_matching_header_not_latest_bbox():
    depth = np.arange(200, dtype=np.float32).reshape(10, 20)
    header_a = _header(sec=20, nanosec=30, frame_id='camera')
    header_b = _header(sec=21, nanosec=30, frame_id='camera')
    roi_filter = DetectionRoiFilter(enabled=True, timeout_sec=0.5)

    roi_filter.update_detection_array(
        _detection_bbox_array_message(
            [DetectionCandidate((2, 3, 7, 8), 0.9, 3)],
            frame_shape=(10, 20, 3),
            header=header_a,
        ),
        received_sec=50.0,
    )
    roi_filter.update_detection_array(
        _detection_bbox_array_message(
            [DetectionCandidate((10, 1, 15, 5), 0.9, 3)],
            frame_shape=(10, 20, 3),
            header=header_b,
        ),
        received_sec=50.1,
    )

    active_result = roi_filter.apply(depth, now_sec=50.2, source_header=header_a)

    assert active_result.bbox == (2, 3, 7, 8)
    assert active_result.missing_required_bbox is False
    assert np.isnan(active_result.depth[0, 0])
    np.testing.assert_array_equal(active_result.depth[3:8, 2:7], depth[3:8, 2:7])


def test_detection_roi_filter_rejects_zero_stamp_without_latest_bbox_fallback():
    depth = np.arange(200, dtype=np.float32).reshape(10, 20)
    valid_header = _header(sec=20, nanosec=30, frame_id='camera')
    zero_header = _header(sec=0, nanosec=0, frame_id='camera')
    roi_filter = DetectionRoiFilter(enabled=True, timeout_sec=0.5)

    roi_filter.update_detection_array(
        _detection_bbox_array_message(
            [DetectionCandidate((2, 3, 7, 8), 0.9, 3)],
            frame_shape=(10, 20, 3),
            header=valid_header,
        ),
        received_sec=50.0,
    )
    roi_filter.update_detection_array(
        _detection_bbox_array_message(
            [DetectionCandidate((10, 1, 15, 5), 0.9, 3)],
            frame_shape=(10, 20, 3),
            header=zero_header,
        ),
        received_sec=50.1,
    )

    zero_result = roi_filter.apply(depth, now_sec=50.2, source_header=zero_header)

    assert roi_filter.has_fresh_response_for_header(zero_header, now_sec=50.2) is False
    assert roi_filter.has_fresh_bbox_for_header(zero_header, now_sec=50.2) is False
    assert zero_result.missing_required_bbox is True
    assert np.isnan(zero_result.depth).all()

    valid_result = roi_filter.apply(depth, now_sec=50.2, source_header=valid_header)
    assert valid_result.bbox == (2, 3, 7, 8)
    assert valid_result.missing_required_bbox is False


def test_detection_roi_filter_treats_empty_stamped_detection_as_fresh_empty_response():
    depth = np.arange(12, dtype=np.float32).reshape(3, 4)
    header = _header(sec=20, nanosec=30, frame_id='camera')
    roi_filter = DetectionRoiFilter(enabled=True, timeout_sec=0.5)

    roi_filter.update_detection_array(_detection_bbox_array_message([], (3, 4, 3), header), received_sec=50.0)

    assert roi_filter.has_fresh_response_for_header(header, now_sec=50.2) is True
    assert roi_filter.has_fresh_bbox_for_header(header, now_sec=50.2) is False
    result = roi_filter.apply(depth, now_sec=50.2, source_header=header)
    assert result.missing_required_bbox is True
    assert np.isnan(result.depth).all()


def test_bbox_payload_from_detection_clips_and_encodes_score_x1000():
    payload = bbox_payload_from_detection(
        DetectionCandidate((-5.0, 10.2, 205.9, 90.1), 0.8764, 7),
        (100, 200, 3),
    )

    assert payload == [0, 10, 199, 90, 200, 100, 7, 876]


def test_bbox_from_mask_supports_inclusive_and_exclusive_end_semantics():
    mask = np.zeros((5, 6), dtype=np.uint8)
    mask[1:4, 2:5] = 255

    assert bbox_from_mask(mask, (5, 6), min_pixels=1) == (2, 1, 4, 3)
    assert bbox_from_mask(mask, (5, 6), min_pixels=1, end_exclusive=True) == (2, 1, 5, 4)
    assert bbox_from_mask(mask, (5, 6), min_pixels=16) is None
    assert bbox_from_mask(np.array([1, 1]), (5, 6), min_pixels=1) is None


def test_bbox_from_mask_preserves_exclusive_image_boundary():
    full_mask = np.ones((4, 5), dtype=np.uint8)

    assert bbox_from_mask(full_mask, (4, 5), min_pixels=1) == (0, 0, 4, 3)
    assert bbox_from_mask(full_mask, (4, 5), min_pixels=1, end_exclusive=True) == (0, 0, 5, 4)


def test_clip_half_open_bbox_returns_valid_numpy_slice_bounds():
    assert clip_half_open_bbox((1, 1, 2, 2), 4, 4) == (1, 1, 2, 2)
    assert clip_half_open_bbox((-0.2, 1.1, 4.0, 3.2), 4, 4) == (0, 1, 4, 4)
    assert clip_half_open_bbox((4, 1, 5, 2), 4, 4) is None


def test_mask_from_best_detection_uses_mask_or_bbox_fallback():
    detection_mask = np.array([[0.0, 1.0], [0.6, 0.0]], dtype=np.float32)
    mask = mask_from_best_detection(
        [DetectionCandidate((0, 0, 1, 1), 0.5, 1, detection_mask)],
        (2, 2, 3),
        fill_value=255,
    )
    np.testing.assert_array_equal(mask, np.array([[0, 255], [255, 0]], dtype=np.uint8))

    fallback = mask_from_best_detection(
        [DetectionCandidate((1, 1, 2, 2), 0.5, 1)],
        (4, 4, 3),
        fill_value=128,
    )
    assert fallback[1, 1] == 128
    assert int(fallback.sum()) == 128
