"""
Test YOLO detection, bbox, and mask ROS message construction.

Responsibility: Preserve source headers, best-candidate selection, and half-open bbox semantics.
"""

from pathlib import Path
import sys

import numpy as np
from std_msgs.msg import Header


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from manipulation_perception.common.detection_candidate import (  # noqa: E402,I100,I101,I201
    DetectionCandidate,
)
from manipulation_perception.yolo.yolo_outputs import (  # noqa: E402,I100,I101,I201
    detection_bbox_array_message,
    detection_bbox_message,
    detection_mask_array,
)


def test_detection_bbox_message_uses_best_detection_and_empty_fallback():
    detections = [
        DetectionCandidate((1.0, 2.0, 5.0, 6.0), 0.2, 1),
        DetectionCandidate((2.0, 3.0, 7.0, 8.0), 0.9, 3),
    ]

    msg = detection_bbox_message(detections, frame_shape=(10, 20, 3))

    assert list(msg.data) == [2, 3, 7, 8, 20, 10, 3, 900]
    assert list(detection_bbox_message([], frame_shape=(10, 20, 3)).data) == []


def test_detection_bbox_array_message_uses_source_header_and_best_detection():
    header = Header()
    header.stamp.sec = 42
    header.stamp.nanosec = 7
    header.frame_id = 'table_stereo_left_optical_frame'
    detections = [
        DetectionCandidate((1.0, 2.0, 5.0, 6.0), 0.2, 1),
        DetectionCandidate((2.0, 3.0, 7.0, 8.0), 0.9, 3),
    ]

    msg = detection_bbox_array_message(detections, frame_shape=(10, 20, 3), header=header)

    assert msg.header == header
    assert len(msg.detections) == 1
    detection = msg.detections[0]
    assert detection.header == header
    assert detection.bbox.center.position.x == 4.5
    assert detection.bbox.center.position.y == 5.5
    assert detection.bbox.size_x == 5.0
    assert detection.bbox.size_y == 5.0
    assert detection.results[0].hypothesis.class_id == '3'
    assert detection.results[0].hypothesis.score == 0.9

    empty_msg = detection_bbox_array_message([], frame_shape=(10, 20, 3), header=header)
    assert empty_msg.header == header
    assert empty_msg.detections == []


def test_stamped_bbox_keeps_half_open_border_while_legacy_payload_stays_unchanged():
    header = Header()
    candidate = DetectionCandidate((0, 0, 20, 10), 0.9, 3)

    legacy = detection_bbox_message([candidate], frame_shape=(10, 20, 3))
    stamped = detection_bbox_array_message(
        [candidate],
        frame_shape=(10, 20, 3),
        header=header,
    )

    assert list(legacy.data[:4]) == [0, 0, 19, 9]
    assert stamped.detections[0].bbox.center.position.x == 10.0
    assert stamped.detections[0].bbox.center.position.y == 5.0
    assert stamped.detections[0].bbox.size_x == 20.0
    assert stamped.detections[0].bbox.size_y == 10.0


def test_detection_mask_array_uses_best_mask_or_bbox_fallback():
    mask = np.array([[0.0, 1.0], [1.0, 0.0]], dtype=np.float32)
    detections = [DetectionCandidate((0, 0, 1, 1), 0.9, 1, mask)]

    mask_array = detection_mask_array(detections, frame_shape=(2, 2, 3), fill_value=255)

    np.testing.assert_array_equal(mask_array, np.array([[0, 255], [255, 0]], dtype=np.uint8))

    fallback = detection_mask_array(
        [DetectionCandidate((1, 1, 2, 2), 0.9, 1)],
        frame_shape=(4, 4, 3),
        fill_value=999,
    )
    assert fallback[1, 1] == 255
    assert int(fallback.sum()) == 255
