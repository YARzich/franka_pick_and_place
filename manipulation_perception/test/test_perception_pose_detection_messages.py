"""
Test construction of stamped 3D pose-detection messages.

Responsibility: Preserve headers and invalidation semantics while rejecting non-finite poses.
"""

from pathlib import Path
import sys

import pytest
from std_msgs.msg import Header


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from manipulation_perception.pose.pose_detection_messages import (  # noqa: E402,I100,I101,I201
    build_empty_pose_detection_array,
    build_pose_detection_array,
)


def test_build_empty_pose_detection_array_preserves_stamp_and_invalidates_output():
    header = Header()
    header.frame_id = 'camera_input'
    header.stamp.sec = 8

    msg = build_empty_pose_detection_array(header, 'table_stereo_left_optical_frame')

    assert header.frame_id == 'camera_input'
    assert msg.header.frame_id == 'table_stereo_left_optical_frame'
    assert msg.header.stamp.sec == 8
    assert list(msg.detections) == []


def test_build_pose_detection_array_preserves_detection_contract_without_mutating_header():
    header = Header()
    header.frame_id = 'camera_input'
    header.stamp.sec = 3
    pose_data = {'TWO': [[0.0, 0.0, 0.0, 1.0], [0.1, 0.2, 0.3]]}

    msg = build_pose_detection_array(
        pose_data=pose_data,
        image_header=header,
        output_frame_id='table_stereo_left_optical_frame',
        object_label='cube_5cm',
        object_size_m=(0.05, 0.06, 0.07),
        score=0.8,
    )

    assert header.frame_id == 'camera_input'
    assert msg.header.frame_id == 'table_stereo_left_optical_frame'
    assert msg.header.stamp.sec == 3
    detection = msg.detections[0]
    assert detection.id == 'cube_5cm'
    assert detection.results[0].hypothesis.class_id == 'cube_5cm'
    assert detection.results[0].hypothesis.score == 0.8
    assert detection.results[0].pose.pose.position.x == 0.1
    assert detection.results[0].pose.pose.orientation.w == 1.0
    assert detection.bbox.center.position.z == 0.3
    assert detection.bbox.size.x == 0.05
    assert detection.bbox.size.y == 0.06
    assert detection.bbox.size.z == 0.07


def test_build_pose_detection_array_prefers_valid_backend_score():
    pose_data = {
        'TWO': [[0.0, 0.0, 0.0, 1.0], [0.1, 0.2, 0.3]],
        'score': 0.42,
    }

    msg = build_pose_detection_array(
        pose_data=pose_data,
        image_header=Header(),
        output_frame_id='world',
        object_label='cube',
        object_size_m=(0.05, 0.05, 0.05),
        score=0.8,
    )

    assert msg.detections[0].results[0].hypothesis.score == 0.42


@pytest.mark.parametrize(
    'backend_score',
    [None, 'bad', float('nan'), float('inf'), -0.01, 1.01],
)
def test_build_pose_detection_array_rejects_malformed_backend_score(
    backend_score,
):
    pose_data = {
        'TWO': [[0.0, 0.0, 0.0, 1.0], [0.1, 0.2, 0.3]],
        'score': backend_score,
    }

    with pytest.raises(ValueError, match=r"pose_data\['score'\]"):
        build_pose_detection_array(
            pose_data,
            Header(),
            'world',
            'cube',
            (0.05, 0.05, 0.05),
            0.8,
        )


def test_build_pose_detection_array_reports_invalid_pose_payloads():
    header = Header()

    with pytest.raises(ValueError, match='TWO'):
        build_pose_detection_array({}, header, 'world', 'cube', (0.05, 0.05, 0.05), 1.0)

    with pytest.raises(ValueError, match='quaternion'):
        build_pose_detection_array(
            {'TWO': [[0.0], [1.0]]},
            header,
            'world',
            'cube',
            (0.05, 0.05, 0.05),
            1.0,
        )


@pytest.mark.parametrize(
    'pose_data',
    [
        {'TWO': [[float('nan'), 0.0, 0.0, 1.0], [0.1, 0.2, 0.3]]},
        {'TWO': [[0.0, 0.0, 0.0, 1.0], [0.1, float('inf'), 0.3]]},
    ],
)
def test_build_pose_detection_array_rejects_non_finite_pose_components(pose_data):
    with pytest.raises(ValueError, match='конечных чисел'):
        build_pose_detection_array(
            pose_data,
            Header(),
            'world',
            'cube',
            (0.05, 0.05, 0.05),
            1.0,
        )


@pytest.mark.parametrize(
    'quaternion',
    [
        [0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 2.0],
        [0.0, 0.0, 0.0, 1.0, 0.0],
    ],
)
def test_build_pose_detection_array_rejects_invalid_quaternion(quaternion):
    with pytest.raises(ValueError, match='quaternion'):
        build_pose_detection_array(
            {'TWO': [quaternion, [0.1, 0.2, 0.3]]},
            Header(),
            'world',
            'cube',
            (0.05, 0.05, 0.05),
            1.0,
        )


@pytest.mark.parametrize(
    ('object_size_m', 'score', 'expected_error'),
    [
        ((0.05, float('inf'), 0.05), 1.0, 'object_size_m'),
        ((0.05, 0.05, 0.05), float('nan'), 'score'),
        ((0.05, 0.05, 0.05), -0.01, 'score'),
        ((0.05, 0.05, 0.05), 1.01, 'score'),
    ],
)
def test_build_pose_detection_array_rejects_non_finite_message_fields(
    object_size_m,
    score,
    expected_error,
):
    with pytest.raises(ValueError, match=expected_error):
        build_pose_detection_array(
            {'TWO': [[0.0, 0.0, 0.0, 1.0], [0.1, 0.2, 0.3]]},
            Header(),
            'world',
            'cube',
            object_size_m,
            score,
        )
