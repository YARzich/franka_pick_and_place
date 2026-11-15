"""
Test conversion of pose detections into poses and visualization markers.

Responsibility: Select valid stamped headers and preserve marker identity and deletion behavior.
"""

from pathlib import Path
import sys

from geometry_msgs.msg import PoseStamped
import pytest
from std_msgs.msg import Header
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose
from visualization_msgs.msg import Marker


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from manipulation_perception.pose.pose_markers import (  # noqa: E402,I100,I101,I201
    delete_object_markers,
    object_geometry_markers,
)
from manipulation_perception.pose.pose_output_adapter import (  # noqa: E402,I100,I101,I201
    pose_with_source_stamp,
    select_pose_from_detection_array,
)


def _set_stamp(header: Header, sec: int = 1) -> None:
    header.stamp.sec = sec


def test_select_pose_prefers_result_pose_and_detection_header():
    msg = Detection3DArray()
    msg.header.frame_id = 'message_frame'
    _set_stamp(msg.header, sec=1)

    detection = Detection3D()
    detection.header.frame_id = 'detection_frame'
    _set_stamp(detection.header, sec=2)
    detection.bbox.center.position.x = 1.0
    result = ObjectHypothesisWithPose()
    result.pose.pose.position.x = 2.0
    detection.results.append(result)
    msg.detections.append(detection)

    selection = select_pose_from_detection_array(msg)

    assert selection is not None
    assert selection.detection is detection
    assert selection.pose_stamped.header.frame_id == 'detection_frame'
    assert selection.pose_stamped.header.stamp.sec == 2
    assert selection.pose_stamped.pose.position.x == 2.0


def test_select_pose_uses_bbox_pose_and_valid_array_header():
    msg = Detection3DArray()
    msg.header.frame_id = 'camera_frame'
    _set_stamp(msg.header, sec=3)
    detection = Detection3D()
    detection.bbox.center.position.y = 3.0
    msg.detections.append(detection)

    selection = select_pose_from_detection_array(msg)

    assert selection is not None
    assert selection.pose_stamped.header.frame_id == 'camera_frame'
    assert selection.pose_stamped.header.stamp.sec == 3
    assert selection.pose_stamped.pose.position.y == 3.0
    assert select_pose_from_detection_array(Detection3DArray()) is None


def test_select_pose_rejects_unstamped_or_frameless_source_headers():
    zero_stamp = Detection3DArray()
    zero_stamp.header.frame_id = 'camera_frame'
    zero_stamp.detections.append(Detection3D())

    empty_frame = Detection3DArray()
    _set_stamp(empty_frame.header)
    empty_frame.detections.append(Detection3D())

    assert select_pose_from_detection_array(zero_stamp) is None
    assert select_pose_from_detection_array(empty_frame) is None


def test_select_pose_falls_back_from_invalid_detection_header_to_array_header():
    msg = Detection3DArray()
    msg.header.frame_id = 'camera_frame'
    _set_stamp(msg.header, sec=4)
    detection = Detection3D()
    detection.header.frame_id = 'wrong_without_stamp'
    msg.detections.append(detection)

    selection = select_pose_from_detection_array(msg)

    assert selection is not None
    assert selection.pose_stamped.header.frame_id == 'camera_frame'
    assert selection.pose_stamped.header.stamp.sec == 4


@pytest.mark.parametrize('use_result', [False, True])
@pytest.mark.parametrize(
    ('part', 'component', 'value'),
    [
        ('position', 'x', float('nan')),
        ('position', 'y', float('inf')),
        ('position', 'z', -float('inf')),
        ('orientation', 'x', float('nan')),
        ('orientation', 'y', float('inf')),
        ('orientation', 'w', 0.0),
        ('orientation', 'w', 2.0),
    ],
)
def test_select_pose_rejects_invalid_geometry(use_result, part, component, value):
    msg = Detection3DArray()
    msg.header.frame_id = 'camera_frame'
    _set_stamp(msg.header)
    detection = Detection3D()
    if use_result:
        result = ObjectHypothesisWithPose()
        detection.results.append(result)
        pose = result.pose.pose
    else:
        pose = detection.bbox.center
    setattr(getattr(pose, part), component, value)
    msg.detections.append(detection)

    assert select_pose_from_detection_array(msg) is None


def test_select_pose_normalizes_small_quaternion_drift_without_mutating_input():
    msg = Detection3DArray()
    msg.header.frame_id = 'camera_frame'
    _set_stamp(msg.header)
    detection = Detection3D()
    detection.bbox.center.orientation.w = 1.0005
    msg.detections.append(detection)

    selection = select_pose_from_detection_array(msg)

    assert selection is not None
    assert selection.pose_stamped.pose.orientation.w == pytest.approx(1.0)
    assert detection.bbox.center.orientation.w == 1.0005


def test_transformed_pose_restores_source_stamp_and_uses_output_frame():
    transformed = PoseStamped()
    transformed.header.frame_id = 'tf_target'
    transformed.header.stamp.sec = 99
    transformed.pose.position.x = 1.25
    source_header = Header()
    source_header.frame_id = 'camera_frame'
    source_header.stamp.sec = 7
    source_header.stamp.nanosec = 123

    result = pose_with_source_stamp(transformed, source_header, 'world')

    assert result.header.frame_id == 'world'
    assert result.header.stamp.sec == 7
    assert result.header.stamp.nanosec == 123
    assert result.pose.position.x == 1.25
    assert transformed.header.stamp.sec == 99
    assert source_header.frame_id == 'camera_frame'


def test_pose_marker_builders_keep_namespace_ids_and_size_fallback():
    msg = Detection3DArray()
    msg.header.frame_id = 'world'
    _set_stamp(msg.header)
    detection = Detection3D()
    detection.bbox.size.x = 0.0
    detection.bbox.size.y = 0.02
    detection.bbox.size.z = 0.0
    msg.detections.append(detection)
    selection = select_pose_from_detection_array(msg)

    assert selection is not None
    markers = object_geometry_markers(
        selection.pose_stamped,
        detection,
        fallback_object_size_m=0.05,
        marker_namespace="test_object",
    )

    assert len(markers.markers) == 2
    object_marker, pose_marker = markers.markers
    assert object_marker.ns == 'test_object'
    assert object_marker.id == 0
    assert object_marker.type == Marker.CUBE
    assert object_marker.scale.x == 0.05
    assert object_marker.scale.y == 0.02
    assert object_marker.scale.z == 0.05
    assert pose_marker.id == 1
    assert pose_marker.type == Marker.ARROW


def test_delete_marker_builder_deletes_object_and_pose_markers():
    header = Header()
    header.frame_id = 'world'
    _set_stamp(header)

    markers = delete_object_markers(header, "test_object")

    assert [marker.id for marker in markers.markers] == [0, 1]
    assert all(marker.ns == 'test_object' for marker in markers.markers)
    assert all(marker.action == Marker.DELETE for marker in markers.markers)
    assert all(marker.header.frame_id == 'world' for marker in markers.markers)
