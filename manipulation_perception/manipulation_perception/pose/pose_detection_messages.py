"""
ROS message builders for pose-estimation detections.

Responsibility:
- Convert backend pose payloads into `vision_msgs/Detection3DArray`.
- Prefer a validated backend score and otherwise use the configured fallback.
- Preserve the existing object id, class id, pose and bbox-size contract.

Does not own:
- ROS publishers or node callbacks.
- Pose estimation backend execution.
- TF transforms or marker publication.
"""

from __future__ import annotations

import copy
from typing import Sequence

from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

from manipulation_perception.pose.pose_geometry import finite_values, validated_pose_components


def build_empty_pose_detection_array(image_header, output_frame_id: str) -> Detection3DArray:
    """Build a no-detection response preserving the source ROS sample stamp."""
    detection_array = Detection3DArray()
    detection_array.header = copy.deepcopy(image_header)
    detection_array.header.frame_id = output_frame_id
    return detection_array


def _pose_parts(pose_data: dict) -> tuple[Sequence[float], Sequence[float]]:
    try:
        quat_list, trans_list = pose_data["TWO"]
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(
            "pose_data['TWO'] должен содержать [quat_xyzw, translation_xyz]"
        ) from exc
    return validated_pose_components(quat_list, trans_list)


def _detection_score(pose_data: dict, fallback_score: float) -> float:
    backend_score_present = "score" in pose_data
    raw_score = pose_data["score"] if backend_score_present else fallback_score
    score_name = "pose_data['score']" if backend_score_present else "score"
    score_value = finite_values(
        (raw_score,),
        expected_size=1,
        name=score_name,
    )[0]
    if not 0.0 <= score_value <= 1.0:
        raise ValueError(
            f"{score_name} должен быть в диапазоне [0, 1]"
        )
    return score_value


def build_pose_detection_array(
    pose_data: dict,
    image_header,
    output_frame_id: str,
    object_label: str,
    object_size_m: tuple[float, float, float],
    score: float,
) -> Detection3DArray:
    """
    Build one validated 6D object detection in ``output_frame_id``.

    Backend ``TWO`` translation and object dimensions are metres; quaternion is
    XYZW and must already be normalized within tolerance. The copied image stamp
    correlates this output with the exact inference input.
    """
    quat_list, trans_list = _pose_parts(pose_data)
    object_size = finite_values(object_size_m, expected_size=3, name="object_size_m")
    if any(size <= 0.0 for size in object_size):
        raise ValueError(
            "object_size_m должен содержать положительные размеры"
        )
    score_value = _detection_score(pose_data, score)

    detection_array = build_empty_pose_detection_array(image_header, output_frame_id)

    detection = Detection3D()
    detection.header = detection_array.header
    detection.id = str(object_label)

    hypothesis = ObjectHypothesisWithPose()
    hypothesis.hypothesis.class_id = str(object_label)
    hypothesis.hypothesis.score = score_value
    hypothesis.pose.pose.position.x = trans_list[0]
    hypothesis.pose.pose.position.y = trans_list[1]
    hypothesis.pose.pose.position.z = trans_list[2]
    hypothesis.pose.pose.orientation.x = quat_list[0]
    hypothesis.pose.pose.orientation.y = quat_list[1]
    hypothesis.pose.pose.orientation.z = quat_list[2]
    hypothesis.pose.pose.orientation.w = quat_list[3]
    detection.results.append(hypothesis)

    detection.bbox.center = hypothesis.pose.pose
    detection.bbox.size.x = object_size[0]
    detection.bbox.size.y = object_size[1]
    detection.bbox.size.z = object_size[2]
    detection_array.detections.append(detection)
    return detection_array
