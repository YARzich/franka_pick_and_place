"""
Detection3DArray to PoseStamped selection helpers.

Responsibility:
- Select the current object pose from a Detection3DArray.
- Accept only a source header with a positive stamp and nonempty frame_id.
- Prefer the detection header, then the array header, without inventing a frame.
- Reject non-finite geometry and invalid quaternions before TF or publication.
- Restore the source sample stamp after tf2 transforms replace the output header.
- Return a small value object that the ROS node can transform and publish.

Does not own:
- TF lookup or frame transformation.
- Pose/marker publication.
- Backend-specific pose estimation or worker lifecycle.
"""

from __future__ import annotations

import copy
from dataclasses import dataclass

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from vision_msgs.msg import Detection3D, Detection3DArray

from manipulation_perception.common.header_sync import header_key
from manipulation_perception.pose.pose_geometry import validated_pose_components


@dataclass(frozen=True)
class PoseSelection:
    """Selected detection and source-stamped pose awaiting TF transformation."""

    detection: Detection3D
    pose_stamped: PoseStamped


def select_pose_from_detection_array(
    msg: Detection3DArray,
) -> PoseSelection | None:
    """
    Select a pose whose source header is safe for timestamped TF lookup.

    Changing frames is owned by the TF-aware ROS adapter; this selector never
    invents a source frame.
    """
    if not msg.detections:
        return None

    detection = msg.detections[0]
    source_header = next(
        (
            header
            for header in (detection.header, msg.header)
            if header_key(header) is not None
        ),
        None,
    )
    if source_header is None:
        return None

    pose_stamped = PoseStamped()
    pose_stamped.header = source_header

    if detection.results:
        pose = detection.results[0].pose.pose
    else:
        pose = detection.bbox.center
    position = pose.position
    orientation = pose.orientation
    try:
        quaternion, _ = validated_pose_components(
            (orientation.x, orientation.y, orientation.z, orientation.w),
            (position.x, position.y, position.z),
        )
    except ValueError:
        return None
    pose_stamped.pose = copy.deepcopy(pose)
    (
        pose_stamped.pose.orientation.x,
        pose_stamped.pose.orientation.y,
        pose_stamped.pose.orientation.z,
        pose_stamped.pose.orientation.w,
    ) = quaternion

    return PoseSelection(detection, pose_stamped)


def pose_with_source_stamp(
    transformed_pose: PoseStamped,
    source_header: Header,
    output_frame: str,
) -> PoseStamped:
    """
    Keep transformed coordinates while restoring source sample identity.

    Jazzy ``do_transform_pose_stamped`` assigns the TransformStamped header to
    its result. That transform stamp identifies TF data, not the sensor sample;
    downstream exact-time consumers require the original detection stamp.
    """
    result = copy.deepcopy(transformed_pose)
    result.header = copy.deepcopy(source_header)
    result.header.frame_id = output_frame
    return result


__all__ = ["PoseSelection", "pose_with_source_stamp", "select_pose_from_detection_array"]
