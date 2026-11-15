"""
Generic marker builders for published 6D object poses.

Responsibility:
- Build bounding-box and pose-arrow markers from a selected pose.
- Build delete markers when no detection is available.
- Keep IDs, colors and injected size/namespace policy centralized.

Does not own:
- Detection selection.
- TF lookup or frame transformation.
- ROS publishers/subscribers.
"""

from __future__ import annotations

import math

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from vision_msgs.msg import Detection3D
from visualization_msgs.msg import Marker, MarkerArray


def delete_object_markers(header: Header, marker_namespace: str) -> MarkerArray:
    """Delete both persistent object markers using the supplied stamped frame."""
    delete_array = MarkerArray()
    for marker_id in (0, 1):
        marker = Marker()
        marker.header = header
        marker.ns = marker_namespace
        marker.id = marker_id
        marker.action = Marker.DELETE
        delete_array.markers.append(marker)
    return delete_array


def _positive_size_or_default(value: float, default_value: float) -> float:
    if value > 1.0e-6 and not math.isclose(value, 0.0):
        return value
    return default_value


def object_geometry_markers(
    pose_stamped: PoseStamped,
    detection: Detection3D,
    fallback_object_size_m: float,
    marker_namespace: str,
) -> MarkerArray:
    """
    Build persistent metric bounding-box and orientation-arrow markers.

    Marker pose, stamp and frame come from the already TF-transformed pose.
    Positive detected dimensions take precedence over the fixed-size fallback.
    """
    fixed_size = max(0.001, float(fallback_object_size_m))

    object_marker = Marker()
    object_marker.header = pose_stamped.header
    object_marker.ns = marker_namespace
    object_marker.id = 0
    object_marker.type = Marker.CUBE
    object_marker.action = Marker.ADD
    object_marker.pose = pose_stamped.pose
    object_marker.scale.x = _positive_size_or_default(detection.bbox.size.x, fixed_size)
    object_marker.scale.y = _positive_size_or_default(detection.bbox.size.y, fixed_size)
    object_marker.scale.z = _positive_size_or_default(detection.bbox.size.z, fixed_size)
    object_marker.color.r = 0.2
    object_marker.color.g = 0.9
    object_marker.color.b = 0.3
    object_marker.color.a = 0.45
    object_marker.lifetime.sec = 0
    object_marker.lifetime.nanosec = 0

    pose_marker = Marker()
    pose_marker.header = pose_stamped.header
    pose_marker.ns = marker_namespace
    pose_marker.id = 1
    pose_marker.type = Marker.ARROW
    pose_marker.action = Marker.ADD
    pose_marker.pose = pose_stamped.pose
    pose_marker.scale.x = max(fixed_size * 1.6, 0.06)
    pose_marker.scale.y = max(fixed_size * 0.14, 0.007)
    pose_marker.scale.z = max(fixed_size * 0.14, 0.007)
    pose_marker.color.r = 0.95
    pose_marker.color.g = 0.25
    pose_marker.color.b = 0.1
    pose_marker.color.a = 0.95
    pose_marker.lifetime.sec = 0
    pose_marker.lifetime.nanosec = 0

    marker_array = MarkerArray()
    marker_array.markers.extend([object_marker, pose_marker])
    return marker_array
