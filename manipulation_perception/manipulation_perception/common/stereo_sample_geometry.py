"""
Validate local stereo images and calibration as one exact sensor sample.

Responsibility:
- Enforce exact timestamps and configured frame identities for left/right images.
- Require CameraInfo to match the left image by exact ``stamp + frame_id``.
- Return one immutable calibration snapshot for disparity, depth, and cloud outputs.

Does not own ROS subscriptions, image decoding, or stereo matching.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from manipulation_perception.common.depth_geometry import camera_intrinsics_from_matrices
from manipulation_perception.common.header_sync import header_key


Intrinsics = tuple[float, float, float, float]


@dataclass(frozen=True)
class LocalStereoGeometry:
    """Calibration proven to belong to one exact local stereo image pair."""

    intrinsics: Intrinsics


def resolve_local_stereo_geometry(
    left_image: Any,
    right_image: Any,
    camera_info: Any,
    *,
    expected_left_frame: str,
    expected_right_frame: str,
) -> LocalStereoGeometry:
    """Validate sample identity, resolution, and finite pinhole calibration."""
    left_key = header_key(getattr(left_image, "header", None))
    right_key = header_key(getattr(right_image, "header", None))
    camera_key = header_key(getattr(camera_info, "header", None))
    if left_key is None or right_key is None or camera_key is None:
        raise ValueError("local stereo inputs require valid positive stamped headers")
    if left_key.frame_id != expected_left_frame or right_key.frame_id != expected_right_frame:
        raise ValueError("local stereo image frame_id does not match the configured sensor frames")
    if camera_key != left_key or right_key.stamp_ns != left_key.stamp_ns:
        raise ValueError("local stereo images and CameraInfo must have one exact timestamp")

    left_size = (int(left_image.width), int(left_image.height))
    right_size = (int(right_image.width), int(right_image.height))
    camera_size = (int(camera_info.width), int(camera_info.height))
    if left_size[0] <= 0 or left_size[1] <= 0:
        raise ValueError("local stereo images must have positive width and height")
    if right_size != left_size or camera_size != left_size:
        raise ValueError(
            "local stereo image and CameraInfo resolutions must match exactly: "
            f"left={left_size}, right={right_size}, camera={camera_size}"
        )

    intrinsics = camera_intrinsics_from_matrices(camera_info.k, camera_info.p)
    if intrinsics is None:
        raise ValueError("local CameraInfo has no finite positive pinhole calibration")
    return LocalStereoGeometry(intrinsics=intrinsics)


__all__ = ["LocalStereoGeometry", "resolve_local_stereo_geometry"]
