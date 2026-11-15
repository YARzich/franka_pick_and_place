"""
Pure validation for Isaac runner configuration.

Responsibility:
- Reject unsafe geometry, names and loop limits before importing Isaac Sim.
- Keep detailed validation independent from argparse and filesystem access.

Does not own:
- Parsing command-line arguments or checking whether asset paths exist.
- Isaac scene creation or runtime error recovery.
"""

from __future__ import annotations

import math
import re
from typing import Sequence, TYPE_CHECKING

from franka_motion_runtime.frame_id import (
    is_canonical_relative_frame_id,
)

if TYPE_CHECKING:
    from .config import (
        CubeConfig,
        IsaacRunnerConfig,
        StereoCameraConfig,
    )

_ABSOLUTE_NAME_PATTERN = re.compile(
    r"^/(?:[A-Za-z_][A-Za-z0-9_]*)(?:/[A-Za-z_][A-Za-z0-9_]*)*$"
)
_USD_PRIM_PATH_PATTERN = _ABSOLUTE_NAME_PATTERN


def validate_runner_config(config: IsaacRunnerConfig) -> None:
    """Validate all non-filesystem runner invariants before Isaac startup."""
    if config.time_steps < 0:
        raise ValueError("time_steps must be zero or positive")
    _require_nonempty(config.ros_distro, "ros_distro")
    _require_absolute_name(config.command_topic, "command_topic")
    _require_absolute_name(config.state_topic, "state_topic")
    _require_usd_prim_path(config.scene_prim, "scene_prim")
    _validate_cube(config.cube)
    _validate_stereo(config.stereo)


def _validate_cube(cube: CubeConfig) -> None:
    _require_positive(cube.size, "cube.size")
    _require_finite_vector(cube.position, 3, "cube.position")
    _require_nonnegative(cube.static_friction, "cube.static_friction")
    _require_nonnegative(cube.dynamic_friction, "cube.dynamic_friction")
    _require_finite(cube.restitution, "cube.restitution")
    if not 0.0 <= cube.restitution <= 1.0:
        raise ValueError("cube.restitution must be in the [0, 1] range")
    _require_usd_prim_path(cube.prim, "cube.prim")


def _validate_stereo(stereo: StereoCameraConfig) -> None:
    _require_finite_vector(stereo.center_position, 3, "stereo.center_position")
    _require_positive(stereo.baseline_m, "stereo.baseline_m")
    _require_finite(stereo.yaw_rad, "stereo.yaw_rad")
    _require_finite(stereo.pitch_down_rad, "stereo.pitch_down_rad")
    _require_positive(stereo.look_distance_m, "stereo.look_distance_m")
    if stereo.width <= 0 or stereo.height <= 0:
        raise ValueError("stereo width and height must be positive")
    _require_positive(stereo.focal_length_mm, "stereo.focal_length_mm")
    _require_positive(
        stereo.horizontal_aperture_mm,
        "stereo.horizontal_aperture_mm",
    )
    _require_positive(stereo.vertical_aperture_mm, "stereo.vertical_aperture_mm")
    _require_positive(stereo.focus_distance_m, "stereo.focus_distance_m")
    _require_positive(stereo.near_clip_m, "stereo.near_clip_m")
    _require_positive(stereo.far_clip_m, "stereo.far_clip_m")
    if stereo.near_clip_m >= stereo.far_clip_m:
        raise ValueError("stereo.near_clip_m must be less than far_clip_m")
    _require_finite(
        stereo.camera_local_z_offset_m,
        "stereo.camera_local_z_offset_m",
    )

    topic_fields = {
        "stereo.left_image_topic": stereo.left_image_topic,
        "stereo.right_image_topic": stereo.right_image_topic,
        "stereo.left_camera_info_topic": stereo.left_camera_info_topic,
        "stereo.right_camera_info_topic": stereo.right_camera_info_topic,
    }
    for field_name, value in topic_fields.items():
        _require_absolute_name(value, field_name)

    _require_frame_id(stereo.left_frame_id, "stereo.left_frame_id")
    _require_frame_id(stereo.right_frame_id, "stereo.right_frame_id")


def _require_finite_vector(
    values: Sequence[float],
    expected_size: int,
    field_name: str,
) -> None:
    if len(values) != expected_size:
        raise ValueError(f"{field_name} must contain exactly {expected_size} values")
    for value in values:
        _require_finite(value, field_name)


def _require_positive(value: float, field_name: str) -> None:
    _require_finite(value, field_name)
    if value <= 0.0:
        raise ValueError(f"{field_name} must be positive")


def _require_nonnegative(value: float, field_name: str) -> None:
    _require_finite(value, field_name)
    if value < 0.0:
        raise ValueError(f"{field_name} must be zero or positive")


def _require_finite(value: float, field_name: str) -> None:
    if not math.isfinite(value):
        raise ValueError(f"{field_name} must be finite")


def _require_nonempty(value: str, field_name: str) -> None:
    if not value or value != value.strip():
        raise ValueError(f"{field_name} must be a non-empty trimmed string")


def _require_absolute_name(value: str, field_name: str) -> None:
    if not _ABSOLUTE_NAME_PATTERN.fullmatch(value):
        raise ValueError(f"{field_name} must be a valid absolute ROS name")


def _require_usd_prim_path(value: str, field_name: str) -> None:
    if not _USD_PRIM_PATH_PATTERN.fullmatch(value):
        raise ValueError(f"{field_name} must be a valid absolute USD prim path")


def _require_frame_id(value: str, field_name: str) -> None:
    if not is_canonical_relative_frame_id(value):
        raise ValueError(f"{field_name} must be a canonical relative TF frame id")
