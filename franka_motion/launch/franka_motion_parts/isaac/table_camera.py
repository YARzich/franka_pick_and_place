"""
Compatibility exports for canonical Isaac stereo-camera helpers.

Responsibility:
- Preserve the historical ``franka_motion_parts.isaac.table_camera`` import path.

Does not own:
- Stereo geometry, profile parsing or Isaac process-action implementation.
"""

from franka_motion_runtime.simulation.isaac.table_camera import (
    IsaacStereoRigSpec,
    create_isaac_stereo_world_poses,
    create_isaac_table_camera_process,
    load_gazebo_camera_profile,
)
from franka_motion_runtime.simulation.isaac.contracts import (
    LEFT_CAMERA_INFO_TOPIC,
    LEFT_IMAGE_TOPIC,
    RIGHT_CAMERA_INFO_TOPIC,
    RIGHT_IMAGE_TOPIC,
    STEREO_BASELINE_M,
    STEREO_LEFT_OPTICAL_FRAME_ID,
    STEREO_RIGHT_OPTICAL_FRAME_ID,
)

__all__ = [
    "IsaacStereoRigSpec",
    "LEFT_CAMERA_INFO_TOPIC",
    "LEFT_IMAGE_TOPIC",
    "RIGHT_CAMERA_INFO_TOPIC",
    "RIGHT_IMAGE_TOPIC",
    "STEREO_BASELINE_M",
    "STEREO_LEFT_OPTICAL_FRAME_ID",
    "STEREO_RIGHT_OPTICAL_FRAME_ID",
    "create_isaac_stereo_world_poses",
    "create_isaac_table_camera_process",
    "load_gazebo_camera_profile",
]
