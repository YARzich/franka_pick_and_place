"""
Compatibility exports for canonical Isaac stereo-camera helpers.

Responsibility:
    Preserve legacy imports for canonical stereo prim and visual-model helpers
    without owning camera objects or ROS publishers.
"""

from franka_motion_runtime.simulation.isaac.runtime.stereo import (
    StereoCameraPaths,
    create_stereo_camera_prims,
    create_stereo_visual_model,
)

__all__ = [
    "StereoCameraPaths",
    "create_stereo_camera_prims",
    "create_stereo_visual_model",
]
