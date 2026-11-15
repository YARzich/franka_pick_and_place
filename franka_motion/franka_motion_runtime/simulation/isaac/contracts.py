"""
Stable topic/frame defaults and structural path contracts for Isaac Sim.

Responsibility:
    Centralize validated ROS topic, frame, prim-path, and graph-path defaults
    shared by Isaac launch and runtime modules.

Does not own:
    Runtime graph construction or ROS publication.
"""

from __future__ import annotations

from pathlib import Path
from typing import Protocol


ISAAC_JOINT_COMMANDS_TOPIC = "/isaac_joint_commands"
ISAAC_JOINT_STATES_TOPIC = "/isaac_joint_states"

STEREO_BASELINE_M = 0.06
STEREO_LEFT_OPTICAL_FRAME_ID = "table_stereo_left_optical_frame"
STEREO_RIGHT_OPTICAL_FRAME_ID = "table_stereo_right_optical_frame"
LEFT_IMAGE_TOPIC = "/table_stereo/left/image_raw"
RIGHT_IMAGE_TOPIC = "/table_stereo/right/image_raw"
LEFT_CAMERA_INFO_TOPIC = "/table_stereo/left/camera_info"
RIGHT_CAMERA_INFO_TOPIC = "/table_stereo/right/camera_info"


class CanonicalIsaacLaunchPathsLike(Protocol):
    """Package-local path contract used after the bringup cutover."""

    package_share: Path
    managed_process_wrapper: Path
    isaac_worlds: Path
    panda_usd: Path
    isaac_script: Path


class TransitionalIsaacLaunchPathsLike(Protocol):
    """Former bringup path contract retained until the launch cutover."""

    package_share: Path
    managed_process_wrapper: Path
    simulator_share: Path | None
    panda_usd: Path
    isaac_script: Path


IsaacLaunchPathsLike = CanonicalIsaacLaunchPathsLike | TransitionalIsaacLaunchPathsLike
