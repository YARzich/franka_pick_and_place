"""
Static TF node factories for table stereo camera frames.

Responsibility:
- Create static transforms for table stereo camera link and optical frames.
- Keep REP-103 optical-frame rotations consistent across Gazebo and Isaac.

Does not own:
- Dynamic TF publication from robot_state_publisher.
- Camera sensor spawning or perception node remapping.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from franka_motion_runtime.bringup.launch_parts.camera.constants import (
    STEREO_LEFT_FRAME_ID,
    STEREO_LEFT_OPTICAL_FRAME_ID,
    STEREO_LEFT_WORLD_POSE,
    STEREO_RIGHT_FRAME_ID,
    STEREO_RIGHT_OPTICAL_FRAME_ID,
    STEREO_RIGHT_WORLD_POSE,
    WORLD_FRAME,
)


@dataclass(frozen=True)
class CameraTfActions:
    """World-camera and camera-optical static TF publishers."""

    left: Node
    right: Node
    left_optical: Node
    right_optical: Node


def _create_world_camera_tf_node(
    name: str,
    pose: dict[str, Any],
    parent_frame_id: str,
    child_frame_id: str,
    use_sim_time: LaunchConfiguration | bool,
) -> Node:
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=name,
        output="log",
        ros_arguments=["--log-level", "error"],
        arguments=[
            "--x",
            str(pose["x"]),
            "--y",
            str(pose["y"]),
            "--z",
            str(pose["z"]),
            "--roll",
            str(pose["roll"]),
            "--pitch",
            str(pose["pitch"]),
            "--yaw",
            str(pose["yaw"]),
            "--frame-id",
            parent_frame_id,
            "--child-frame-id",
            child_frame_id,
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )


def _create_optical_camera_tf_node(
    name: str,
    parent_frame_id: str,
    child_frame_id: str,
    use_sim_time: LaunchConfiguration | bool,
) -> Node:
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=name,
        output="log",
        ros_arguments=["--log-level", "error"],
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            "0.0",
            "--roll",
            "-1.57079632679",
            "--pitch",
            "0.0",
            "--yaw",
            "-1.57079632679",
            "--frame-id",
            parent_frame_id,
            "--child-frame-id",
            child_frame_id,
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )


def create_camera_tf_nodes(
    use_sim_time: LaunchConfiguration | bool,
    *,
    left_world_pose: dict[str, Any] | None = None,
    right_world_pose: dict[str, Any] | None = None,
    parent_frame_id: str = WORLD_FRAME,
) -> CameraTfActions:
    """Create the explicit REP-103 optical-frame TF chain."""
    left_pose = left_world_pose if left_world_pose is not None else STEREO_LEFT_WORLD_POSE
    right_pose = right_world_pose if right_world_pose is not None else STEREO_RIGHT_WORLD_POSE

    stereo_left_tf_node = _create_world_camera_tf_node(
        "table_stereo_left_static_tf",
        left_pose,
        parent_frame_id,
        STEREO_LEFT_FRAME_ID,
        use_sim_time,
    )
    stereo_right_tf_node = _create_world_camera_tf_node(
        "table_stereo_right_static_tf",
        right_pose,
        parent_frame_id,
        STEREO_RIGHT_FRAME_ID,
        use_sim_time,
    )
    stereo_left_optical_tf_node = _create_optical_camera_tf_node(
        "table_stereo_left_optical_static_tf",
        STEREO_LEFT_FRAME_ID,
        STEREO_LEFT_OPTICAL_FRAME_ID,
        use_sim_time,
    )
    stereo_right_optical_tf_node = _create_optical_camera_tf_node(
        "table_stereo_right_optical_static_tf",
        STEREO_RIGHT_FRAME_ID,
        STEREO_RIGHT_OPTICAL_FRAME_ID,
        use_sim_time,
    )
    return CameraTfActions(
        left=stereo_left_tf_node,
        right=stereo_right_tf_node,
        left_optical=stereo_left_optical_tf_node,
        right_optical=stereo_right_optical_tf_node,
    )
