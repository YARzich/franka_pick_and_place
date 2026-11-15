"""
Gazebo image bridge nodes for the table stereo camera.

Responsibility:
- Create ros_gz_image bridge nodes for left/right stereo image transport.
- Apply the scenario's shared ROS clock policy to both bridge nodes.
- Apply managed shutdown prefixes for bridge processes started by launch.

Does not own:
- CameraInfo or clock bridge configuration.
- Stereo depth, detector or pose-estimator nodes.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from franka_motion_runtime.bringup.launch_parts.camera.constants import LEFT_IMAGE_TOPIC, RIGHT_IMAGE_TOPIC


@dataclass(frozen=True)
class ImageBridgeActions:
    """Managed left/right Gazebo image bridge processes."""

    left: Node
    right: Node


def image_bridge_shutdown_prefix(managed_process_wrapper: Path, child_name: str) -> str:
    """Build the bounded shutdown prefix for one native image bridge."""
    return f"{managed_process_wrapper} --child-name {child_name} --shutdown-signal kill --"


def create_image_bridge_nodes(
    managed_process_wrapper: Path,
    use_sim_time: LaunchConfiguration | bool = True,
) -> ImageBridgeActions:
    """
    Create image bridges with the selected ROS clock policy.

    The default preserves the historical one-argument Python helper used by
    legacy aliases; canonical bringup always passes its launch configuration.
    """
    left_camera_image_bridge_node = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name="table_stereo_left_image_bridge",
        output="log",
        arguments=[LEFT_IMAGE_TOPIC],
        parameters=[{"use_sim_time": use_sim_time}],
        prefix=image_bridge_shutdown_prefix(
            managed_process_wrapper,
            "table_stereo_left_image_bridge",
        ),
    )
    right_camera_image_bridge_node = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name="table_stereo_right_image_bridge",
        output="log",
        arguments=[RIGHT_IMAGE_TOPIC],
        parameters=[{"use_sim_time": use_sim_time}],
        prefix=image_bridge_shutdown_prefix(
            managed_process_wrapper,
            "table_stereo_right_image_bridge",
        ),
    )
    return ImageBridgeActions(
        left=left_camera_image_bridge_node,
        right=right_camera_image_bridge_node,
    )
