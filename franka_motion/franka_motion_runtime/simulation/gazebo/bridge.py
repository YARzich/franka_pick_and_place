"""
Gazebo bridge configuration and process-action factories.

Responsibility:
- Write the one-way Gazebo-to-ROS clock bridge configuration.
- Create the managed bridge wrapper process used by Gazebo bringup.

Does not own:
- Camera-specific bridge topics, Gazebo startup, or entity spawning.
"""

from __future__ import annotations

from pathlib import Path
from typing import Protocol

from launch.actions import ExecuteProcess
import yaml


class GazeboBridgePathsLike(Protocol):
    """Minimal bringup-owned paths consumed by the Gazebo bridge factory."""

    bridge_wrapper: Path
    tmp_bridge_config: Path


def write_clock_bridge_config(tmp_bridge_config_path: Path, gz_clock_topic: str) -> None:
    """Write a one-way Gazebo-to-ROS clock bridge configuration."""
    clock_bridge_config = [
        {
            "ros_topic_name": "/clock",
            "gz_topic_name": gz_clock_topic,
            "ros_type_name": "rosgraph_msgs/msg/Clock",
            "gz_type_name": "gz.msgs.Clock",
            "direction": "GZ_TO_ROS",
            "lazy": False,
            "qos_profile": "CLOCK",
        }
    ]
    tmp_bridge_config_path.write_text(
        yaml.safe_dump(clock_bridge_config, sort_keys=False),
        encoding="utf-8",
    )


def create_clock_bridge_node(paths: GazeboBridgePathsLike) -> ExecuteProcess:
    """Create the managed bridge action using bringup-resolved paths."""
    return ExecuteProcess(
        cmd=[
            str(paths.bridge_wrapper),
            "--bridge-name",
            "clock_bridge",
            "--config-file",
            str(paths.tmp_bridge_config),
            "--log-level",
            "info",
        ],
        output="screen",
    )
