"""
Gazebo environment-variable action factories.

Responsibility:
- Build plugin search paths required by ``gz_ros2_control`` and camera sensors.
- Create launch environment actions for regular and table-camera Gazebo runs.

Does not own:
- Gazebo process commands or runtime plugin behavior.
"""

from __future__ import annotations

import os

from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration

ROS_JAZZY_LIB_PATH = "/opt/ros/jazzy/lib"


def gazebo_plugin_path(env_name: str) -> str:
    """Append the ROS Jazzy library directory to one Gazebo plugin path."""
    return os.pathsep.join(
        filter(
            None,
            [
                os.environ.get(env_name, ""),
                ROS_JAZZY_LIB_PATH,
            ],
        )
    )


def create_gazebo_plugin_environment() -> tuple[SetEnvironmentVariable, SetEnvironmentVariable]:
    """Create plugin-path actions for current and legacy Gazebo variables."""
    ign_plugin_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
        value=gazebo_plugin_path("IGN_GAZEBO_SYSTEM_PLUGIN_PATH"),
    )
    gz_plugin_path = SetEnvironmentVariable(
        name="GZ_SIM_SYSTEM_PLUGIN_PATH",
        value=gazebo_plugin_path("GZ_SIM_SYSTEM_PLUGIN_PATH"),
    )
    return ign_plugin_path, gz_plugin_path


def create_gazebo_camera_environment(
    fastdds_builtin_transports: LaunchConfiguration,
) -> tuple[SetEnvironmentVariable, SetEnvironmentVariable, SetEnvironmentVariable]:
    """Create Gazebo plugin and Fast DDS transport environment actions."""
    ign_plugin_path, gz_plugin_path = create_gazebo_plugin_environment()
    fastdds_transports = SetEnvironmentVariable(
        name="FASTDDS_BUILTIN_TRANSPORTS",
        value=fastdds_builtin_transports,
    )
    return ign_plugin_path, gz_plugin_path, fastdds_transports
