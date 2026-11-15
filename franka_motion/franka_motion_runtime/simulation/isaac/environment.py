"""
Isaac Sim process environment and ROS-facing launch environment actions.

Responsibility:
- Build the Python and library environment needed by Isaac ROS 2 bridge runtime.
- Create ROS-facing environment actions such as RMW selection and Fast DDS profile.

Does not own:
- Isaac command construction or process startup.
- ROS node parameter wiring after the environment is set.
"""

from __future__ import annotations

import os
from pathlib import Path
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration

from .runtime.environment import sanitize_ros_environment


def isaac_root_from_python(isaac_python: str) -> Path:
    """Preserve binary-launcher compatibility; pip roots require in-process discovery."""
    launcher = Path(isaac_python).expanduser()
    if launcher.name != "python.sh":
        raise ValueError(
            "A Python interpreter does not identify Isaac resources; "
            "use resolve_isaac_root inside the selected interpreter."
        )
    return launcher.resolve().parent


def create_isaac_process_environment(isaac_python: str, rmw_implementation: str) -> dict[str, str]:
    """Isolate inherited ROS paths; the selected interpreter discovers its own root."""
    env = dict(os.environ)
    sanitize_ros_environment(environ=env, sys_path=[])
    return {
        "PYTHONNOUSERSITE": "1",
        "ROS_DISTRO": "jazzy",
        "RMW_IMPLEMENTATION": rmw_implementation,
        "AMENT_PREFIX_PATH": "",
        "COLCON_PREFIX_PATH": "",
        "ROS_PACKAGE_PATH": "",
        "PYTHONPATH": env.get("PYTHONPATH", ""),
        "LD_LIBRARY_PATH": env.get("LD_LIBRARY_PATH", ""),
    }


def create_isaac_environment_actions(
    *,
    include_gxf_log_level: bool = False,
) -> list[SetEnvironmentVariable]:
    """Create launch-level ROS and optional GXF environment actions."""
    ros_domain_env = SetEnvironmentVariable(
        name="ROS_DOMAIN_ID",
        value=LaunchConfiguration("ros_domain_id"),
    )
    rmw_env = SetEnvironmentVariable(
        name="RMW_IMPLEMENTATION",
        value=LaunchConfiguration("rmw_implementation"),
    )
    environment_actions = [
        ros_domain_env,
        rmw_env,
    ]
    if include_gxf_log_level:
        gxf_log_level_env = SetEnvironmentVariable(
            name="GXF_LOG_LEVEL",
            value=LaunchConfiguration("gxf_log_level"),
        )
        environment_actions.append(gxf_log_level_env)

    return environment_actions
