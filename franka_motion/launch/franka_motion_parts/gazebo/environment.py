"""
Compatibility exports for canonical Gazebo environment factories.

Responsibility:
- Preserve the historical ``franka_motion_parts.gazebo.environment`` import path.

Does not own:
- Gazebo plugin-path or launch-environment implementation.
"""

from franka_motion_runtime.simulation.gazebo.environment import (
    ROS_JAZZY_LIB_PATH,
    create_gazebo_camera_environment,
    create_gazebo_plugin_environment,
    gazebo_plugin_path,
)

__all__ = [
    "ROS_JAZZY_LIB_PATH",
    "create_gazebo_camera_environment",
    "create_gazebo_plugin_environment",
    "gazebo_plugin_path",
]
