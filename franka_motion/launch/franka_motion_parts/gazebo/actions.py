"""
Compatibility exports for canonical Gazebo action factories.

Responsibility:
- Preserve the historical ``franka_motion_parts.gazebo.actions`` import path.

Does not own:
- Gazebo process or entity-spawn implementation.
"""

from franka_motion_runtime.simulation.gazebo.actions import (
    ROBOT_NAME,
    create_gazebo_processes,
    create_spawn_robot_node,
)

__all__ = ["ROBOT_NAME", "create_gazebo_processes", "create_spawn_robot_node"]
