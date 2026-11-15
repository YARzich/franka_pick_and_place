"""
Gazebo table-camera bringup orchestration and compatibility exports.

Responsibility:
- Coordinate table-camera startup with controllers, MoveIt and perception.
- Preserve historical imports for canonical Gazebo camera factories.

Does not own:
- Gazebo entity/bridge implementation or camera topic constants.
- Controller, MoveIt or perception backend implementations.
"""

from __future__ import annotations

from franka_motion_runtime.bringup.launch_parts.gazebo_table_camera import (
    GazeboTableCameraActions,
    GazeboTableCameraMotionActions,
    create_gazebo_table_camera_start_events,
)
from franka_motion_runtime.simulation.gazebo.table_camera import (
    CAMERA_NAME,
    CAMERA_POSE,
    LEFT_CAMERA_INFO_TOPIC,
    RIGHT_CAMERA_INFO_TOPIC,
    create_spawn_camera_node,
    create_table_camera_bridge_node,
    write_table_camera_bridge_config,
)

__all__ = [
    "CAMERA_NAME",
    "CAMERA_POSE",
    "GazeboTableCameraActions",
    "GazeboTableCameraMotionActions",
    "LEFT_CAMERA_INFO_TOPIC",
    "RIGHT_CAMERA_INFO_TOPIC",
    "create_gazebo_table_camera_start_events",
    "create_spawn_camera_node",
    "create_table_camera_bridge_node",
    "write_table_camera_bridge_config",
]
