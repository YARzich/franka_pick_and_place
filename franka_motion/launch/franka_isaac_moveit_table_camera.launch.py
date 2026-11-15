"""
Canonical Isaac Sim table-camera MoveIt launch entrypoint.

Responsibility:
- Expose the stable public launch filename.

Does not own:
- Graph assembly or subsystem implementation.
"""

from launch import LaunchDescription

from franka_motion_runtime.bringup.isaac_table_camera_moveit import (
    create_isaac_table_camera_moveit_description,
)


def generate_launch_description() -> LaunchDescription:
    """Delegate graph assembly to the tested bringup helper."""
    launch_description = create_isaac_table_camera_moveit_description()
    return launch_description
