"""
Generic typed entrypoint for canonical Franka simulation bringup.

Responsibility:
- Expose fail-fast simulator, scenario and planner selection.
- Delegate one validated route through the typed routing factory.

Does not own:
- Leaf launch arguments, nodes, processes or backend implementation.
- Compatibility wrappers retained in the franka_motion facade package.
"""

from launch import LaunchDescription

from franka_motion_runtime.bringup.launch_factory import (
    create_typed_bringup_description,
)


def generate_launch_description() -> LaunchDescription:
    """Create the typed generic bringup launch description."""
    launch_description = create_typed_bringup_description()
    return launch_description
