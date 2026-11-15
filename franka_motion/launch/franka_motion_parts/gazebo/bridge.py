"""
Compatibility exports for canonical Gazebo bridge factories.

Responsibility:
- Preserve the historical ``franka_motion_parts.gazebo.bridge`` import path.

Does not own:
- Bridge configuration or process-action implementation.
"""

from franka_motion_runtime.simulation.gazebo.bridge import (
    GazeboBridgePathsLike,
    create_clock_bridge_node,
    write_clock_bridge_config,
)

__all__ = [
    "GazeboBridgePathsLike",
    "create_clock_bridge_node",
    "write_clock_bridge_config",
]
