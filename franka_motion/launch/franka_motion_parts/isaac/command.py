"""
Compatibility export for canonical Isaac command construction.

Responsibility:
- Preserve the historical ``franka_motion_parts.isaac.command`` import path.

Does not own:
- Isaac runner command construction.
"""

from franka_motion_runtime.simulation.isaac.command import create_isaac_command
from franka_motion_runtime.simulation.isaac.contracts import (
    ISAAC_JOINT_COMMANDS_TOPIC,
    ISAAC_JOINT_STATES_TOPIC,
)

__all__ = [
    "ISAAC_JOINT_COMMANDS_TOPIC",
    "ISAAC_JOINT_STATES_TOPIC",
    "create_isaac_command",
]
