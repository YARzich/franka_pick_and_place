"""
Compatibility export for canonical Gazebo world helpers.

Responsibility:
- Preserve the historical ``franka_motion_parts.gazebo.world`` import path.

Does not own:
- SDF parsing or clock-topic derivation.
"""

from franka_motion_runtime.simulation.gazebo.world import gazebo_clock_topic

__all__ = ["gazebo_clock_topic"]
