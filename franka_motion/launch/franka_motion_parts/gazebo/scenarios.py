"""
Compatibility export for the canonical Gazebo scenario contract.

Responsibility:
- Preserve the historical ``franka_motion_parts.gazebo.scenarios`` import path.

Does not own:
- Gazebo scenario data or validation rules.
"""

from franka_motion_runtime.simulation.gazebo.scenarios import GazeboScenario

__all__ = ["GazeboScenario"]
