"""
Compatibility export for the canonical Isaac scenario contract.

Responsibility:
- Preserve the historical ``franka_motion_parts.isaac.scenarios`` import path.

Does not own:
- Isaac scenario data or validation rules.
"""

from franka_motion_runtime.simulation.isaac.scenarios import IsaacMoveItScenario

__all__ = ["IsaacMoveItScenario"]
