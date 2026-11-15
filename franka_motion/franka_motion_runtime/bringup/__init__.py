"""
Typed orchestration contracts for Franka simulation bringup.

Responsibility:
- Export the small typed selection API used by bringup entrypoints.

Does not own:
- Launch actions, routes or any simulator and motion implementation.
"""

from franka_motion_runtime.bringup.contracts import (
    BringupSelection,
    InvalidBringupSelection,
    PlannerBackend,
    ScenarioProfile,
    SimulatorBackend,
)
from franka_motion_runtime.bringup.legacy_routes import (
    PUBLIC_COMPATIBILITY_LAUNCHES,
)

__all__ = [
    "BringupSelection",
    "InvalidBringupSelection",
    "PlannerBackend",
    "PUBLIC_COMPATIBILITY_LAUNCHES",
    "ScenarioProfile",
    "SimulatorBackend",
]
