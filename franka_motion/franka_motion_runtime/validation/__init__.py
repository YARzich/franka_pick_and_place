"""
Reusable infrastructure for bounded Franka runtime validation.

Responsibility:
- Export the versioned runtime configuration and harness contracts.

Does not own:
- Simulator, planner or controller implementation.
"""

from franka_motion_runtime.validation.runtime_config import (
    CommandSpec,
    ConfigError,
    EvidenceKind,
    RuntimeMatrix,
    RuntimeProfile,
    load_runtime_matrix,
)

__all__ = [
    "CommandSpec",
    "ConfigError",
    "EvidenceKind",
    "RuntimeMatrix",
    "RuntimeProfile",
    "load_runtime_matrix",
]
