"""
Compatibility import for the process runtime extracted from bringup.

Responsibility:
- Preserve the temporary bringup import path during package cutover.

Does not own:
- Process lifecycle implementation or configuration validation.
"""

from franka_motion_runtime.processes.process_group_runner import (
    ProcessGroupRunner,
    ProcessGroupRunnerConfig,
)

__all__ = ["ProcessGroupRunner", "ProcessGroupRunnerConfig"]
