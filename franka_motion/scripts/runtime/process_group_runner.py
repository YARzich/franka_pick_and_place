"""
Compatibility import for the canonical process-group lifecycle helper.

Responsibility:
- Preserve direct imports used by historical franka_motion tests and tools.

Does not own:
- Process lifecycle implementation or configuration validation.
"""

from franka_motion_runtime.processes.process_group_runner import (
    ProcessGroupRunner,
    ProcessGroupRunnerConfig,
)

__all__ = ["ProcessGroupRunner", "ProcessGroupRunnerConfig"]
