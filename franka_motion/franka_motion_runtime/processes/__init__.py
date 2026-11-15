"""
Bounded subprocess lifecycle helpers for simulator bringup.

Responsibility:
- Export the process-group runner shared by launch-managed wrappers.

Does not own:
- Simulator commands, ROS nodes, or scenario-specific shutdown policy.
"""

from franka_motion_runtime.processes.process_group_runner import (
    ProcessGroupRunner,
    ProcessGroupRunnerConfig,
)

__all__ = ["ProcessGroupRunner", "ProcessGroupRunnerConfig"]
