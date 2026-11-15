"""
Expose the small scenario-task API used by public Franka launch files.

Responsibility:
- Export typed object-move requests without importing ROS launch machinery.

Does not own:
- Robot control, planning, perception, or PlanningScene transactions.
"""

from franka_motion_runtime.tasks.model import MoveObject, Pose, TransferObjectIntent


__all__ = [
    "MoveObject",
    "Pose",
    "TransferObjectIntent",
]
