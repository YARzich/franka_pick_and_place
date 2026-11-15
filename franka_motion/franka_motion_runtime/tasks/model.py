"""
Expose concise scenario-facing V2 transfer values.

Responsibility:
- Define the absolute-move convenience request and re-export intent geometry.

Does not own:
- Object catalogs, providers, planning, launch actions, or action transport.
"""

from __future__ import annotations

from dataclasses import dataclass

from manipulation_msgs.msg import TransferObjectStepV2

from franka_motion_runtime.tasks.geometry import (
    Pose,
    Quaternion,
    TranslationTarget,
    Vector3,
)
from franka_motion_runtime.tasks.intent import TransferObjectIntent
from franka_motion_runtime.tasks.task_validation import (
    validate_identifier,
    validate_recovery_policy,
)


@dataclass(frozen=True)
class MoveObject:
    """Public convenience request for one absolute-pose V2 transfer intent."""

    object_id: str
    target: Pose
    return_home: bool = True
    recovery_policy: int = TransferObjectStepV2.RECOVERY_SAFE_STOP

    def __post_init__(self) -> None:
        """Reject malformed requests before launch creates any process."""
        validate_identifier(self.object_id, field="object_id")
        if not isinstance(self.target, Pose):
            raise TypeError("target must be Pose")
        if not isinstance(self.return_home, bool):
            raise TypeError("return_home must be bool")
        validate_recovery_policy(self.recovery_policy)

    def to_intent(self) -> TransferObjectIntent:
        """Discard no client data while selecting the V2 absolute target mode."""
        return TransferObjectIntent.absolute_pose(
            object_id=self.object_id,
            target=self.target,
            return_home=self.return_home,
            recovery_policy=self.recovery_policy,
        )


__all__ = [
    "MoveObject",
    "Pose",
    "Quaternion",
    "TransferObjectIntent",
    "TranslationTarget",
    "Vector3",
]
