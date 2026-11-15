"""
Define one server-resolved V2 object-transfer intent.

Responsibility:
- Validate object identity, target-mode payload, return-home, and recovery policy.

Does not own:
- Object geometry, observation/grasp providers, planning, or action transport.
"""

from __future__ import annotations

from dataclasses import dataclass

from manipulation_msgs.msg import TransferObjectStepV2

from franka_motion_runtime.tasks.geometry import Pose, TranslationTarget
from franka_motion_runtime.tasks.task_validation import (
    validate_identifier,
    validate_recovery_policy,
)


@dataclass(frozen=True)
class TransferObjectIntent:
    """Pure client-owned fields of one TransferObjectStepV2 message."""

    object_id: str
    target_mode: int
    target_object_pose: Pose | None = None
    target_translation: TranslationTarget | None = None
    return_home: bool = True
    recovery_policy: int = TransferObjectStepV2.RECOVERY_SAFE_STOP

    def __post_init__(self) -> None:
        """Require exactly the target expression selected by target_mode."""
        validate_identifier(self.object_id, field="object_id")
        if not isinstance(self.return_home, bool):
            raise TypeError("return_home must be bool")
        validate_recovery_policy(self.recovery_policy)
        modes = {
            TransferObjectStepV2.TARGET_ABSOLUTE_POSE,
            TransferObjectStepV2.TARGET_TRANSLATION_FROM_CURRENT_SOURCE,
            TransferObjectStepV2.TARGET_INITIAL_SOURCE,
        }
        if (
            isinstance(self.target_mode, bool)
            or not isinstance(self.target_mode, int)
            or self.target_mode not in modes
        ):
            raise ValueError("target_mode is outside the TransferObjectStepV2 contract")

        if self.target_mode == TransferObjectStepV2.TARGET_ABSOLUTE_POSE:
            if not isinstance(self.target_object_pose, Pose):
                raise TypeError("absolute target requires target_object_pose")
            if self.target_translation is not None:
                raise ValueError("absolute target cannot include target_translation")
        elif (
            self.target_mode
            == TransferObjectStepV2.TARGET_TRANSLATION_FROM_CURRENT_SOURCE
        ):
            if self.target_object_pose is not None:
                raise ValueError("translation target cannot include target_object_pose")
            if not isinstance(self.target_translation, TranslationTarget):
                raise TypeError("translation target requires target_translation")
        elif self.target_object_pose is not None or self.target_translation is not None:
            raise ValueError("initial-source target cannot include another target payload")

    @classmethod
    def absolute_pose(
        cls,
        *,
        object_id: str,
        target: Pose,
        return_home: bool = True,
        recovery_policy: int = TransferObjectStepV2.RECOVERY_SAFE_STOP,
    ) -> "TransferObjectIntent":
        """Create an absolute-pose target expression."""
        return cls(
            object_id=object_id,
            target_mode=TransferObjectStepV2.TARGET_ABSOLUTE_POSE,
            target_object_pose=target,
            return_home=return_home,
            recovery_policy=recovery_policy,
        )

    @classmethod
    def translation_from_current_source(
        cls,
        *,
        object_id: str,
        target: TranslationTarget,
        return_home: bool = True,
        recovery_policy: int = TransferObjectStepV2.RECOVERY_SAFE_STOP,
    ) -> "TransferObjectIntent":
        """Create a translation relative to the server's fresh observation."""
        return cls(
            object_id=object_id,
            target_mode=TransferObjectStepV2.TARGET_TRANSLATION_FROM_CURRENT_SOURCE,
            target_translation=target,
            return_home=return_home,
            recovery_policy=recovery_policy,
        )

    @classmethod
    def initial_source(
        cls,
        *,
        object_id: str,
        return_home: bool = True,
        recovery_policy: int = TransferObjectStepV2.RECOVERY_SAFE_STOP,
    ) -> "TransferObjectIntent":
        """Create a return to this goal's first observed source pose."""
        return cls(
            object_id=object_id,
            target_mode=TransferObjectStepV2.TARGET_INITIAL_SOURCE,
            return_home=return_home,
            recovery_policy=recovery_policy,
        )
