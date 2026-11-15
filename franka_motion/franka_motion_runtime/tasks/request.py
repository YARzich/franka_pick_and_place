"""
Serialize transfer intents into the sequence client's flat parameters.

Responsibility:
- Convert immutable TransferObjectSequenceV2 intent to indexed ROS parameters.

Does not own:
- Observation, geometry, grasp resolution, action execution, or timeouts.
"""

from __future__ import annotations

from collections.abc import Sequence

from manipulation_msgs.msg import TransferObjectStepV2

from franka_motion_runtime.tasks.model import (
    Pose,
    TransferObjectIntent,
    TranslationTarget,
)


def _pose_parameters(prefix: str, pose: Pose) -> dict[str, object]:
    """Convert one validated absolute target to flat parameter names."""
    return {
        f"{prefix}.frame_id": pose.frame_id,
        f"{prefix}.position": list(pose.position),
        f"{prefix}.orientation_xyzw": list(pose.orientation_xyzw),
    }


def _translation_parameters(
    prefix: str,
    target: TranslationTarget,
) -> dict[str, object]:
    """Convert one validated relative target to flat parameter names."""
    return {
        f"{prefix}.frame_id": target.frame_id,
        f"{prefix}.vector": list(target.vector),
    }


def transfer_sequence_parameters(
    steps: Sequence[TransferObjectIntent],
) -> dict[str, object]:
    """Build a parameter payload containing only V2 client-owned fields."""
    if not steps:
        raise ValueError("transfer sequence must contain at least one intent")
    parameters: dict[str, object] = {"steps.count": len(steps)}
    for index, step in enumerate(steps):
        if not isinstance(step, TransferObjectIntent):
            raise TypeError("steps must contain only TransferObjectIntent values")
        prefix = f"steps.{index}"
        parameters.update(
            {
                f"{prefix}.object_id": step.object_id,
                f"{prefix}.target_mode": step.target_mode,
                f"{prefix}.return_home": step.return_home,
                f"{prefix}.recovery_policy": step.recovery_policy,
            }
        )
        if step.target_mode == TransferObjectStepV2.TARGET_ABSOLUTE_POSE:
            parameters.update(
                _pose_parameters(
                    f"{prefix}.target_object_pose",
                    step.target_object_pose,
                )
            )
        elif (
            step.target_mode
            == TransferObjectStepV2.TARGET_TRANSLATION_FROM_CURRENT_SOURCE
        ):
            parameters.update(
                _translation_parameters(
                    f"{prefix}.target_translation",
                    step.target_translation,
                )
            )
    return parameters
