"""
Parse the V2 sequence client's flat ROS parameter contract.

Responsibility:
- Reconstruct validated immutable target expressions from indexed parameters.

Does not own:
- Parameter declaration, server-side resolution, transport, or scene ownership.
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import cast

from manipulation_msgs.msg import TransferObjectStepV2

from franka_motion_runtime.tasks.model import (
    Pose,
    Quaternion,
    TransferObjectIntent,
    TranslationTarget,
    Vector3,
)


def _required(mapping: Mapping[str, object], name: str) -> object:
    """Read one required flat parameter with a useful schema error."""
    try:
        return mapping[name]
    except KeyError as error:
        raise ValueError(f"missing sequence parameter '{name}'") from error


def _pose_from_parameters(mapping: Mapping[str, object], prefix: str) -> Pose:
    """Reconstruct one validated absolute target pose."""
    return Pose(
        frame_id=cast(str, _required(mapping, f"{prefix}.frame_id")),
        position=cast(Vector3, _required(mapping, f"{prefix}.position")),
        orientation_xyzw=cast(
            Quaternion,
            _required(mapping, f"{prefix}.orientation_xyzw"),
        ),
    )


def _translation_from_parameters(
    mapping: Mapping[str, object],
    prefix: str,
) -> TranslationTarget:
    """Reconstruct one validated relative target vector."""
    return TranslationTarget(
        frame_id=cast(str, _required(mapping, f"{prefix}.frame_id")),
        vector=cast(Vector3, _required(mapping, f"{prefix}.vector")),
    )


def transfer_steps_from_parameters(
    mapping: Mapping[str, object],
) -> tuple[TransferObjectIntent, ...]:
    """Parse indexed client parameters into V2 transfer intents."""
    count = _required(mapping, "steps.count")
    if isinstance(count, bool) or not isinstance(count, int) or count <= 0:
        raise ValueError("steps.count must be a positive integer")
    steps = []
    for index in range(count):
        prefix = f"steps.{index}"
        target_mode = cast(int, _required(mapping, f"{prefix}.target_mode"))
        if isinstance(target_mode, bool) or not isinstance(target_mode, int):
            raise ValueError(f"steps.{index}.target_mode must be an integer")
        common = {
            "object_id": cast(str, _required(mapping, f"{prefix}.object_id")),
            "return_home": cast(
                bool,
                _required(mapping, f"{prefix}.return_home"),
            ),
            "recovery_policy": cast(
                int,
                _required(mapping, f"{prefix}.recovery_policy"),
            ),
        }
        if target_mode == TransferObjectStepV2.TARGET_ABSOLUTE_POSE:
            step = TransferObjectIntent.absolute_pose(
                target=_pose_from_parameters(
                    mapping,
                    f"{prefix}.target_object_pose",
                ),
                **common,
            )
        elif (
            target_mode
            == TransferObjectStepV2.TARGET_TRANSLATION_FROM_CURRENT_SOURCE
        ):
            step = TransferObjectIntent.translation_from_current_source(
                target=_translation_from_parameters(
                    mapping,
                    f"{prefix}.target_translation",
                ),
                **common,
            )
        elif target_mode == TransferObjectStepV2.TARGET_INITIAL_SOURCE:
            step = TransferObjectIntent.initial_source(**common)
        else:
            raise ValueError(
                f"steps.{index}.target_mode is outside the V2 contract"
            )
        steps.append(step)
    return tuple(steps)
