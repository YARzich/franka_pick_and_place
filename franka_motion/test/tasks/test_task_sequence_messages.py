"""
Test parameter and wire mapping for TransferObjectSequenceV2 intent.

Responsibility:
- Preserve intent-only parameter round trips and the common-stamp contract.
"""

from builtin_interfaces.msg import Time
from manipulation_msgs.action import TransferObjectSequenceV2
from manipulation_msgs.msg import TransferObjectStepV2
import pytest

from franka_motion_runtime.tasks.model import (
    Pose,
    TransferObjectIntent,
    TranslationTarget,
)
from franka_motion_runtime.tasks.request import transfer_sequence_parameters
from franka_motion_runtime.tasks.request_parsing import transfer_steps_from_parameters
from franka_motion_runtime.tasks.sequence_messages import build_transfer_sequence_goal


def _steps() -> tuple[TransferObjectIntent, ...]:
    """Return all three target modes without server-owned resolution data."""
    return (
        TransferObjectIntent.absolute_pose(
            object_id="cube",
            target=Pose(
                frame_id="panda_link0",
                position=(0.555, 0.0, 0.027),
            ),
            return_home=False,
        ),
        TransferObjectIntent.translation_from_current_source(
            object_id="cube",
            target=TranslationTarget(
                frame_id="panda_link0",
                vector=(-0.15, 0.0, 0.0),
            ),
            return_home=False,
        ),
        TransferObjectIntent.initial_source(object_id="cube"),
    )


def test_flat_parameter_mapping_round_trips_only_v2_intent() -> None:
    """Indexed launch parameters contain no source, grasp, geometry, or timeout."""
    steps = _steps()
    parameters = transfer_sequence_parameters(steps)

    assert parameters["steps.count"] == 3
    assert parameters["steps.0.object_id"] == "cube"
    assert parameters["steps.0.target_mode"] == 0
    assert parameters["steps.1.target_mode"] == 1
    assert parameters["steps.2.target_mode"] == 2
    assert transfer_steps_from_parameters(parameters) == steps
    forbidden_fragments = (
        "timeout",
        "dimensions",
        "source_object_pose",
        "grasp_pose",
        "provider",
        "strategy",
    )
    assert not any(
        fragment in name
        for name in parameters
        for fragment in forbidden_fragments
    )


def test_goal_uses_common_stamp_only_for_active_target_payloads() -> None:
    """Absolute and relative targets are stamped; initial-source stays default."""
    goal = build_transfer_sequence_goal(
        _steps(),
        stamp=Time(sec=7, nanosec=123),
    )

    assert isinstance(goal, TransferObjectSequenceV2.Goal)
    assert not hasattr(goal, "timeout")
    absolute, relative, initial = goal.steps
    assert absolute.target_mode == TransferObjectStepV2.TARGET_ABSOLUTE_POSE
    assert (
        absolute.target_object_pose.header.stamp.sec,
        absolute.target_object_pose.header.stamp.nanosec,
    ) == (7, 123)
    assert absolute.target_translation.header.frame_id == ""
    assert (
        relative.target_translation.header.stamp.sec,
        relative.target_translation.header.stamp.nanosec,
    ) == (7, 123)
    assert relative.target_object_pose.header.frame_id == ""
    assert initial.target_object_pose.header.frame_id == ""
    assert initial.target_translation.header.frame_id == ""


def test_goal_rejects_empty_steps_and_invalid_stamp() -> None:
    """Malformed client construction fails before action transport starts."""
    with pytest.raises(ValueError, match="at least one"):
        build_transfer_sequence_goal((), stamp=Time(sec=1))
    with pytest.raises(ValueError, match="non-zero"):
        build_transfer_sequence_goal(_steps(), stamp=Time())
    with pytest.raises(ValueError, match="normalized"):
        build_transfer_sequence_goal(
            _steps(),
            stamp=Time(sec=1, nanosec=1_000_000_000),
        )


def test_parameter_parser_rejects_boolean_target_mode() -> None:
    """Python bool equality cannot silently select translation mode one."""
    parameters = transfer_sequence_parameters(_steps())
    parameters["steps.0.target_mode"] = True

    with pytest.raises(ValueError, match="target_mode must be an integer"):
        transfer_steps_from_parameters(parameters)
