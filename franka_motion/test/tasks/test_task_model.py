"""
Test the public scenario request model without launching ROS processes.

Responsibility: Preserve fail-fast object, frame, pose, and recovery validation.
"""

import math

from manipulation_msgs.msg import TransferObjectStepV2
import pytest

from franka_motion_runtime.tasks.model import (
    MoveObject,
    Pose,
    TransferObjectIntent,
    TranslationTarget,
)


def test_valid_move_object_keeps_only_user_facing_goal_fields() -> None:
    """A normal request contains object identity and target policy only."""
    target = Pose(
        frame_id="panda_link0",
        position=(0.555, 0.0, 0.025),
    )
    request = MoveObject(object_id="cube", target=target)

    assert request.object_id == "cube"
    assert request.target is target
    assert request.return_home is True
    assert request.recovery_policy == TransferObjectStepV2.RECOVERY_SAFE_STOP
    intent = request.to_intent()
    assert intent.target_mode == TransferObjectStepV2.TARGET_ABSOLUTE_POSE
    assert intent.target_object_pose is target
    assert intent.target_translation is None


@pytest.mark.parametrize(
    ("kwargs", "message"),
    [
        ({"frame_id": "", "position": (0.0, 0.0, 0.0)}, "frame_id"),
        (
            {"frame_id": "/world", "position": (0.0, 0.0, 0.0)},
            "canonical relative",
        ),
        (
            {"frame_id": "world/", "position": (0.0, 0.0, 0.0)},
            "canonical relative",
        ),
        (
            {"frame_id": "robot//camera", "position": (0.0, 0.0, 0.0)},
            "canonical relative",
        ),
        ({"frame_id": "world", "position": (0.0, 0.0)}, "3 values"),
        (
            {"frame_id": "world", "position": (0.0, 0.0, math.inf)},
            "finite",
        ),
        (
            {
                "frame_id": "world",
                "position": (0.0, 0.0, 0.0),
                "orientation_xyzw": (0.0, 0.0, 0.0, 0.5),
            },
            "unit quaternion",
        ),
    ],
)
def test_pose_rejects_invalid_frames_and_geometry(kwargs, message) -> None:
    """Malformed targets must fail before launch creates a client process."""
    with pytest.raises(ValueError, match=message):
        Pose(**kwargs)


def test_move_object_rejects_invalid_identity_and_policy() -> None:
    """Unknown recovery values and whitespace IDs never reach the action."""
    target = Pose(frame_id="world", position=(0.0, 0.0, 0.0))
    with pytest.raises(ValueError, match="object_id"):
        MoveObject(object_id="bad object", target=target)
    with pytest.raises(ValueError, match="object_id"):
        MoveObject(object_id="bad|object", target=target)
    with pytest.raises(ValueError, match="recovery_policy"):
        MoveObject(object_id="cube", target=target, recovery_policy=255)
    with pytest.raises(TypeError, match="return_home"):
        MoveObject(object_id="cube", target=target, return_home=1)


def test_pose_normalizes_mutable_numeric_sequences() -> None:
    """A frozen request must not retain caller-owned mutable pose arrays."""
    position = [0, 1, 2]
    pose = Pose(frame_id="world", position=position)
    position[0] = 99

    assert pose.position == (0.0, 1.0, 2.0)


def test_target_modes_require_exactly_one_active_expression() -> None:
    """Inactive V2 target payloads cannot enter the message builder."""
    translation = TranslationTarget(
        frame_id="panda_link0",
        vector=(0.1, 0.0, 0.0),
    )
    relative = TransferObjectIntent.translation_from_current_source(
        object_id="cube",
        target=translation,
    )
    initial = TransferObjectIntent.initial_source(object_id="cube")

    assert relative.target_translation is translation
    assert relative.target_object_pose is None
    assert initial.target_object_pose is None
    assert initial.target_translation is None
    with pytest.raises(ValueError, match="initial-source"):
        TransferObjectIntent(
            object_id="cube",
            target_mode=TransferObjectStepV2.TARGET_INITIAL_SOURCE,
            target_object_pose=Pose(
                frame_id="panda_link0",
                position=(0.5, 0.0, 0.03),
            ),
        )
