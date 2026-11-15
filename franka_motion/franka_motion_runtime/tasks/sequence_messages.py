"""
Map pure Franka transfer intents to TransferObjectSequenceV2 messages.

Responsibility:
- Apply one common ROS timestamp to the active target expression of each step.
- Construct the complete V2 action goal without reading parameters or ROS state.

Does not own:
- Object observation, geometry, grasp resolution, transport, or timeout policy.
"""

from __future__ import annotations

from typing import Sequence

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from manipulation_msgs.action import TransferObjectSequenceV2
from manipulation_msgs.msg import TransferObjectStepV2

from franka_motion_runtime.tasks.model import (
    Pose,
    TransferObjectIntent,
    TranslationTarget,
)


def _validate_stamp(stamp: Time) -> None:
    """Require one normalized non-zero ROS timestamp for stamped targets."""
    if not isinstance(stamp, Time):
        raise TypeError("stamp must be builtin_interfaces/Time")
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        raise ValueError("goal stamp must be normalized")
    if stamp.sec == 0 and stamp.nanosec == 0:
        raise ValueError("transfer sequence goal requires a non-zero ROS stamp")


def _stamped_pose(pose: Pose, stamp: Time) -> PoseStamped:
    """Create one absolute pose using the goal-wide common timestamp."""
    message = PoseStamped()
    message.header.frame_id = pose.frame_id
    message.header.stamp.sec = stamp.sec
    message.header.stamp.nanosec = stamp.nanosec
    message.pose.position.x = pose.position[0]
    message.pose.position.y = pose.position[1]
    message.pose.position.z = pose.position[2]
    message.pose.orientation.x = pose.orientation_xyzw[0]
    message.pose.orientation.y = pose.orientation_xyzw[1]
    message.pose.orientation.z = pose.orientation_xyzw[2]
    message.pose.orientation.w = pose.orientation_xyzw[3]
    return message


def _stamped_translation(
    target: TranslationTarget,
    stamp: Time,
) -> Vector3Stamped:
    """Create one relative translation using the common goal timestamp."""
    message = Vector3Stamped()
    message.header.frame_id = target.frame_id
    message.header.stamp.sec = stamp.sec
    message.header.stamp.nanosec = stamp.nanosec
    message.vector.x = target.vector[0]
    message.vector.y = target.vector[1]
    message.vector.z = target.vector[2]
    return message


def _step_message(
    intent: TransferObjectIntent,
    stamp: Time,
) -> TransferObjectStepV2:
    """Map exactly one validated target expression to the V2 wire step."""
    if not isinstance(intent, TransferObjectIntent):
        raise TypeError("steps must contain only TransferObjectIntent values")
    message = TransferObjectStepV2()
    message.object_id = intent.object_id
    message.target_mode = intent.target_mode
    if intent.target_mode == TransferObjectStepV2.TARGET_ABSOLUTE_POSE:
        message.target_object_pose = _stamped_pose(
            intent.target_object_pose,
            stamp,
        )
    elif (
        intent.target_mode
        == TransferObjectStepV2.TARGET_TRANSLATION_FROM_CURRENT_SOURCE
    ):
        message.target_translation = _stamped_translation(
            intent.target_translation,
            stamp,
        )
    message.return_home = intent.return_home
    message.recovery_policy = intent.recovery_policy
    return message


def build_transfer_sequence_goal(
    steps: Sequence[TransferObjectIntent],
    *,
    stamp: Time,
) -> TransferObjectSequenceV2.Goal:
    """Create one timeout-free V2 goal from client-owned intent only."""
    if not steps:
        raise ValueError("transfer sequence goal must contain at least one step")
    _validate_stamp(stamp)
    goal = TransferObjectSequenceV2.Goal()
    goal.steps = [_step_message(step, stamp) for step in steps]
    return goal
