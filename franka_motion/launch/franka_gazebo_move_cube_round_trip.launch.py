"""
Move the Gazebo cube to the target and back to its original source pose.

Responsibility:
- Declare the two concrete object goals of the round-trip example.
- Delegate all resolution and action transport to the task launch helper.

Does not own:
- Bringup, object geometry, grasp transforms, or motion execution.
"""

from franka_motion_runtime.tasks import MoveObject, Pose, TransferObjectIntent
from franka_motion_runtime.tasks.launch import (
    create_gazebo_transfer_sequence_description,
)


REQUESTS = (
    MoveObject(
        object_id="cube",
        target=Pose(
            frame_id="panda_link0",
            position=(0.555, 0.0, 0.027),
        ),
    ),
    TransferObjectIntent.initial_source(
        object_id="cube",
    ),
)


def generate_launch_description():
    """Create one action request containing the complete round trip."""
    launch_description = create_gazebo_transfer_sequence_description(REQUESTS)
    return launch_description
