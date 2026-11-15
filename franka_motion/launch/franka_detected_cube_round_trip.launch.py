"""
Request one detected-cube transfer and return through the running action server.

Responsibility:
- Keep the editable task intent visible without exposing backend configuration.

Does not own:
- Robot/simulator bringup, perception, PlanningScene, MoveIt, or action execution.
"""

from franka_motion_runtime.tasks import TransferObjectIntent
from franka_motion_runtime.tasks.model import TranslationTarget
from franka_motion_runtime.tasks.launch import (
    create_transfer_sequence_description,
)


REQUESTS = (
    TransferObjectIntent.translation_from_current_source(
        object_id="cube",
        target=TranslationTarget(
            frame_id="panda_link0",
            vector=(0.15, 0.0, 0.0),
        ),
        return_home=False,
    ),
    TransferObjectIntent.initial_source(
        object_id="cube",
        return_home=True,
    ),
)


def generate_launch_description():
    """Create the standalone request client for an already running bringup."""
    launch_description = create_transfer_sequence_description(REQUESTS)
    return launch_description
