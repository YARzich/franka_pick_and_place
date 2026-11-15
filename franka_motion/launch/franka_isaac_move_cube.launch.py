"""
Send one visible cube-transfer request to an already running Isaac Sim stack.

Responsibility:
- Declare the concrete Isaac object and target pose for this task example.
- Delegate profile resolution, sequencing, and failure handling to the task runtime.

Does not own:
- Bringup, action implementation, PlanningScene leases, or grasp generation.
"""

from franka_motion_runtime.tasks import MoveObject, Pose
from franka_motion_runtime.tasks.launch import (
    create_isaac_transfer_sequence_description,
)


REQUESTS = (
    MoveObject(
        object_id="cube",
        target=Pose(
            frame_id="panda_link0",
            position=(0.55, 0.0, 0.03),
        ),
    ),
)


def generate_launch_description():
    """Create the concrete Isaac cube-transfer sequence."""
    launch_description = create_isaac_transfer_sequence_description(REQUESTS)
    return launch_description
