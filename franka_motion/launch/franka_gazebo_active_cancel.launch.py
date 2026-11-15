"""
Cancel one visible Gazebo cube transfer during active approach motion.

Responsibility:
- Declare the concrete cube target and persistent table fixture.
- Delegate resolution, exact cancellation, and evidence checks to task runtime.

Does not own:
- Bringup, object geometry, action implementation, or PlanningScene mutation.
"""

from franka_motion_runtime.tasks import MoveObject, Pose
from franka_motion_runtime.tasks.active_cancel_launch import (
    create_gazebo_active_cancel_description,
)


REQUESTS = (
    MoveObject(
        object_id="cube",
        target=Pose(
            frame_id="panda_link0",
            position=(0.555, 0.0, 0.027),
        ),
    ),
)
PERSISTENT_WORLD_OBJECT_IDS = ("table",)
CONTROLLER_ACTION_NAME = "/panda_arm_controller/follow_joint_trajectory"


def generate_launch_description():
    """Create the concrete active-cancel cube request."""
    launch_description = create_gazebo_active_cancel_description(
        REQUESTS,
        controller_action_name=CONTROLLER_ACTION_NAME,
        persistent_world_object_ids=PERSISTENT_WORLD_OBJECT_IDS,
    )
    return launch_description
