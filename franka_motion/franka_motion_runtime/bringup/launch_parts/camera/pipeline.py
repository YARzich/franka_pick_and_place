"""
Assemble table-camera perception actions into launch start groups.

Responsibility:
- Convert typed perception, TF and RViz action groups into the ordered camera pipeline list.
- Include simulator-specific image bridges only when the caller requests them.

Does not own:
- Creating individual launch actions.
- Mode conditions or dependency validation for optional backends.
"""

from __future__ import annotations

from franka_motion_runtime.bringup.launch_parts.camera.perception import PerceptionActions
from franka_motion_runtime.bringup.launch_parts.camera.rviz import RvizActions
from franka_motion_runtime.bringup.launch_parts.camera.tf import CameraTfActions


def create_camera_pipeline_actions(
    perception_actions: PerceptionActions,
    camera_tf_nodes: CameraTfActions,
    rviz_actions: RvizActions,
    *,
    include_image_bridges: bool,
) -> list[object]:
    """Flatten selected perception, TF, bridge and RViz actions."""
    image_bridge_actions = []
    if include_image_bridges:
        image_bridge_actions = [
            perception_actions.image_bridges.left,
            perception_actions.image_bridges.right,
        ]

    return [
        *image_bridge_actions,
        perception_actions.stereo_pointcloud,
        perception_actions.ess.disparity_frame_fix,
        perception_actions.ess.disparity_container,
        perception_actions.ess.depth_pointcloud,
        perception_actions.yolo_visualizer,
        camera_tf_nodes.left,
        camera_tf_nodes.right,
        camera_tf_nodes.left_optical,
        camera_tf_nodes.right_optical,
        perception_actions.pose_input.frame_fix,
        perception_actions.pose_input.depth_clean,
        perception_actions.foundationpose.container,
        perception_actions.foundationpose.pose_adapter,
        perception_actions.megapose.detection,
        perception_actions.megapose.pose_adapter,
        rviz_actions.delayed_start,
    ]
