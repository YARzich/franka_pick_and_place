"""
Aggregate table-camera perception launch actions from backend-specific factories.

Responsibility:
- Build the typed perception action group used by table-camera launch entrypoints.
- Delegate concrete node/container construction to stereo, detector and pose modules.

Does not own:
- Startup ordering of the returned actions.
- Backend inference logic inside the launched ROS nodes.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from franka_motion_runtime.bringup.launch_parts.camera.image_bridge import (
    ImageBridgeActions,
    create_image_bridge_nodes,
)
from franka_motion_runtime.bringup.launch_parts.camera.pose_estimators import (
    FoundationPoseActions,
    MegaPoseActions,
    PoseEstimatorInputActions,
    create_foundationpose_actions,
    create_megapose_actions,
    create_pose_estimator_input_nodes,
)
from franka_motion_runtime.bringup.launch_parts.camera.stereo_depth import (
    EssActions,
    create_ess_actions,
    create_sgbm_node,
)
from franka_motion_runtime.bringup.launch_parts.camera.yolo import create_yolo_node


@dataclass(frozen=True)
class PerceptionActions:
    """Grouped image, stereo, detector and pose-estimator launch actions."""

    image_bridges: ImageBridgeActions
    stereo_pointcloud: Node
    ess: EssActions
    pose_input: PoseEstimatorInputActions
    foundationpose: FoundationPoseActions
    megapose: MegaPoseActions
    yolo_visualizer: Node


def create_perception_actions(
    paths: Any,
    configs: dict[str, LaunchConfiguration],
    conditions: dict[str, IfCondition],
) -> PerceptionActions:
    """Create the selected table-camera perception backend graph."""
    image_bridge_nodes = create_image_bridge_nodes(
        paths.managed_process_wrapper,
        configs["use_sim_time"],
    )
    sgbm_node = create_sgbm_node(configs, conditions)
    ess_actions = create_ess_actions(paths, configs, conditions)
    pose_input_nodes = create_pose_estimator_input_nodes(configs, conditions)
    foundationpose_actions = create_foundationpose_actions(paths, configs, conditions)
    megapose_actions = create_megapose_actions(configs, conditions)
    yolo_visualizer_node = create_yolo_node(paths.managed_process_wrapper, configs, conditions)

    return PerceptionActions(
        image_bridges=image_bridge_nodes,
        stereo_pointcloud=sgbm_node,
        ess=ess_actions,
        pose_input=pose_input_nodes,
        foundationpose=foundationpose_actions,
        megapose=megapose_actions,
        yolo_visualizer=yolo_visualizer_node,
    )
