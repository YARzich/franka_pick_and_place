"""
Pose-estimator input, FoundationPose and MegaPose node factories.

Responsibility:
- Create launch actions for pose-estimator input adapters and backend nodes.
- Wire FoundationPose, MegaPose and GigaPose topic remappings from shared camera constants.

Does not own:
- Detector bbox/mask generation or stereo depth computation.
- Pose-estimator runtime inference implementation.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue

from franka_motion_runtime.bringup.launch_parts.camera.constants import (
    FOUNDATIONPOSE_CAMERA_INFO_TOPIC,
    FOUNDATIONPOSE_DEPTH_TOPIC,
    FOUNDATIONPOSE_IMAGE_TOPIC,
    FOUNDATIONPOSE_OUTPUT_TOPIC,
    FOUNDATIONPOSE_SEGMENTATION_TOPIC,
)
from franka_motion_runtime.bringup.launch_parts.camera.parameters import table_camera_parameters
from franka_motion_runtime.bringup.launch_parts.constants import PERCEPTION_POSE_PACKAGE


@dataclass(frozen=True)
class PoseEstimatorInputActions:
    """Frame-normalization nodes shared by pose-estimator backends."""

    frame_fix: Node
    depth_clean: Node


@dataclass(frozen=True)
class FoundationPoseActions:
    """FoundationPose component container and output adapter actions."""

    container: ComposableNodeContainer
    pose_adapter: Node


@dataclass(frozen=True)
class MegaPoseActions:
    """MegaPose/GigaPose adapter and output-conversion actions."""

    detection: Node
    pose_adapter: Node


def create_pose_estimator_input_nodes(
    configs: dict[str, LaunchConfiguration],
    conditions: dict[str, IfCondition],
) -> PoseEstimatorInputActions:
    """Create stamped image/depth metadata normalization adapters."""
    foundationpose_input_frame_fix_node = Node(
        package=PERCEPTION_POSE_PACKAGE,
        executable="foundationpose_input_frame_fix_node.py",
        name="foundationpose_input_frame_fix",
        output="log",
        ros_arguments=["--log-level", "error"],
        condition=conditions["pose_estimator_input"],
        parameters=table_camera_parameters(configs),
    )
    foundationpose_depth_clean_node = Node(
        package=PERCEPTION_POSE_PACKAGE,
        executable="foundationpose_depth_clean_node.py",
        name="foundationpose_depth_clean",
        output="log",
        ros_arguments=["--log-level", "error"],
        condition=conditions["pose_estimator_input"],
        parameters=table_camera_parameters(configs),
    )
    return PoseEstimatorInputActions(
        frame_fix=foundationpose_input_frame_fix_node,
        depth_clean=foundationpose_depth_clean_node,
    )


def create_foundationpose_actions(
    paths: Any,
    configs: dict[str, LaunchConfiguration],
    conditions: dict[str, IfCondition],
) -> FoundationPoseActions:
    """Create the conditional FoundationPose component pipeline."""
    foundationpose_node = ComposableNode(
        package="isaac_ros_foundationpose",
        plugin="nvidia::isaac_ros::foundationpose::FoundationPoseNode",
        name="foundationpose",
        parameters=table_camera_parameters(configs),
        remappings=[
            ("pose_estimation/depth_image", FOUNDATIONPOSE_DEPTH_TOPIC),
            ("pose_estimation/image", FOUNDATIONPOSE_IMAGE_TOPIC),
            ("pose_estimation/camera_info", FOUNDATIONPOSE_CAMERA_INFO_TOPIC),
            ("pose_estimation/segmentation", FOUNDATIONPOSE_SEGMENTATION_TOPIC),
            ("pose_estimation/output", FOUNDATIONPOSE_OUTPUT_TOPIC),
        ],
    )
    foundationpose_container = ComposableNodeContainer(
        name="foundationpose_container",
        namespace="foundationpose",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[foundationpose_node],
        output="log",
        ros_arguments=["--log-level", "error"],
        condition=conditions["foundationpose"],
        parameters=[{"use_sim_time": configs["use_sim_time"]}],
        prefix=f"{paths.managed_process_wrapper} --child-name foundationpose_container --shutdown-signal kill --",
    )
    foundationpose_pose_adapter_node = Node(
        package=PERCEPTION_POSE_PACKAGE,
        executable="foundationpose_detection_to_pose_node.py",
        name="foundationpose_detection_to_pose",
        output="log",
        ros_arguments=["--log-level", "error"],
        condition=conditions["foundationpose"],
        parameters=table_camera_parameters(configs),
    )
    return FoundationPoseActions(
        container=foundationpose_container,
        pose_adapter=foundationpose_pose_adapter_node,
    )


def create_megapose_actions(
    configs: dict[str, LaunchConfiguration],
    conditions: dict[str, IfCondition],
) -> MegaPoseActions:
    """Create the conditional MegaPose/GigaPose worker pipeline."""
    megapose_detection_node = Node(
        package=PERCEPTION_POSE_PACKAGE,
        executable="megapose_detection_node.py",
        name="megapose_detection",
        output="screen",
        condition=conditions["megapose"],
        parameters=table_camera_parameters(
            configs,
            {
                "coarse_estimator": ParameterValue(
                    PythonExpression(
                        ["'gigapose' if '", configs["mode"], "'.lower() == 'gigapose' else 'megapose'"]
                    ),
                    value_type=str,
                )
            },
        ),
    )
    megapose_pose_adapter_node = Node(
        package=PERCEPTION_POSE_PACKAGE,
        executable="foundationpose_detection_to_pose_node.py",
        name="megapose_detection_to_pose",
        output="log",
        ros_arguments=["--log-level", "error"],
        condition=conditions["megapose"],
        parameters=table_camera_parameters(configs),
    )
    return MegaPoseActions(
        detection=megapose_detection_node,
        pose_adapter=megapose_pose_adapter_node,
    )
