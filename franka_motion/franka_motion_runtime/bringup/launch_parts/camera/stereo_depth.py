"""
SGBM and ESS stereo-depth pipeline node factories.

Responsibility:
- Create the local SGBM pointcloud node for non-ESS modes.
- Create Isaac ROS ESS resize, disparity, frame-fix and depth/pointcloud actions.

Does not own:
- Camera image bridging or TF publication.
- Detector and pose-estimator backend construction.
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
    DISPARITY_TOPIC,
    LEFT_CAMERA_INFO_TOPIC,
    LEFT_IMAGE_TOPIC,
    LEFT_RESIZED_CAMERA_INFO_TOPIC,
    LEFT_RESIZED_IMAGE_TOPIC,
    RIGHT_CAMERA_INFO_TOPIC,
    RIGHT_IMAGE_TOPIC,
    RIGHT_RESIZED_CAMERA_INFO_TOPIC,
    RIGHT_RESIZED_IMAGE_TOPIC,
)
from franka_motion_runtime.bringup.launch_parts.camera.parameters import table_camera_parameters
from franka_motion_runtime.bringup.launch_parts.constants import PERCEPTION_PACKAGE


@dataclass(frozen=True)
class EssActions:
    """Grouped resize, ESS disparity and downstream cloud actions."""

    disparity_frame_fix: Node
    disparity_container: ComposableNodeContainer
    depth_pointcloud: Node


def create_sgbm_node(
    configs: dict[str, LaunchConfiguration],
    conditions: dict[str, IfCondition],
) -> Node:
    """Create the canonical local SGBM depth/cloud adapter."""
    return Node(
        package=PERCEPTION_PACKAGE,
        executable="stereo_sgbm_pointcloud_node.py",
        name="stereo_sgbm_pointcloud",
        output="screen",
        condition=conditions["sgbm"],
        parameters=table_camera_parameters(configs),
    )


def _create_ess_resize_nodes(
    configs: dict[str, LaunchConfiguration],
) -> tuple[ComposableNode, ComposableNode]:
    ess_resize_right_node = ComposableNode(
        package="isaac_ros_image_proc",
        plugin="nvidia::isaac_ros::image_proc::ResizeNode",
        name="image_resize_node_right",
        parameters=table_camera_parameters(configs),
        remappings=[
            ("camera_info", RIGHT_CAMERA_INFO_TOPIC),
            ("image", RIGHT_IMAGE_TOPIC),
            ("resize/camera_info", RIGHT_RESIZED_CAMERA_INFO_TOPIC),
            ("resize/image", RIGHT_RESIZED_IMAGE_TOPIC),
        ],
    )
    ess_resize_left_node = ComposableNode(
        package="isaac_ros_image_proc",
        plugin="nvidia::isaac_ros::image_proc::ResizeNode",
        name="image_resize_node_left",
        parameters=table_camera_parameters(configs),
        remappings=[
            ("camera_info", LEFT_CAMERA_INFO_TOPIC),
            ("image", LEFT_IMAGE_TOPIC),
            ("resize/camera_info", LEFT_RESIZED_CAMERA_INFO_TOPIC),
            ("resize/image", LEFT_RESIZED_IMAGE_TOPIC),
        ],
    )
    return ess_resize_left_node, ess_resize_right_node


def create_ess_actions(
    paths: Any,
    configs: dict[str, LaunchConfiguration],
    conditions: dict[str, IfCondition],
) -> EssActions:
    """Create the external ESS pipeline and canonical output adapters."""
    ess_resize_left_node, ess_resize_right_node = _create_ess_resize_nodes(configs)
    ess_disparity_node = ComposableNode(
        package="isaac_ros_ess",
        plugin="nvidia::isaac_ros::dnn_stereo_depth::ESSDisparityNode",
        name="ess_disparity",
        parameters=table_camera_parameters(configs),
        remappings=[
            ("left/image_rect", LEFT_RESIZED_IMAGE_TOPIC),
            ("left/camera_info", LEFT_RESIZED_CAMERA_INFO_TOPIC),
            ("right/image_rect", RIGHT_RESIZED_IMAGE_TOPIC),
            ("right/camera_info", RIGHT_RESIZED_CAMERA_INFO_TOPIC),
            ("disparity", DISPARITY_TOPIC),
        ],
    )
    ess_disparity_frame_fix_node = Node(
        package=PERCEPTION_PACKAGE,
        executable="disparity_frame_fix_node.py",
        name="ess_disparity_frame_fix",
        output="log",
        ros_arguments=["--log-level", "error"],
        condition=conditions["ess"],
        parameters=table_camera_parameters(configs),
    )
    ess_depth_pointcloud_node = Node(
        package=PERCEPTION_PACKAGE,
        executable="stereo_sgbm_pointcloud_node.py",
        name="ess_depth_pointcloud",
        output="screen",
        condition=conditions["ess"],
        parameters=table_camera_parameters(
            configs,
            {
                "use_detection_bbox_roi": ParameterValue(
                    PythonExpression(["'", configs["mode"], "'.lower() == 'yolo'"]),
                    value_type=bool,
                )
            },
        ),
    )
    ess_disparity_container = ComposableNodeContainer(
        name="ess_disparity_container",
        namespace="disparity",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ess_disparity_node,
            ess_resize_left_node,
            ess_resize_right_node,
        ],
        output="log",
        ros_arguments=["--log-level", "error"],
        condition=conditions["ess"],
        parameters=[{"use_sim_time": configs["use_sim_time"]}],
        prefix=f"{paths.managed_process_wrapper} --child-name ess_disparity_container --shutdown-signal kill --",
    )
    return EssActions(
        disparity_frame_fix=ess_disparity_frame_fix_node,
        disparity_container=ess_disparity_container,
        depth_pointcloud=ess_depth_pointcloud_node,
    )
