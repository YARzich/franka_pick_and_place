"""
YOLO visualizer and detection-mask node factory for table-camera modes.

Responsibility:
- Create the YOLO detector node with mode-dependent image topic and mask settings.
- Keep launch-time detector remapping aligned with FoundationPose/MegaPose inputs.

Does not own:
- YOLO model inference or bbox/mask post-processing.
- Stereo depth and pose-estimator node construction.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from franka_motion_runtime.bringup.launch_parts.camera.constants import (
    FOUNDATIONPOSE_IMAGE_TOPIC,
    LEFT_RESIZED_IMAGE_TOPIC,
)
from franka_motion_runtime.bringup.launch_parts.camera.parameters import table_camera_parameters
from franka_motion_runtime.bringup.launch_parts.constants import PERCEPTION_YOLO_PACKAGE


def yolo_shutdown_prefix(
    managed_process_wrapper: Path,
    yolo_python: LaunchConfiguration,
) -> list[Any]:
    """Build the bounded external-Python YOLO process prefix."""
    return [
        TextSubstitution(
            text=(
                f"{managed_process_wrapper} "
                "--child-name yolo_visualizer "
                "--shutdown-signal int "
                "--shutdown-timeout-sec 0.8 "
                "--kill-timeout-sec 1.0 -- "
            )
        ),
        yolo_python,
    ]


def create_yolo_node(
    managed_process_wrapper: Path,
    configs: dict[str, LaunchConfiguration],
    conditions: dict[str, IfCondition],
) -> Node:
    """Create the conditional YOLO ROS adapter with stable topics."""
    return Node(
        package=PERCEPTION_YOLO_PACKAGE,
        executable="yolo_visualizer_node.py",
        name="yolo_visualizer",
        output="log",
        ros_arguments=["--log-level", "error"],
        condition=conditions["yolo"],
        prefix=yolo_shutdown_prefix(managed_process_wrapper, configs["yolo_python"]),
        parameters=table_camera_parameters(
            configs,
            {
                "image_topic": ParameterValue(
                    PythonExpression(
                        [
                            "'",
                            FOUNDATIONPOSE_IMAGE_TOPIC,
                            "' if '",
                            configs["mode"],
                            "'.lower() in ('foundationpose', 'megapose', 'auto', 'gigapose') "
                            "else '",
                            LEFT_RESIZED_IMAGE_TOPIC,
                            "'",
                        ]
                    ),
                    value_type=str,
                )
            },
            {
                "class_whitelist": ParameterValue(
                    PythonExpression(
                        [
                            "'' if '",
                            configs["mode"],
                            "'.lower() in ('foundationpose', 'megapose', 'auto', 'gigapose') else '",
                            configs["yolo_class_whitelist"],
                            "'",
                        ]
                    ),
                    value_type=str,
                )
            },
            {
                "publish_detection_mask": ParameterValue(
                    PythonExpression(
                        [
                            "'",
                            configs["mode"],
                            "'.lower() == 'foundationpose' or '",
                            configs["mode"],
                            "'.lower() == 'gigapose'",
                        ]
                    ),
                    value_type=bool,
                )
            },
            {
                "enable_window": ParameterValue(
                    PythonExpression(["'", configs["mode"], "'.lower() == 'yolo'"]),
                    value_type=bool,
                )
            },
            {
                "enable_runtime_controls": ParameterValue(
                    PythonExpression(["'", configs["mode"], "'.lower() == 'yolo'"]),
                    value_type=bool,
                )
            },
        ),
    )
