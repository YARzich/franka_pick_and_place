"""
Launch factories for production workflow admission and execution.

Responsibility:
- Start the typed system-health observer for MoveIt leaves.
- Conditionally expose the sole TransferObjectSequenceV2 workflow.

Does not own:
- Health aggregation, workflow state-machine or MoveIt backend logic.
- Simulator/controller startup ordering.
"""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from franka_motion_runtime.bringup.contracts import PlannerBackend
from franka_motion_runtime.bringup.launch_parts.constants import (
    MANIPULATION_PACKAGE,
    MOVEIT_BACKEND_PACKAGE,
    SCENARIO_PACKAGE,
)
from franka_motion_runtime.bringup.launch_parts.moveit import (
    planning_pipeline_name,
)

ISAAC_PHYSICAL_ATTACHMENT_SERVICE = (
    "/franka_motion/isaac/set_pick_cube_attached"
)


def declare_start_transfer_server_argument(
    *,
    default_value: bool,
) -> DeclareLaunchArgument:
    """Declare the additive workflow-server switch for one public leaf."""
    argument = DeclareLaunchArgument(
        "start_transfer_server",
        default_value=str(default_value).lower(),
        choices=["true", "false"],
        description=(
            "Запускать production TransferObjectSequenceV2 workflow."
        ),
    )
    return argument


def create_system_health_monitor_node(
    *,
    use_sim_time: LaunchConfiguration | bool,
    perception_config: LaunchConfiguration | None = None,
) -> Node:
    """Create the read-only typed health monitor used for motion admission."""
    monitoring_share = Path(
        get_package_share_directory(MANIPULATION_PACKAGE)
    )
    scenario_share = Path(get_package_share_directory(SCENARIO_PACKAGE))
    parameter_files: list[object] = [
        str(monitoring_share / "config" / "system_health.yaml"),
        str(
            scenario_share / "config" / "manipulation" /
            "system_health.yaml"
        ),
    ]
    if perception_config is not None:
        parameter_files.append(perception_config)
    parameter_files.append(
        str(
            scenario_share / "config" / "manipulation" /
            "robot_profile.yaml"
        )
    )
    monitor_node = Node(
        package=MANIPULATION_PACKAGE,
        executable="system_health_monitor",
        name="system_health_monitor",
        output="screen",
        parameters=[*parameter_files, {"use_sim_time": use_sim_time}],
    )
    return monitor_node


def create_servo_task_server_node(
    *,
    use_sim_time: LaunchConfiguration | bool,
) -> Node:
    """Create the standalone ServoToPose host when reactive control is enabled."""
    scenario_share = Path(get_package_share_directory(SCENARIO_PACKAGE))
    servo_task_server_node = Node(
        package=MOVEIT_BACKEND_PACKAGE,
        executable="moveit_servo_task_server",
        name="franka_moveit_servo_task_server",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_servo")),
        parameters=[
            str(
                scenario_share / "config" / "manipulation" /
                "servo_to_pose.yaml"
            ),
            str(
                scenario_share / "config" / "manipulation" /
                "robot_profile.yaml"
            ),
            {"use_sim_time": use_sim_time},
        ],
    )
    return servo_task_server_node


def create_transfer_sequence_server_node(
    *,
    use_sim_time: LaunchConfiguration | bool,
    detected_observation: bool = False,
    configured_observation_preset: str = "configured_observations_gazebo.yaml",
    planner_backend: PlannerBackend = PlannerBackend.MOVEIT,
    physical_attachment_service: str = "",
    parameter_overrides: dict[str, object] | None = None,
    conditioned_on_start_argument: bool = True,
) -> Node:
    """Create the sole V2 workflow with one scenario-selected observation port."""
    scenario_share = Path(get_package_share_directory(SCENARIO_PACKAGE))
    provider_id = "detected" if detected_observation else "configured"
    provider_file = (
        "detected_observation.yaml"
        if detected_observation
        else configured_observation_preset
    )
    runtime_parameters: dict[str, object] = {
        "workflow.observation_provider_id": provider_id,
        "workflow.observation_provider_config_file": (
            f"package://franka_motion/config/tasks/{provider_file}"
        ),
        "moveit.planning_pipeline": planning_pipeline_name(planner_backend),
        "endpoints.physical_attachment_service": physical_attachment_service,
    }
    runtime_parameters.update(parameter_overrides or {})
    runtime_parameters["use_sim_time"] = use_sim_time
    parameters_with_use_sim_time: list[object] = [
        str(
            scenario_share / "config" / "manipulation" /
            "transfer_object_sequence_v2.yaml"
        ),
        str(
            scenario_share / "config" / "manipulation" /
            "moveit_backend.yaml"
        ),
        str(
            scenario_share / "config" / "manipulation" /
            "transfer_object_sequence_v2_deployment.yaml"
        ),
        str(
            scenario_share / "config" / "manipulation" /
            "robot_profile.yaml"
        ),
        runtime_parameters,
    ]
    condition = (
        IfCondition(LaunchConfiguration("start_transfer_server"))
        if conditioned_on_start_argument
        else None
    )
    sequence_server_node = Node(
        package=SCENARIO_PACKAGE,
        executable="franka_transfer_sequence_v2_server",
        output="screen",
        condition=condition,
        parameters=parameters_with_use_sim_time,
    )
    return sequence_server_node
