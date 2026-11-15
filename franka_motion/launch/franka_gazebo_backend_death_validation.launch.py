"""
Validation-only Gazebo graph with one independently restartable V2 workflow.

Responsibility:
- Reuse canonical table bringup without its embedded transfer workflow.
- Start the V2 composition root under an exact process-fault role.

Does not own:
- Process fault injection, robot goals, planning implementation, or evidence policy.
"""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from franka_motion_runtime.bringup.launch_parts.constants import SCENARIO_PACKAGE


_WORKFLOW_ROLE = "v2_transfer_workflow"


def generate_launch_description() -> LaunchDescription:
    """Create canonical table bringup with one faultable V2 workflow child."""
    scenario_share = Path(get_package_share_directory(SCENARIO_PACKAGE))
    scenario_prefix = Path(get_package_prefix(SCENARIO_PACKAGE))

    headless_argument = DeclareLaunchArgument(
        "headless",
        default_value="true",
        choices=["true", "false"],
        description="Run Gazebo without its graphical client.",
    )
    use_sim_time_argument = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        choices=["true"],
        description="This simulator-only validation always uses Gazebo clock.",
    )
    headless = LaunchConfiguration("headless")
    use_sim_time = LaunchConfiguration("use_sim_time")

    canonical_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(scenario_share / "launch" / "franka_gazebo_moveit_table.launch.py")
        ),
        launch_arguments={
            "headless": headless,
            "use_sim_time": use_sim_time,
            "start_transfer_server": "false",
        }.items(),
    )

    controller_executable = (
        scenario_prefix / "lib" / SCENARIO_PACKAGE /
        "franka_runtime_process_fault"
    )
    workflow_executable = (
        scenario_prefix / "lib" / SCENARIO_PACKAGE /
        "franka_transfer_sequence_v2_server"
    )
    faultable_workflow = ExecuteProcess(
        cmd=[
            str(controller_executable),
            "serve",
            "--role",
            _WORKFLOW_ROLE,
            "--",
            str(workflow_executable),
            "--ros-args",
            "--params-file",
            str(
                scenario_share / "config" / "manipulation" /
                "transfer_object_sequence_v2.yaml"
            ),
            "--params-file",
            str(
                scenario_share / "config" / "manipulation" /
                "moveit_backend.yaml"
            ),
            "--params-file",
            str(
                scenario_share / "config" / "manipulation" /
                "transfer_object_sequence_v2_deployment.yaml"
            ),
            "--params-file",
            str(
                scenario_share / "config" / "manipulation" /
                "robot_profile.yaml"
            ),
            "-p",
            ["use_sim_time:=", use_sim_time],
            "-p",
            "workflow.observation_provider_id:=configured",
            "-p",
            (
                "workflow.observation_provider_config_file:="
                "package://franka_motion/config/tasks/"
                "configured_observations_gazebo.yaml"
            ),
        ],
        name="faultable_v2_transfer_workflow",
        output="screen",
    )

    launch_actions = [
        headless_argument,
        use_sim_time_argument,
        canonical_bringup,
        faultable_workflow,
    ]
    return LaunchDescription(launch_actions)
