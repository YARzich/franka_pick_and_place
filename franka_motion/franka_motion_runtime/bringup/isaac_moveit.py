"""
Assemble standard empty/table Isaac MoveIt launch graphs.

Responsibility:
- Compose the shared Isaac/control stack with production MoveIt services.
- Add optional coordinated static-table geometry and workflow monitoring.

Does not own:
- Table-camera perception assembly or simulator backend implementation.
- MoveIt planning, scene-coordinator or workflow algorithms.
"""

from __future__ import annotations

from functools import partial

from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration

from franka_motion_runtime.bringup.launch_parts.arguments import (
    declare_common_isaac_arguments,
    declare_start_servo_argument,
)
from franka_motion_runtime.bringup.launch_parts.isaac_stack import (
    IsaacRuntimeSettings,
    create_isaac_control_stack,
)
from franka_motion_runtime.bringup.launch_parts.moveit import (
    OctomapProfile,
    create_isaac_move_group_node,
    create_isaac_moveit_config,
    create_moveit_servo_node,
    declare_planner_backend_argument,
    resolve_planner_backend,
)
from franka_motion_runtime.bringup.launch_parts.paths import create_isaac_launch_paths
from franka_motion_runtime.bringup.launch_parts.planning_scene import (
    TableCollisionProfile,
    create_scene_coordinator_node,
    create_table_planning_scene_node,
)
from franka_motion_runtime.bringup.launch_parts.workflow import (
    ISAAC_PHYSICAL_ATTACHMENT_SERVICE,
    create_servo_task_server_node,
    create_system_health_monitor_node,
    create_transfer_sequence_server_node,
    declare_start_transfer_server_argument,
)
from franka_motion_runtime.simulation.isaac.actions import create_isaac_process
from franka_motion_runtime.simulation.isaac.environment import create_isaac_environment_actions
from franka_motion_runtime.simulation.isaac.scenarios import IsaacMoveItScenario


def _launch_setup(
    context: LaunchContext,
    *,
    scenario: IsaacMoveItScenario,
    octomap_profile: OctomapProfile,
    table_collision_profile: TableCollisionProfile | None,
) -> list[object]:
    """Resolve substitutions and assemble one standard Isaac MoveIt graph."""
    planner_backend = resolve_planner_backend(
        context,
        octomap_profile=octomap_profile,
    )
    paths = create_isaac_launch_paths()
    settings = IsaacRuntimeSettings.from_context(
        context,
        include_isaac_experience=scenario.include_isaac_experience_arg,
    )
    moveit_config = create_isaac_moveit_config(
        paths,
        octomap_profile=octomap_profile,
        planner_backend=planner_backend,
    )
    isaac_process = create_isaac_process(
        paths=paths,
        scenario=scenario,
        isaac_python=settings.isaac_python,
        isaac_prim=settings.isaac_robot_prim,
        headless=settings.headless,
        fix_base=settings.fix_base,
        time_steps=settings.time_steps,
        isaac_experience=settings.isaac_experience,
        rmw_implementation=settings.rmw_implementation,
    )
    control_stack = create_isaac_control_stack(
        paths=paths,
        settings=settings,
        robot_description=moveit_config.robot_description,
    )
    move_group_node = create_isaac_move_group_node(
        paths=paths,
        use_sim_time=settings.use_sim_time,
        moveit_config=moveit_config,
    )
    servo_node = create_moveit_servo_node(
        use_sim_time=settings.use_sim_time,
        start_servo=LaunchConfiguration("start_servo"),
        moveit_config=moveit_config,
        servo_config_path=(
            paths.package_share / "config" / "moveit" / "servo.yaml"
        ),
    )
    planning_scene_node = None
    if table_collision_profile is not None:
        planning_scene_node = create_table_planning_scene_node(
            use_sim_time=settings.use_sim_time,
            profile=table_collision_profile,
        )
    scene_coordinator_node = create_scene_coordinator_node(
        use_sim_time=settings.use_sim_time,
    )
    health_monitor_node = create_system_health_monitor_node(
        use_sim_time=settings.use_sim_time,
    )
    servo_task_server_node = create_servo_task_server_node(
        use_sim_time=settings.use_sim_time,
    )
    transfer_sequence_server_node = create_transfer_sequence_server_node(
        use_sim_time=settings.use_sim_time,
        configured_observation_preset="configured_observations_isaac.yaml",
        planner_backend=planner_backend,
        physical_attachment_service=ISAAC_PHYSICAL_ATTACHMENT_SERVICE,
    )
    launch_actions = control_stack.create_motion_start_events(
        isaac_process=isaac_process,
        motion_backend_node=move_group_node,
        motion_companion_nodes=[servo_node, servo_task_server_node],
        additional_nodes=[
            scene_coordinator_node,
            health_monitor_node,
            transfer_sequence_server_node,
        ],
        planning_scene_node=planning_scene_node,
    )
    return launch_actions


def create_isaac_moveit_description(
    scenario: IsaacMoveItScenario,
    *,
    octomap_profile: OctomapProfile,
    start_transfer_server_default: bool,
    table_collision_profile: TableCollisionProfile | None = None,
) -> LaunchDescription:
    """Create one standard Isaac MoveIt leaf from immutable scenario policy."""
    launch_arguments = declare_common_isaac_arguments(
        include_isaac_experience=scenario.include_isaac_experience_arg,
    )
    transfer_server_argument = declare_start_transfer_server_argument(
        default_value=start_transfer_server_default,
    )
    start_servo_argument = declare_start_servo_argument()
    planner_backend_argument = declare_planner_backend_argument()
    environment_actions = create_isaac_environment_actions()
    launch_setup_action = OpaqueFunction(
        function=partial(
            _launch_setup,
            scenario=scenario,
            octomap_profile=octomap_profile,
            table_collision_profile=table_collision_profile,
        )
    )
    launch_actions = [
        *launch_arguments,
        transfer_server_argument,
        start_servo_argument,
        planner_backend_argument,
        *environment_actions,
        launch_setup_action,
    ]
    launch_description = LaunchDescription(launch_actions)
    return launch_description
