"""
Assemble standard empty/table Gazebo MoveIt launch graphs.

Responsibility:
- Compose the shared Gazebo/control stack with one selected MoveIt pipeline.
- Add optional coordinated static-table geometry and workflow monitoring.

Does not own:
- Table-camera perception assembly or simulator backend implementation.
- Planner, scene-coordinator or workflow algorithms.
"""

from __future__ import annotations

from functools import partial

from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration

from franka_motion_runtime.bringup.launch_parts.arguments import (
    declare_headless_argument,
    declare_start_servo_argument,
    declare_use_sim_time_argument,
)
from franka_motion_runtime.bringup.launch_parts.gazebo_stack import (
    create_gazebo_control_stack,
    prepare_gazebo_robot_model,
)
from franka_motion_runtime.bringup.launch_parts.moveit import (
    OctomapProfile,
    create_gazebo_move_group_node,
    create_gazebo_moveit_config,
    create_moveit_servo_node,
    declare_planner_backend_argument,
    resolve_planner_backend,
)
from franka_motion_runtime.bringup.launch_parts.planning_scene import (
    TableCollisionProfile,
    create_scene_coordinator_node,
    create_table_planning_scene_node,
)
from franka_motion_runtime.bringup.launch_parts.workflow import (
    create_servo_task_server_node,
    create_system_health_monitor_node,
    create_transfer_sequence_server_node,
    declare_start_transfer_server_argument,
)
from franka_motion_runtime.simulation.gazebo.bridge import (
    create_clock_bridge_node,
    write_clock_bridge_config,
)
from franka_motion_runtime.simulation.gazebo.environment import (
    create_gazebo_plugin_environment,
)
from franka_motion_runtime.simulation.gazebo.scenarios import GazeboScenario
from franka_motion_runtime.simulation.gazebo.world import gazebo_clock_topic


def _launch_setup(
    context: LaunchContext,
    *,
    scenario: GazeboScenario,
    octomap_profile: OctomapProfile,
    table_collision_profile: TableCollisionProfile | None,
) -> list[object]:
    """Validate the selected pipeline before creating files or processes."""
    planner_backend = resolve_planner_backend(
        context,
        octomap_profile=octomap_profile,
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    headless = LaunchConfiguration("headless")
    start_servo = LaunchConfiguration("start_servo")

    robot_model = prepare_gazebo_robot_model(scenario)
    paths = robot_model.paths
    moveit_config = create_gazebo_moveit_config(
        paths,
        robot_model.description_xml,
        octomap_profile=octomap_profile,
        planner_backend=planner_backend,
    )
    write_clock_bridge_config(
        paths.tmp_bridge_config,
        gazebo_clock_topic(paths.world),
    )

    ign_plugin_environment, gz_plugin_environment = (
        create_gazebo_plugin_environment()
    )
    clock_bridge = create_clock_bridge_node(paths)
    control_stack = create_gazebo_control_stack(
        robot_model,
        use_sim_time=use_sim_time,
        headless=headless,
        bridge=clock_bridge,
    )
    move_group_node = create_gazebo_move_group_node(
        use_sim_time=use_sim_time,
        moveit_config=moveit_config,
        managed_process_wrapper_path=paths.managed_process_wrapper,
    )
    servo_node = create_moveit_servo_node(
        use_sim_time=use_sim_time,
        start_servo=start_servo,
        moveit_config=moveit_config,
        servo_config_path=(
            paths.package_share / "config" / "moveit" / "servo.yaml"
        ),
    )
    planning_scene_node = None
    if table_collision_profile is not None:
        planning_scene_node = create_table_planning_scene_node(
            use_sim_time=use_sim_time,
            profile=table_collision_profile,
        )
    scene_coordinator_node = create_scene_coordinator_node(
        use_sim_time=use_sim_time,
    )
    health_monitor_node = create_system_health_monitor_node(
        use_sim_time=use_sim_time,
    )
    servo_task_server_node = create_servo_task_server_node(
        use_sim_time=use_sim_time,
    )
    transfer_sequence_server_node = create_transfer_sequence_server_node(
        use_sim_time=use_sim_time,
        planner_backend=planner_backend,
    )
    motion_start_events = control_stack.create_motion_start_events(
        motion_backend_node=move_group_node,
        motion_companion_nodes=[servo_node, servo_task_server_node],
        planning_scene_node=planning_scene_node,
    )

    return [
        ign_plugin_environment,
        gz_plugin_environment,
        *control_stack.simulator_exit_events,
        *motion_start_events,
        control_stack.headless_simulator,
        control_stack.gui_simulator,
        control_stack.bridge,
        control_stack.robot_state_publisher,
        scene_coordinator_node,
        health_monitor_node,
        transfer_sequence_server_node,
        control_stack.spawn_robot,
    ]


def create_gazebo_moveit_description(
    scenario: GazeboScenario,
    *,
    octomap_profile: OctomapProfile,
    start_transfer_server_default: bool,
    table_collision_profile: TableCollisionProfile | None = None,
) -> LaunchDescription:
    """Create one Gazebo leaf with a launch-selected MoveIt pipeline."""
    use_sim_time_argument = declare_use_sim_time_argument(
        "Использовать /clock от Gazebo"
    )
    headless_argument = declare_headless_argument(
        "Запуск Gazebo без GUI (gz sim -s)"
    )
    transfer_server_argument = declare_start_transfer_server_argument(
        default_value=start_transfer_server_default,
    )
    start_servo_argument = declare_start_servo_argument()
    planner_backend_argument = declare_planner_backend_argument()
    launch_setup_action = OpaqueFunction(
        function=partial(
            _launch_setup,
            scenario=scenario,
            octomap_profile=octomap_profile,
            table_collision_profile=table_collision_profile,
        )
    )
    launch_actions = [
        use_sim_time_argument,
        headless_argument,
        transfer_server_argument,
        start_servo_argument,
        planner_backend_argument,
        launch_setup_action,
    ]
    launch_description = LaunchDescription(launch_actions)
    return launch_description
