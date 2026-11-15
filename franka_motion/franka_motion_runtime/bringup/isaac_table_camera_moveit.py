"""
Assemble the canonical Isaac table-camera MoveIt launch graph.

Responsibility:
- Compose simulator, control, MoveIt, perception and health launch actions.
Does not own:
- Simulator, perception, controller or motion-planning implementation.
"""

from __future__ import annotations

from functools import partial

from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from franka_motion_runtime.bringup.launch_parts.arguments import (
    declare_isaac_runtime_arguments,
    declare_start_servo_argument,
)
from franka_motion_runtime.bringup.launch_parts.camera.arguments import (
    create_isaac_table_camera_launch_arguments,
)
from franka_motion_runtime.bringup.launch_parts.camera.modes import create_pipeline_conditions
from franka_motion_runtime.bringup.launch_parts.camera.paths import (
    TableCameraPaths,
    create_table_camera_paths,
)
from franka_motion_runtime.bringup.launch_parts.camera.perception import create_perception_actions
from franka_motion_runtime.bringup.launch_parts.camera.pipeline import (
    create_camera_pipeline_actions,
)
from franka_motion_runtime.bringup.launch_parts.camera.rviz import create_rviz_actions
from franka_motion_runtime.bringup.launch_parts.camera.tf import create_camera_tf_nodes
from franka_motion_runtime.bringup.launch_parts.camera.validation import (
    create_table_camera_validation_actions,
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
    create_octomap_tf_gate_node,
    declare_planner_backend_argument,
    resolve_planner_backend,
)
from franka_motion_runtime.bringup.launch_parts.paths import (
    IsaacLaunchPaths,
    create_isaac_launch_paths,
)
from franka_motion_runtime.bringup.launch_parts.planning_scene import (
    ISAAC_TABLE_COLLISION,
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
from franka_motion_runtime.simulation.isaac.environment import create_isaac_environment_actions
from franka_motion_runtime.simulation.isaac.scenarios import (
    ISAAC_TABLE_SCENARIO,
    IsaacMoveItScenario,
)
from franka_motion_runtime.simulation.isaac.table_camera import (
    IsaacStereoRigSpec,
    create_isaac_table_camera_process,
    load_gazebo_camera_profile,
)


def _launch_setup(
    context: LaunchContext,
    *args,
    paths: IsaacLaunchPaths,
    camera_paths: TableCameraPaths,
    configs: dict[str, LaunchConfiguration],
    conditions: dict[str, IfCondition],
    scenario: IsaacMoveItScenario,
    moveit_config,
    **kwargs,
) -> list[object]:
    """Resolve substitutions and assemble the table-camera Isaac graph."""
    resolve_planner_backend(
        context,
        octomap_profile=OctomapProfile.TABLE_CAMERA,
    )
    settings = IsaacRuntimeSettings.from_context(
        context,
        include_isaac_experience=True,
    )
    camera_profile = load_gazebo_camera_profile(camera_paths.camera_sdf)
    stereo_rig = IsaacStereoRigSpec.above_cube(scenario.cube_position)
    isaac_process = create_isaac_table_camera_process(
        paths=paths,
        scenario=scenario,
        camera_profile=camera_profile,
        stereo_rig=stereo_rig,
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
    octomap_tf_gate_node = create_octomap_tf_gate_node(
        use_sim_time=settings.use_sim_time,
    )
    planning_scene_node = create_table_planning_scene_node(
        use_sim_time=settings.use_sim_time,
        profile=ISAAC_TABLE_COLLISION,
    )
    scene_coordinator_node = create_scene_coordinator_node(
        use_sim_time=settings.use_sim_time
    )
    health_monitor_node = create_system_health_monitor_node(
        use_sim_time=settings.use_sim_time,
        perception_config=configs["table_camera_params_file"],
    )
    servo_task_server_node = create_servo_task_server_node(
        use_sim_time=settings.use_sim_time,
    )
    transfer_sequence_server_node = create_transfer_sequence_server_node(
        use_sim_time=settings.use_sim_time,
        detected_observation=True,
        physical_attachment_service=ISAAC_PHYSICAL_ATTACHMENT_SERVICE,
    )

    perception_actions = create_perception_actions(paths, configs, conditions)
    left_world_pose, right_world_pose = stereo_rig.world_poses()
    camera_tf_nodes = create_camera_tf_nodes(
        settings.use_sim_time,
        left_world_pose=left_world_pose,
        right_world_pose=right_world_pose,
    )
    rviz_actions = create_rviz_actions(configs, moveit_config)
    camera_pipeline_actions = create_camera_pipeline_actions(
        perception_actions,
        camera_tf_nodes,
        rviz_actions,
        include_image_bridges=False,
    )

    additional_nodes = [
        scene_coordinator_node,
        health_monitor_node,
        transfer_sequence_server_node,
        octomap_tf_gate_node,
        *camera_pipeline_actions,
    ]
    launch_actions = control_stack.create_motion_start_events(
        isaac_process=isaac_process,
        motion_backend_node=move_group_node,
        motion_companion_nodes=[servo_node, servo_task_server_node],
        additional_nodes=additional_nodes,
        planning_scene_node=planning_scene_node,
    )
    return launch_actions


def create_isaac_table_camera_moveit_description() -> LaunchDescription:
    """Create the canonical Isaac table-camera MoveIt stack."""
    scenario = ISAAC_TABLE_SCENARIO
    paths = create_isaac_launch_paths()
    camera_paths = create_table_camera_paths(paths.package_share)
    moveit_config = create_isaac_moveit_config(
        paths,
        octomap_profile=OctomapProfile.TABLE_CAMERA,
    )
    configs, camera_arguments = create_isaac_table_camera_launch_arguments(camera_paths)
    conditions = create_pipeline_conditions(configs)
    runtime_arguments = declare_isaac_runtime_arguments(
        include_isaac_experience=True,
        include_gxf_log_level=True,
    )
    start_transfer_server_argument = declare_start_transfer_server_argument(default_value=True)
    start_servo_argument = declare_start_servo_argument()
    planner_backend_argument = declare_planner_backend_argument()
    validation_actions = create_table_camera_validation_actions()
    environment_actions = create_isaac_environment_actions(include_gxf_log_level=True)
    launch_setup_action = OpaqueFunction(
        function=partial(
            _launch_setup,
            paths=paths,
            camera_paths=camera_paths,
            configs=configs,
            conditions=conditions,
            scenario=scenario,
            moveit_config=moveit_config,
        )
    )

    launch_actions = [
        *camera_arguments,
        *runtime_arguments,
        start_transfer_server_argument,
        start_servo_argument,
        planner_backend_argument,
        *validation_actions,
        *environment_actions,
        launch_setup_action,
    ]
    launch_description = LaunchDescription(launch_actions)
    return launch_description
