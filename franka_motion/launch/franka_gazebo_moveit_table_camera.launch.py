"""
Gazebo table-camera MoveIt and perception launch entrypoint.

Responsibility:
- Start the Gazebo table-camera scene, robot, controllers, gripper and move_group.
- Configure table stereo camera bridges, TF, perception backends and optional RViz.
- Wire perception observations into the resident V2 MTC grasp provider.

Does not own:
- Isaac Sim startup.
- cuRobo planner startup.
- Backend inference implementation details.
"""

from __future__ import annotations

from pathlib import Path

from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration

from franka_motion_runtime.bringup.launch_parts.arguments import (
    declare_start_servo_argument,
)

from franka_motion_runtime.bringup.launch_parts.camera.arguments import (
    create_gazebo_table_camera_launch_arguments,
)
from franka_motion_runtime.bringup.launch_parts.camera.constants import TABLE_BASE_POSE
from franka_motion_runtime.bringup.launch_parts.camera.modes import create_pipeline_conditions
from franka_motion_runtime.bringup.launch_parts.camera.paths import create_table_camera_paths
from franka_motion_runtime.bringup.launch_parts.camera.perception import create_perception_actions
from franka_motion_runtime.bringup.launch_parts.camera.pipeline import (
    create_camera_pipeline_actions,
)
from franka_motion_runtime.bringup.launch_parts.camera.rviz import create_rviz_actions
from franka_motion_runtime.bringup.launch_parts.camera.tf import create_camera_tf_nodes
from franka_motion_runtime.bringup.launch_parts.camera.validation import (
    create_table_camera_validation_actions,
)
from franka_motion_runtime.bringup.launch_parts.gazebo_stack import (
    create_gazebo_control_stack,
    prepare_gazebo_robot_model,
)
from franka_motion_runtime.bringup.launch_parts.gazebo_table_camera import (
    GazeboTableCameraActions,
    GazeboTableCameraMotionActions,
    create_gazebo_table_camera_start_events,
)
from franka_motion_runtime.bringup.launch_parts.moveit import (
    OctomapProfile,
    create_gazebo_move_group_node,
    create_gazebo_moveit_config,
    create_moveit_servo_node,
    create_octomap_tf_gate_node,
    declare_planner_backend_argument,
    resolve_planner_backend,
)
from franka_motion_runtime.bringup.launch_parts.planning_scene import (
    GAZEBO_TABLE_COLLISION,
    create_scene_coordinator_node,
    create_table_planning_scene_node,
)
from franka_motion_runtime.bringup.launch_parts.workflow import (
    create_servo_task_server_node,
    create_system_health_monitor_node,
    create_transfer_sequence_server_node,
    declare_start_transfer_server_argument,
)
from franka_motion_runtime.simulation.gazebo.environment import create_gazebo_camera_environment
from franka_motion_runtime.simulation.gazebo.scenarios import GazeboScenario
from franka_motion_runtime.simulation.gazebo.table_camera import (
    create_spawn_camera_node,
    create_table_camera_bridge_node,
    write_table_camera_bridge_config,
)
from franka_motion_runtime.simulation.gazebo.world import gazebo_clock_topic


def _validate_planner_backend(context: LaunchContext) -> list[object]:
    """Reject cuRobo before starting a scene containing an Octomap."""
    resolve_planner_backend(
        context,
        octomap_profile=OctomapProfile.TABLE_CAMERA,
    )
    return []


def generate_launch_description() -> LaunchDescription:
    """Create the canonical Gazebo table-camera MoveIt stack."""
    scenario = GazeboScenario(
        world_file="table_world_camera.sdf",
        tmp_urdf_path=Path("/tmp/franka_panda_table_camera.urdf"),
        tmp_bridge_config_path=Path("/tmp/franka_gazebo_table_camera_bridge.yaml"),
        base_pose=TABLE_BASE_POSE,
    )
    robot_model = prepare_gazebo_robot_model(scenario)
    paths = robot_model.paths
    camera_paths = create_table_camera_paths(paths.package_share)
    configs, launch_arguments = create_gazebo_table_camera_launch_arguments(camera_paths)
    start_transfer_server_arg = declare_start_transfer_server_argument(
        default_value=True,
    )
    start_servo_arg = declare_start_servo_argument()
    planner_backend_arg = declare_planner_backend_argument()
    planner_backend_validation = OpaqueFunction(
        function=_validate_planner_backend
    )
    start_servo = LaunchConfiguration("start_servo")
    conditions = create_pipeline_conditions(configs)

    moveit_config = create_gazebo_moveit_config(
        paths,
        robot_model.description_xml,
        octomap_profile=OctomapProfile.TABLE_CAMERA,
    )
    write_table_camera_bridge_config(paths.tmp_bridge_config, gazebo_clock_topic(paths.world))

    validation_actions = create_table_camera_validation_actions()
    environment_actions = create_gazebo_camera_environment(configs["fastdds_builtin_transports"])
    bridge_node = create_table_camera_bridge_node(paths.bridge_wrapper, paths.tmp_bridge_config)
    control_stack = create_gazebo_control_stack(
        robot_model,
        use_sim_time=configs["use_sim_time"],
        headless=configs["headless"],
        bridge=bridge_node,
        use_extended_timeouts=True,
        robot_state_publisher_ros_arguments=[
            "--log-level",
            "error",
            "--log-level",
            "tf2_buffer:=error",
        ],
    )
    spawn_camera_node = create_spawn_camera_node(camera_paths.camera_sdf)
    gazebo_actions = GazeboTableCameraActions(
        headless_simulator=control_stack.headless_simulator,
        gui_simulator=control_stack.gui_simulator,
        bridge=control_stack.bridge,
        robot_state_publisher=control_stack.robot_state_publisher,
        spawn_robot=control_stack.spawn_robot,
        spawn_camera=spawn_camera_node,
    )

    move_group_node = create_gazebo_move_group_node(
        use_sim_time=configs["use_sim_time"],
        moveit_config=moveit_config,
        managed_process_wrapper_path=paths.managed_process_wrapper,
        output="log",
        ros_arguments=["--log-level", "error", "--log-level", "tf2_buffer:=error"],
    )
    servo_node = create_moveit_servo_node(
        use_sim_time=configs["use_sim_time"],
        start_servo=start_servo,
        moveit_config=moveit_config,
        servo_config_path=(
            paths.package_share / "config" / "moveit" / "servo.yaml"
        ),
    )
    octomap_tf_gate_node = create_octomap_tf_gate_node(
        use_sim_time=configs["use_sim_time"],
    )
    servo_task_server_node = create_servo_task_server_node(
        use_sim_time=configs["use_sim_time"],
    )
    move_group_start_delayed = TimerAction(
        period=2.0,
        actions=[move_group_node, servo_node, servo_task_server_node],
    )
    planning_scene_node = create_table_planning_scene_node(
        use_sim_time=configs["use_sim_time"],
        profile=GAZEBO_TABLE_COLLISION,
    )
    scene_coordinator_node = create_scene_coordinator_node(
        use_sim_time=configs["use_sim_time"],
    )
    health_monitor_node = create_system_health_monitor_node(
        use_sim_time=configs["use_sim_time"],
        perception_config=configs["table_camera_params_file"],
    )
    transfer_sequence_server_node = create_transfer_sequence_server_node(
        use_sim_time=configs["use_sim_time"],
        detected_observation=True,
    )
    motion_actions = GazeboTableCameraMotionActions(
        joint_state_broadcaster_spawner=(
            control_stack.joint_state_broadcaster_spawner
        ),
        arm_controller_spawner=control_stack.arm_controller_spawner,
        hand_controller_spawner=control_stack.hand_controller_spawner,
        gripper_driver=control_stack.gripper_driver,
        delayed_move_group_start=move_group_start_delayed,
        planning_scene=planning_scene_node,
    )

    perception_actions = create_perception_actions(paths, configs, conditions)
    camera_tf_nodes = create_camera_tf_nodes(configs["use_sim_time"])
    rviz_actions = create_rviz_actions(configs, moveit_config)
    camera_pipeline_actions = create_camera_pipeline_actions(
        perception_actions,
        camera_tf_nodes,
        rviz_actions,
        include_image_bridges=True,
    )
    start_events = create_gazebo_table_camera_start_events(
        gazebo_actions,
        motion_actions,
        camera_pipeline_actions,
    )

    launch_actions = [
        *launch_arguments,
        start_transfer_server_arg,
        start_servo_arg,
        planner_backend_arg,
        planner_backend_validation,
        *validation_actions,
        *environment_actions,
        *control_stack.simulator_exit_events,
        *start_events,
        gazebo_actions.headless_simulator,
        gazebo_actions.gui_simulator,
        gazebo_actions.bridge,
        gazebo_actions.robot_state_publisher,
        scene_coordinator_node,
        health_monitor_node,
        transfer_sequence_server_node,
        octomap_tf_gate_node,
        gazebo_actions.spawn_robot,
    ]
    launch_description = LaunchDescription(launch_actions)
    return launch_description
