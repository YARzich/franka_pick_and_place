"""
Sequence the Gazebo table-camera integration graph.

Responsibility:
- Group already-created simulator and motion actions into typed inputs.
- Release controllers, camera, MoveIt and perception after exact prerequisites.

Does not own:
- Creation of simulator, controller, motion or perception nodes.
- Runtime health and recovery after startup completes.
"""

from __future__ import annotations

from dataclasses import dataclass

from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch_ros.actions import Node

from franka_motion_runtime.bringup.launch_parts.events import (
    create_start_event_after_successful_exit,
)


@dataclass(frozen=True)
class GazeboTableCameraActions:
    """Simulator actions that establish the robot and stereo camera."""

    headless_simulator: ExecuteProcess
    gui_simulator: ExecuteProcess
    bridge: ExecuteProcess
    robot_state_publisher: Node
    spawn_robot: Node
    spawn_camera: Node


@dataclass(frozen=True)
class GazeboTableCameraMotionActions:
    """Motion actions released by the controller startup chain."""

    joint_state_broadcaster_spawner: Node
    arm_controller_spawner: Node
    hand_controller_spawner: Node
    gripper_driver: Node
    delayed_move_group_start: TimerAction
    planning_scene: Node


def create_gazebo_table_camera_start_events(
    gazebo_actions: GazeboTableCameraActions,
    motion_actions: GazeboTableCameraMotionActions,
    camera_pipeline_actions: list[object],
) -> list[RegisterEventHandler]:
    """Create fail-closed startup edges for one table-camera scenario."""
    joint_state_start_event = create_start_event_after_successful_exit(
        target_action=gazebo_actions.spawn_robot,
        actions=[
            motion_actions.joint_state_broadcaster_spawner,
            gazebo_actions.spawn_camera,
        ],
        prerequisite_name="Gazebo robot spawn",
    )
    panda_arm_start_event = create_start_event_after_successful_exit(
        target_action=motion_actions.joint_state_broadcaster_spawner,
        actions=[motion_actions.arm_controller_spawner],
        prerequisite_name="joint_state_broadcaster spawner",
    )
    panda_hand_start_event = create_start_event_after_successful_exit(
        target_action=motion_actions.arm_controller_spawner,
        actions=[motion_actions.hand_controller_spawner],
        prerequisite_name="panda_arm_controller spawner",
    )
    panda_gripper_start_event = create_start_event_after_successful_exit(
        target_action=motion_actions.hand_controller_spawner,
        actions=[
            motion_actions.gripper_driver,
            motion_actions.delayed_move_group_start,
            motion_actions.planning_scene,
        ],
        prerequisite_name="panda_hand_controller spawner",
    )
    camera_pipeline_start_event = create_start_event_after_successful_exit(
        target_action=gazebo_actions.spawn_camera,
        actions=camera_pipeline_actions,
        prerequisite_name="Gazebo table camera spawn",
    )
    return [
        joint_state_start_event,
        panda_arm_start_event,
        panda_hand_start_event,
        panda_gripper_start_event,
        camera_pipeline_start_event,
    ]
