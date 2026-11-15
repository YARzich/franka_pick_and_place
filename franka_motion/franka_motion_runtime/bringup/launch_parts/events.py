"""
Launch event chains for startup ordering and shutdown-sensitive sequencing.

Responsibility:
- Wire Gazebo and Isaac actions into deterministic controller/planner startup order.
- Delay dependent nodes until the simulator, clock or controller prerequisites are ready.

Does not own:
- Creation of the launch actions being sequenced.
- Runtime health monitoring after launch startup is complete.
"""

from __future__ import annotations

from collections.abc import Sequence
from functools import partial

from launch.actions import EmitEvent, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown
from launch_ros.actions import Node


def actions_after_successful_exit(
    event,
    context,
    *,
    actions: tuple[object, ...],
    prerequisite_name: str,
) -> list[object]:
    """Release dependents only when their one-shot prerequisite succeeded."""
    if context.is_shutdown:
        return []
    if event.returncode == 0:
        return list(actions)
    return [
        EmitEvent(
            event=Shutdown(
                reason=(
                    f"Prerequisite '{prerequisite_name}' failed with exit code "
                    f"{event.returncode}"
                )
            )
        )
    ]


def shutdown_after_process_exit(
    event,
    context,
    *,
    process_name: str,
) -> list[EmitEvent]:
    """Request launch shutdown after an unexpected critical-process exit."""
    if context.is_shutdown:
        return []
    return [
        EmitEvent(
            event=Shutdown(
                reason=f"Critical process '{process_name}' exited with code {event.returncode}"
            )
        )
    ]


def shutdown_after_process_failure(
    event,
    context,
    *,
    process_name: str,
) -> list[EmitEvent]:
    """Request shutdown only when a one-shot critical action failed."""
    if context.is_shutdown:
        return []
    if event.returncode == 0:
        return []
    return [
        EmitEvent(
            event=Shutdown(
                reason=f"Critical process '{process_name}' failed with code {event.returncode}"
            )
        )
    ]


def create_start_event_after_successful_exit(
    *,
    target_action: object,
    actions: list[object],
    prerequisite_name: str,
) -> RegisterEventHandler:
    """Create a fail-closed one-shot dependency edge."""
    return RegisterEventHandler(
        OnProcessExit(
            target_action=target_action,
            on_exit=partial(
                actions_after_successful_exit,
                actions=tuple(actions),
                prerequisite_name=prerequisite_name,
            ),
        )
    )


def create_shutdown_on_exit_event(
    *,
    target_action: object,
    process_name: str,
) -> RegisterEventHandler:
    """Create a critical long-lived process exit handler."""
    return RegisterEventHandler(
        OnProcessExit(
            target_action=target_action,
            on_exit=partial(shutdown_after_process_exit, process_name=process_name),
        )
    )


def create_shutdown_on_failure_event(
    *,
    target_action: object,
    process_name: str,
) -> RegisterEventHandler:
    """Create a one-shot process failure handler."""
    return RegisterEventHandler(
        OnProcessExit(
            target_action=target_action,
            on_exit=partial(shutdown_after_process_failure, process_name=process_name),
        )
    )


def create_gazebo_control_events(
    *,
    spawn_robot_node: Node,
    joint_state_broadcaster_spawner: Node,
    panda_arm_spawner: Node,
    panda_hand_spawner: Node,
    gripper_driver: Node,
    move_group_node: Node,
    motion_companion_nodes: Sequence[object] = (),
    planning_scene_node: Node | None = None,
) -> list[RegisterEventHandler]:
    """Sequence Gazebo spawn, controllers, gripper and motion services."""
    joint_state_start_event = create_start_event_after_successful_exit(
        target_action=spawn_robot_node,
        actions=[joint_state_broadcaster_spawner],
        prerequisite_name="Gazebo robot spawn",
    )
    panda_arm_start_event = create_start_event_after_successful_exit(
        target_action=joint_state_broadcaster_spawner,
        actions=[panda_arm_spawner],
        prerequisite_name="joint_state_broadcaster spawner",
    )
    panda_hand_start_event = create_start_event_after_successful_exit(
        target_action=panda_arm_spawner,
        actions=[panda_hand_spawner],
        prerequisite_name="panda_arm_controller spawner",
    )
    final_actions: list[object] = [
        gripper_driver,
        move_group_node,
        *motion_companion_nodes,
    ]
    if planning_scene_node is not None:
        final_actions.append(planning_scene_node)
    panda_gripper_start_event = create_start_event_after_successful_exit(
        target_action=panda_hand_spawner,
        actions=final_actions,
        prerequisite_name="panda_hand_controller spawner",
    )
    events = [
        joint_state_start_event,
        panda_arm_start_event,
        panda_hand_start_event,
        panda_gripper_start_event,
    ]
    if planning_scene_node is not None:
        events.append(
            create_shutdown_on_failure_event(
                target_action=planning_scene_node,
                process_name="static table planning scene",
            )
        )
    return events


def create_isaac_start_events(
    *,
    isaac_process: ExecuteProcess,
    ros_nodes_after_isaac: list[object],
    wait_for_clock: ExecuteProcess | None,
    wait_for_clock_enabled: bool,
    ros2_control_node: Node,
    joint_state_broadcaster_spawner: Node,
    panda_arm_spawner: Node,
    panda_hand_spawner: Node,
    move_group_node: Node,
    gripper_driver: Node,
    motion_companion_nodes: Sequence[object] = (),
    planning_scene_node: Node | None = None,
) -> list[object]:
    """Sequence Isaac startup, clock gating, control and motion services."""
    start_ros_after_isaac = RegisterEventHandler(
        OnProcessStart(
            target_action=isaac_process,
            on_start=[
                TimerAction(
                    period=3.0,
                    actions=[
                        *ros_nodes_after_isaac,
                    ],
                )
            ],
        )
    )

    start_control_after_clock = None
    if wait_for_clock_enabled:
        start_control_after_clock = create_start_event_after_successful_exit(
            target_action=wait_for_clock,
            actions=[ros2_control_node],
            prerequisite_name="/clock wait",
        )

    start_jsb_after_clock = RegisterEventHandler(
        OnProcessStart(
            target_action=ros2_control_node,
            on_start=[
                TimerAction(period=1.0, actions=[joint_state_broadcaster_spawner])
            ],
        )
    )
    start_panda_after_jsb = create_start_event_after_successful_exit(
        target_action=joint_state_broadcaster_spawner,
        actions=[panda_arm_spawner],
        prerequisite_name="joint_state_broadcaster spawner",
    )
    start_hand_after_arm = create_start_event_after_successful_exit(
        target_action=panda_arm_spawner,
        actions=[panda_hand_spawner],
        prerequisite_name="panda_arm_controller spawner",
    )
    final_actions: list[object] = [
        move_group_node,
        *motion_companion_nodes,
        gripper_driver,
    ]
    if planning_scene_node is not None:
        final_actions.append(planning_scene_node)
    start_move_group_and_gripper = create_start_event_after_successful_exit(
        target_action=panda_hand_spawner,
        actions=final_actions,
        prerequisite_name="panda_hand_controller spawner",
    )

    events = [
        create_shutdown_on_exit_event(
            target_action=isaac_process,
            process_name="Isaac Sim",
        ),
        start_ros_after_isaac,
        *(action for action in [start_control_after_clock, start_jsb_after_clock] if action),
        start_panda_after_jsb,
        start_hand_after_arm,
        start_move_group_and_gripper,
        isaac_process,
    ]
    if planning_scene_node is not None:
        events.append(
            create_shutdown_on_failure_event(
                target_action=planning_scene_node,
                process_name="static table planning scene",
            )
        )
    return events
