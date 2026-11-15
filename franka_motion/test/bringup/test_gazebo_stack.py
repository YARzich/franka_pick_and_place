"""
Test shared Gazebo launch composition without starting the simulator.

Responsibility: Verify generated model preparation and named motion-event delegation.
"""

from pathlib import Path
from types import SimpleNamespace

import pytest

import franka_motion_runtime.bringup.launch_parts.gazebo_stack as gazebo_stack_module


def test_model_preparation_writes_the_generated_description(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Path resolution, xacro generation and persistence remain one operation."""
    paths = SimpleNamespace(tmp_urdf=tmp_path / "panda.urdf")
    writes: list[tuple[object, str]] = []
    scenario = SimpleNamespace(base_pose={"z": "1.0"})
    monkeypatch.setattr(
        gazebo_stack_module,
        "create_gazebo_launch_paths",
        lambda requested: paths,
    )
    monkeypatch.setattr(
        gazebo_stack_module,
        "generate_gazebo_robot_description",
        lambda requested_paths, base_pose: "<robot />",
    )
    monkeypatch.setattr(
        gazebo_stack_module,
        "write_robot_urdf",
        lambda path, xml: writes.append((path, xml)),
    )

    model = gazebo_stack_module.prepare_gazebo_robot_model(scenario)

    assert model.paths is paths
    assert model.robot_description == {"robot_description": "<robot />"}
    assert writes == [(paths.tmp_urdf, "<robot />")]


def test_motion_events_delegate_named_stack_actions(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Planner selection must not alter the Gazebo controller readiness chain."""
    captured: dict[str, object] = {}

    def record_events(**kwargs):
        captured.update(kwargs)
        return ["events"]

    monkeypatch.setattr(
        gazebo_stack_module,
        "create_gazebo_control_events",
        record_events,
    )
    stack = gazebo_stack_module.GazeboControlStack(
        model="model",
        headless_simulator="headless",
        gui_simulator="gui",
        bridge="bridge",
        robot_state_publisher="robot_state_publisher",
        spawn_robot="spawn_robot",
        joint_state_broadcaster_spawner="joint_state_broadcaster",
        arm_controller_spawner="arm_controller",
        hand_controller_spawner="hand_controller",
        gripper_driver="gripper_driver",
        simulator_exit_events=("headless_exit", "gui_exit"),
    )

    result = stack.create_motion_start_events(
        motion_backend_node="planner",
        motion_companion_nodes=["servo"],
        planning_scene_node="table",
    )

    assert result == ["events"]
    assert captured == {
        "spawn_robot_node": "spawn_robot",
        "joint_state_broadcaster_spawner": "joint_state_broadcaster",
        "panda_arm_spawner": "arm_controller",
        "panda_hand_spawner": "hand_controller",
        "gripper_driver": "gripper_driver",
        "move_group_node": "planner",
        "motion_companion_nodes": ["servo"],
        "planning_scene_node": "table",
    }
