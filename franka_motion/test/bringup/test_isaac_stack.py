"""
Test shared Isaac Sim launch composition without starting the simulator.

Responsibility: Verify argument resolution, node ordering, and motion-event delegation.
"""

from launch import LaunchContext
import pytest

from franka_motion_runtime.bringup.launch_parts.arguments import (
    declare_common_isaac_arguments,
    declare_isaac_runtime_arguments,
)
import franka_motion_runtime.bringup.launch_parts.isaac_stack as isaac_stack_module


def _settings(*, wait_for_clock: bool) -> isaac_stack_module.IsaacRuntimeSettings:
    """Create resolved settings without invoking a launch context."""
    return isaac_stack_module.IsaacRuntimeSettings(
        use_sim_time=True,
        headless=True,
        fix_base=True,
        wait_for_clock=wait_for_clock,
        time_steps="0",
        isaac_python="/opt/isaac/python.sh",
        isaac_robot_prim="/World/Franka",
        isaac_experience="",
        rmw_implementation="rmw_fastrtps_cpp",
    )


def _control_stack(
    *,
    wait_for_clock: bool,
) -> isaac_stack_module.IsaacControlStack:
    """Create a stack with identity sentinels for ordering assertions."""
    return isaac_stack_module.IsaacControlStack(
        settings=_settings(wait_for_clock=wait_for_clock),
        robot_state_publisher="robot_state_publisher",
        ros2_control_node="ros2_control",
        joint_state_broadcaster_spawner="joint_state_broadcaster",
        arm_controller_spawner="arm_controller",
        hand_controller_spawner="hand_controller",
        gripper_driver="gripper_driver",
        wait_for_clock_action=(
            "wait_for_clock" if wait_for_clock else None
        ),
    )


def test_argument_factories_preserve_names_and_order() -> None:
    """Shared declarations must retain both standard and camera contracts."""
    common_names = [
        argument.name for argument in declare_common_isaac_arguments()
    ]
    table_names = [
        argument.name
        for argument in declare_common_isaac_arguments(
            include_isaac_experience=True,
        )
    ]
    camera_runtime_names = [
        argument.name
        for argument in declare_isaac_runtime_arguments(
            include_isaac_experience=True,
            include_gxf_log_level=True,
        )
    ]

    assert common_names == [
        "use_sim_time",
        "headless",
        "fix_base",
        "isaac_python",
        "isaac_robot_prim",
        "time_steps",
        "wait_for_clock",
        "ros_domain_id",
        "rmw_implementation",
    ]
    assert table_names == [
        "use_sim_time",
        "headless",
        "fix_base",
        "isaac_python",
        "isaac_experience",
        "isaac_robot_prim",
        "time_steps",
        "wait_for_clock",
        "ros_domain_id",
        "rmw_implementation",
    ]
    assert camera_runtime_names == table_names[2:] + ["gxf_log_level"]


def test_settings_resolve_once_and_ignore_undeclared_experience() -> None:
    """Leaves without the optional argument must not resolve it accidentally."""
    context = LaunchContext()
    context.launch_configurations.update(
        {
            "use_sim_time": "true",
            "headless": "yes",
            "fix_base": "1",
            "wait_for_clock": "false",
            "time_steps": "120",
            "isaac_python": "/opt/isaac/python.sh",
            "isaac_robot_prim": "/World/TestPanda",
            "rmw_implementation": "rmw_cyclonedds_cpp",
        }
    )

    settings = isaac_stack_module.IsaacRuntimeSettings.from_context(
        context,
        include_isaac_experience=False,
    )

    assert settings.use_sim_time
    assert settings.headless
    assert settings.fix_base
    assert not settings.wait_for_clock
    assert settings.time_steps == "120"
    assert settings.isaac_robot_prim == "/World/TestPanda"
    assert settings.isaac_experience == ""
    assert settings.rmw_implementation == "rmw_cyclonedds_cpp"

    context.launch_configurations["isaac_experience"] = "/opt/isaac/app.kit"
    settings_with_experience = (
        isaac_stack_module.IsaacRuntimeSettings.from_context(
            context,
            include_isaac_experience=True,
        )
    )
    assert settings_with_experience.isaac_experience == "/opt/isaac/app.kit"


@pytest.mark.parametrize(
    ("wait_for_clock", "expected_gate"),
    [(False, "ros2_control"), (True, "wait_for_clock")],
)
def test_nodes_keep_control_gate_last(
    wait_for_clock: bool,
    expected_gate: str,
) -> None:
    """Clock/control gating must remain after all startup observer nodes."""
    stack = _control_stack(wait_for_clock=wait_for_clock)

    nodes = stack.nodes_after_simulator("health", "perception")

    assert nodes == [
        "robot_state_publisher",
        "health",
        "perception",
        expected_gate,
    ]


def test_motion_events_delegate_named_stack_actions(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The value object must preserve the established readiness API exactly."""
    captured: dict[str, object] = {}

    def record_events(**kwargs):
        captured.update(kwargs)
        return ["events"]

    monkeypatch.setattr(
        isaac_stack_module,
        "create_isaac_start_events",
        record_events,
    )
    stack = _control_stack(wait_for_clock=True)

    result = stack.create_motion_start_events(
        isaac_process="isaac",
        motion_backend_node="planner",
        motion_companion_nodes=["servo"],
        additional_nodes=["health"],
        planning_scene_node="table",
    )

    assert result == ["events"]
    assert captured == {
        "isaac_process": "isaac",
        "ros_nodes_after_isaac": [
            "robot_state_publisher",
            "health",
            "wait_for_clock",
        ],
        "wait_for_clock": "wait_for_clock",
        "wait_for_clock_enabled": True,
        "ros2_control_node": "ros2_control",
        "joint_state_broadcaster_spawner": "joint_state_broadcaster",
        "panda_arm_spawner": "arm_controller",
        "panda_hand_spawner": "hand_controller",
        "move_group_node": "planner",
        "gripper_driver": "gripper_driver",
        "motion_companion_nodes": ["servo"],
        "planning_scene_node": "table",
    }
