"""
Test production clock, health, scene, V2 workflow, and Servo launch wiring.

Responsibility: Preserve sim-time propagation, profile selection, and single-writer ownership.
"""

from pathlib import Path
from types import SimpleNamespace

from launch import LaunchContext
from launch.substitutions import LaunchConfiguration, TextSubstitution
import yaml

from franka_motion_runtime.bringup.launch_parts import (
    arguments,
    control,
    moveit,
    planning_scene,
    workflow,
)
from franka_motion_runtime.bringup.launch_parts.camera import (
    image_bridge,
    perception,
)


_PACKAGE_ROOT = Path(__file__).resolve().parents[2]


class _CapturedNode:
    """Expose factory inputs without depending on launch_ros internals."""

    def __init__(self, **kwargs) -> None:
        """Retain the exact keyword arguments passed by a launch factory."""
        vars(self).update(kwargs)


def _text(substitutions) -> str:
    """Flatten simple launch text substitutions used by these factories."""
    if isinstance(substitutions, str):
        return substitutions
    return "".join(
        substitution.text
        for substitution in substitutions
        if isinstance(substitution, TextSubstitution)
    )


def test_transfer_switch_defaults_are_scenario_selectable() -> None:
    """Empty and manipulation leaves may choose different stable defaults."""
    empty_argument = workflow.declare_start_transfer_server_argument(
        default_value=False,
    )
    table_argument = workflow.declare_start_transfer_server_argument(
        default_value=True,
    )

    assert empty_argument.name == "start_transfer_server"
    assert _text(empty_argument.default_value) == "false"
    assert _text(table_argument.default_value) == "true"


def test_servo_switch_is_strictly_opt_in() -> None:
    """Servo must not alter the default graph or accept ambiguous booleans."""
    argument = arguments.declare_start_servo_argument()

    assert argument.name == "start_servo"
    assert _text(argument.default_value) == "false"
    assert argument.choices == ["true", "false"]


def test_servo_factory_uses_local_config_without_resolving_optional_package(
    tmp_path: Path,
    monkeypatch,
) -> None:
    """Constructing a disabled graph must not require installed moveit_servo."""
    monkeypatch.setattr(moveit, "Node", _CapturedNode)
    servo_config = tmp_path / "servo.yaml"
    servo_config.write_text(
        "publish_period: 0.01\nmove_group_name: panda_arm\n",
        encoding="utf-8",
    )
    moveit_config = SimpleNamespace(
        robot_description={"robot_description": "urdf"},
        robot_description_semantic={"robot_description_semantic": "srdf"},
        robot_description_kinematics={
            "robot_description_kinematics": {"panda_arm": {}}
        },
        joint_limits={"robot_description_planning": {}},
    )
    start_servo = LaunchConfiguration("start_servo")

    node = moveit.create_moveit_servo_node(
        use_sim_time=True,
        start_servo=start_servo,
        moveit_config=moveit_config,
        servo_config_path=servo_config,
    )

    assert node.package == "moveit_servo"
    assert node.executable == "servo_node"
    assert node.name == "servo_node"
    assert node.parameters == [
        {
            "moveit_servo": {
                "publish_period": 0.01,
                "move_group_name": "panda_arm",
            }
        },
        {"update_period": 0.01, "planning_group_name": "panda_arm"},
        {"use_sim_time": True},
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
        moveit_config.joint_limits,
    ]
    context = LaunchContext()
    context.launch_configurations["start_servo"] = "false"
    assert not node.condition.evaluate(context)
    context.launch_configurations["start_servo"] = "true"
    assert node.condition.evaluate(context)


def test_robot_state_publisher_matches_controller_tf_rate(monkeypatch) -> None:
    """Dynamic TF must not undersample the 100 Hz joint-state producer."""
    monkeypatch.setattr(control, "Node", _CapturedNode)
    description = {"robot_description": "<robot />"}

    node = control.create_robot_state_publisher(
        use_sim_time=True,
        robot_description=description,
    )

    assert node.parameters == [
        {"use_sim_time": True, "publish_frequency": 100.0},
        description,
    ]


def test_octomap_gate_preserves_raw_topic_and_requires_both_fingers(
    monkeypatch,
) -> None:
    """Ensure fail-closed input without renaming the sensor contract."""
    monkeypatch.setattr(moveit, "Node", _CapturedNode)

    node = moveit.create_octomap_tf_gate_node(use_sim_time=True)
    parameters = node.parameters[0]

    assert node.package == "manipulation_moveit"
    assert node.executable == "moveit_octomap_tf_gate"
    assert parameters["use_sim_time"] is True
    assert parameters["input_topic"] == "/table_stereo/octomap_points"
    assert parameters["output_topic"] == "/table_stereo/moveit_octomap_points"
    assert parameters["expected_source_frame"] == (
        "table_stereo_left_optical_frame"
    )
    assert parameters["required_frames"] == [
        "panda_leftfinger",
        "panda_rightfinger",
    ]
    assert parameters["queue_size"] > 0
    assert parameters["transform_timeout_sec"] > 0.0
    assert parameters["health_timeout_sec"] > 0.0


def test_servo_task_server_is_independent_and_strictly_opt_in(
    monkeypatch,
) -> None:
    """Servo uses its dedicated process and never admits the retired V1 host."""
    monkeypatch.setattr(workflow, "Node", _CapturedNode)
    monkeypatch.setattr(
        workflow,
        "get_package_share_directory",
        lambda package_name: f"/share/{package_name}",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    node = workflow.create_servo_task_server_node(
        use_sim_time=use_sim_time,
    )

    assert node.package == "manipulation_moveit"
    assert node.executable == "moveit_servo_task_server"
    assert node.name == "franka_moveit_servo_task_server"
    assert node.parameters == [
        str(
            Path(
                "/share/franka_motion/config/manipulation/"
                "servo_to_pose.yaml"
            )
        ),
        str(
            Path(
                "/share/franka_motion/config/manipulation/"
                "robot_profile.yaml"
            )
        ),
        {"use_sim_time": use_sim_time},
    ]

    context = LaunchContext()
    context.launch_configurations["start_servo"] = "false"
    assert not node.condition.evaluate(context)
    context.launch_configurations["start_servo"] = "true"
    assert node.condition.evaluate(context)


def test_health_monitor_accepts_camera_profile_after_generic_defaults(
    monkeypatch,
) -> None:
    """Camera bindings must override only the monitor's perception policy."""
    monkeypatch.setattr(workflow, "Node", _CapturedNode)
    monkeypatch.setattr(
        workflow,
        "get_package_share_directory",
        lambda package_name: f"/share/{package_name}",
    )
    perception_config = LaunchConfiguration("table_camera_params_file")
    node = workflow.create_system_health_monitor_node(
        use_sim_time=True,
        perception_config=perception_config,
    )

    assert node.package == "manipulation"
    assert node.parameters == [
        str(
            Path(
                "/share/manipulation/config/"
                "system_health.yaml"
            )
        ),
        str(
            Path(
                "/share/franka_motion/config/manipulation/"
                "system_health.yaml"
            )
        ),
        perception_config,
        str(
            Path(
                "/share/franka_motion/config/manipulation/"
                "robot_profile.yaml"
            )
        ),
        {"use_sim_time": True},
    ]


def test_servo_policy_matches_the_standalone_action_contract() -> None:
    """Fail fast when Servo endpoints, safety values, or node scope drift."""
    config_directory = _PACKAGE_ROOT / "config" / "manipulation"
    health = yaml.safe_load(
        (config_directory / "system_health.yaml").read_text(encoding="utf-8")
    )
    servo = yaml.safe_load(
        (config_directory / "servo_to_pose.yaml").read_text(encoding="utf-8")
    )
    servo_backend = yaml.safe_load(
        (
            _PACKAGE_ROOT / "config" / "moveit" / "servo.yaml"
        ).read_text(encoding="utf-8")
    )

    health_parameters = health["system_health_monitor"]["ros__parameters"]
    assert health_parameters["motion_backend_action"] == "/move_action"
    assert health_parameters["motion_backend_action_type"] == (
        "moveit_msgs/action/MoveGroup"
    )

    servo_parameters = servo[
        "/**/franka_moveit_servo_task_server"
    ]["ros__parameters"]
    assert set(servo_parameters) == {
        "servo_to_pose.preset_id",
        "servo_to_pose.preset_schema_version",
        "servo_to_pose.action_name",
        "servo_to_pose.pause_service_name",
        "servo_to_pose.heartbeat_period_sec",
        "servo_to_pose.default_timeout_sec",
        "servo_to_pose.position_tolerance_m",
        "servo_to_pose.orientation_tolerance_rad",
        "servo_to_pose.stopped_velocity_tolerance",
        "servo_to_pose.required_target_samples",
        "servo_to_pose.required_stopped_samples",
        "servo_to_pose.halt_timeout_sec",
        "servo_to_pose.control_period_sec",
        "servo_to_pose.incoming_command_timeout_sec",
        "servo_to_pose.service_call_timeout_sec",
        "servo_to_pose.target_prepare_timeout_sec",
        "servo_to_pose.feedback_startup_timeout_sec",
        "servo_to_pose.observation_timeout_sec",
        "servo_to_pose.pose_command_topic",
        "servo_to_pose.command_type_service",
        "servo_to_pose.backend_pause_service",
        "servo_to_pose.status_topic",
        "servo_to_pose.joint_state_topic",
        "servo_to_pose.poll_period_sec",
        "servo_to_pose.current_tf_max_age_sec",
        "servo_to_pose.joint_state_max_age_sec",
        "servo_to_pose.status_max_age_sec",
    }
    assert (
        servo_parameters["servo_to_pose.control_period_sec"]
        < servo_parameters["servo_to_pose.incoming_command_timeout_sec"]
    )
    assert servo_backend["pose_tracking"]["linear_tolerance"] == (
        servo_parameters["servo_to_pose.position_tolerance_m"]
    )
    assert servo_backend["pose_tracking"]["angular_tolerance"] == (
        servo_parameters["servo_to_pose.orientation_tolerance_rad"]
    )


def test_table_bootstrap_uses_canonical_single_writer(monkeypatch) -> None:
    """Production table geometry must not address MoveIt directly."""
    monkeypatch.setattr(planning_scene, "Node", _CapturedNode)
    monkeypatch.setattr(
        planning_scene,
        "get_package_share_directory",
        lambda package_name: f"/share/{package_name}",
    )
    coordinator = planning_scene.create_scene_coordinator_node(
        use_sim_time=True,
    )
    table = planning_scene.create_table_planning_scene_node(
        use_sim_time=True,
        profile=planning_scene.GAZEBO_TABLE_COLLISION,
    )

    assert coordinator.package == "manipulation"
    assert coordinator.executable == (
        "planning_scene_coordinator"
    )
    assert coordinator.parameters == [
        "/share/manipulation/config/planning_scene_coordinator.yaml",
        (
            "/share/franka_motion/config/manipulation/"
            "planning_scene_coordinator.yaml"
        ),
        (
            "/share/franka_motion/config/manipulation/"
            "robot_profile.yaml"
        ),
        {"use_sim_time": True},
    ]
    assert table.package == "franka_motion"
    assert table.executable == "static_table_scene"
    assert table.parameters[0]["use_scene_coordinator"] is True
    assert table.parameters[0]["frame_id"] == "panda_link0"


def test_gazebo_image_bridges_use_selected_sim_time(monkeypatch) -> None:
    """Both ros_gz image nodes must use the same launch clock policy."""
    monkeypatch.setattr(image_bridge, "Node", _CapturedNode)
    use_sim_time = LaunchConfiguration("use_sim_time")

    bridges = image_bridge.create_image_bridge_nodes(
        Path("/share/franka_motion/managed_process_wrapper.py"),
        use_sim_time,
    )

    assert bridges.left.package == "ros_gz_image"
    assert bridges.right.package == "ros_gz_image"
    assert bridges.left.parameters == [{"use_sim_time": use_sim_time}]
    assert bridges.right.parameters == [{"use_sim_time": use_sim_time}]

    legacy_default = image_bridge.create_image_bridge_nodes(
        Path("/share/franka_motion/managed_process_wrapper.py"),
    )
    assert legacy_default.left.parameters == [{"use_sim_time": True}]
    assert legacy_default.right.parameters == [{"use_sim_time": True}]


def test_controller_spawners_use_selected_sim_time(monkeypatch) -> None:
    """Transient controller-manager nodes must share the scenario clock."""
    monkeypatch.setattr(control, "Node", _CapturedNode)
    use_sim_time = LaunchConfiguration("use_sim_time")

    gazebo_spawners = control.create_gazebo_controller_spawners(
        Path("/share/franka_motion/config/control/controllers.yaml"),
        use_sim_time=use_sim_time,
    )
    isaac_spawners = control.create_isaac_controller_spawners(
        SimpleNamespace(
            managed_process_wrapper=Path("/share/bringup/wrapper.py"),
            ros2_control_config=Path("/share/control/controllers.yaml"),
        ),
        use_sim_time=False,
    )

    assert all(
        spawner.parameters == [{"use_sim_time": use_sim_time}]
        for spawner in gazebo_spawners
    )
    assert all(
        spawner.parameters == [{"use_sim_time": False}]
        for spawner in isaac_spawners
    )


def test_perception_factory_forwards_common_sim_time_to_bridges(
    monkeypatch,
) -> None:
    """The aggregate factory must not replace the shared launch substitution."""
    captured: dict[str, object] = {}

    def capture_image_bridges(wrapper, use_sim_time):
        captured["wrapper"] = wrapper
        captured["use_sim_time"] = use_sim_time
        return SimpleNamespace(left=None, right=None)

    monkeypatch.setattr(
        perception,
        "create_image_bridge_nodes",
        capture_image_bridges,
    )
    for factory_name in (
        "create_sgbm_node",
        "create_ess_actions",
        "create_pose_estimator_input_nodes",
        "create_foundationpose_actions",
        "create_megapose_actions",
        "create_yolo_node",
    ):
        monkeypatch.setattr(perception, factory_name, lambda *args: None)

    use_sim_time = LaunchConfiguration("use_sim_time")
    wrapper = Path("/share/franka_motion/managed_process_wrapper.py")
    perception.create_perception_actions(
        SimpleNamespace(managed_process_wrapper=wrapper),
        {"use_sim_time": use_sim_time},
        {},
    )

    assert captured == {
        "wrapper": wrapper,
        "use_sim_time": use_sim_time,
    }
