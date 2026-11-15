"""
Test the Panda binding for the generic ordered transfer workflow.

Responsibility:
- Preserve public endpoints, backend capabilities, steady budgets, and launch condition.

Does not own:
- Generic sequence state-machine behavior or simulator runtime evidence.
"""

from pathlib import Path

from launch import LaunchContext
from launch.substitutions import LaunchConfiguration
import yaml

from franka_motion_runtime.bringup.launch_parts import workflow


_PACKAGE_ROOT = Path(__file__).resolve().parents[2]


class _CapturedNode:
    """Expose factory inputs without depending on launch_ros internals."""

    def __init__(self, **kwargs) -> None:
        """Retain the exact keyword arguments passed by the launch factory."""
        vars(self).update(kwargs)


def test_sequence_server_uses_panda_profile_and_shared_condition(
    monkeypatch,
) -> None:
    """The outer workflow must bind Panda values without a second switch."""
    monkeypatch.setattr(workflow, "Node", _CapturedNode)
    monkeypatch.setattr(
        workflow,
        "get_package_share_directory",
        lambda package_name: f"/share/{package_name}",
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    node = workflow.create_transfer_sequence_server_node(
        use_sim_time=use_sim_time,
    )

    assert node.package == "franka_motion"
    assert node.executable == "franka_transfer_sequence_v2_server"
    assert not hasattr(node, "name")
    assert node.parameters == [
        str(
            Path(
                "/share/franka_motion/config/manipulation/"
                "transfer_object_sequence_v2.yaml"
            )
        ),
        str(Path("/share/franka_motion/config/manipulation/moveit_backend.yaml")),
        str(
            Path(
                "/share/franka_motion/config/manipulation/"
                "transfer_object_sequence_v2_deployment.yaml"
            )
        ),
        str(
            Path(
                "/share/franka_motion/config/manipulation/"
                "robot_profile.yaml"
            )
        ),
        {
            "workflow.observation_provider_id": "configured",
            "workflow.observation_provider_config_file": (
                "package://franka_motion/config/tasks/"
                "configured_observations_gazebo.yaml"
            ),
            "moveit.planning_pipeline": "ompl",
            "endpoints.physical_attachment_service": "",
            "use_sim_time": use_sim_time,
        },
    ]

    context = LaunchContext()
    context.launch_configurations["start_transfer_server"] = "true"
    assert node.condition.evaluate(context)
    context.launch_configurations["start_transfer_server"] = "false"
    assert not node.condition.evaluate(context)


def test_detected_sequence_server_selects_only_the_observation_adapter(
    monkeypatch,
) -> None:
    """Configured and detected scenarios retain one process and lifecycle."""
    monkeypatch.setattr(workflow, "Node", _CapturedNode)
    monkeypatch.setattr(
        workflow,
        "get_package_share_directory",
        lambda package_name: f"/share/{package_name}",
    )
    node = workflow.create_transfer_sequence_server_node(
        use_sim_time=True,
        detected_observation=True,
    )

    assert node.package == "franka_motion"
    assert node.executable == "franka_transfer_sequence_v2_server"
    assert not hasattr(node, "name")
    assert len(node.parameters) == 5
    assert node.parameters[0].endswith("transfer_object_sequence_v2.yaml")
    assert node.parameters[1].endswith("moveit_backend.yaml")
    assert node.parameters[2].endswith("transfer_object_sequence_v2_deployment.yaml")
    assert node.parameters[3].endswith("robot_profile.yaml")
    assert node.parameters[4] == {
        "workflow.observation_provider_id": "detected",
        "workflow.observation_provider_config_file": (
            "package://franka_motion/config/tasks/detected_observation.yaml"
        ),
        "moveit.planning_pipeline": "ompl",
        "endpoints.physical_attachment_service": "",
        "use_sim_time": True,
    }


def test_v2_policy_is_split_by_owner_and_has_only_physical_timeouts() -> None:
    """Task, backend, deployment, and robot values must not collapse again."""
    task_path = (
        _PACKAGE_ROOT / "config" / "manipulation" /
        "transfer_object_sequence_v2.yaml"
    )
    task_config = yaml.safe_load(task_path.read_text(encoding="utf-8"))
    task_parameters = task_config[
        "franka_transfer_object_sequence_server"
    ]["ros__parameters"]
    timeout_keys = {
        key for key in task_parameters if "timeout" in key.lower()
    }
    assert timeout_keys == {
        "physical_actions.arm_motion_timeout_sec",
        "physical_actions.gripper_command_timeout_sec",
    }
    assert task_parameters["workflow.strategy_id"] == "pick_lift_place"
    assert task_parameters["workflow.max_steps"] == 32

    deployment_path = (
        _PACKAGE_ROOT / "config" / "manipulation" /
        "transfer_object_sequence_v2_deployment.yaml"
    )
    deployment = yaml.safe_load(deployment_path.read_text(encoding="utf-8"))
    deployment_parameters = deployment[
        "/**/franka_transfer_object_sequence_server"
    ]["ros__parameters"]
    expected_endpoints = {
        "endpoints.sequence_action",
        "endpoints.sequence_pause_service",
        "endpoints.detected_pose_topic",
        "endpoints.health_topic",
        "endpoints.scene_state_service",
        "endpoints.scene_acquire_service",
        "endpoints.scene_renew_service",
        "endpoints.scene_apply_service",
        "endpoints.scene_validate_service",
        "endpoints.scene_release_service",
        "endpoints.planning_scene_read_service",
        "endpoints.physical_attachment_service",
    }
    assert set(deployment_parameters) == expected_endpoints
    assert deployment_parameters["endpoints.sequence_action"] == (
        "/transfer_object_sequence_v2"
    )
    assert deployment_parameters[
        "endpoints.physical_attachment_service"
    ] == ""


def test_detected_provider_and_catalog_replace_perception_specific_task_policy() -> None:
    """Detection routing and geometry live in their dedicated V2 resources."""
    config_path = (
        _PACKAGE_ROOT / "config" / "tasks" / "detected_observation.yaml"
    )
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    assert config == {
        "schema_version": 1,
        "provider_id": "detected",
        "object_id": "cube",
        "target_frame": "panda_link0",
        # Support-clearance bound: detector z noise must not sink the resting
        # cube into the tabletop collision box (see detected_observation.yaml).
        "minimum_position_z_m": 0.027,
        "max_observation_age_sec": 5.0,
    }

    catalog_path = _PACKAGE_ROOT / "config" / "tasks" / "object_catalog.yaml"
    catalog = yaml.safe_load(catalog_path.read_text(encoding="utf-8"))
    assert catalog["objects"]["cube"]["profile_id"] == "cube_50mm"
    assert catalog["profiles"]["cube_50mm"]["geometry"]["kind"] == "primitive"


def test_table_camera_launches_select_detected_provider_on_the_same_server() -> None:
    """Both simulator leaves select an adapter, not a second action lifecycle."""
    sources = (
        _PACKAGE_ROOT / "launch" /
        "franka_gazebo_moveit_table_camera.launch.py",
        _PACKAGE_ROOT / "franka_motion_runtime" / "bringup" /
        "isaac_table_camera_moveit.py",
    )
    for source_path in sources:
        source = source_path.read_text(encoding="utf-8")
        assert "create_transfer_sequence_server_node(" in source
        assert "detected_observation=True" in source


def test_isaac_leaves_enable_only_the_object_scoped_physical_barrier() -> None:
    """Isaac binds the fixed-joint ACK service while Gazebo stays disabled."""
    isaac_sources = (
        _PACKAGE_ROOT / "franka_motion_runtime" / "bringup" /
        "isaac_moveit.py",
        _PACKAGE_ROOT / "franka_motion_runtime" / "bringup" /
        "isaac_table_camera_moveit.py",
    )
    for source_path in isaac_sources:
        source = source_path.read_text(encoding="utf-8")
        assert (
            "physical_attachment_service="
            "ISAAC_PHYSICAL_ATTACHMENT_SERVICE"
        ) in source

    workflow_source = (
        _PACKAGE_ROOT / "franka_motion_runtime" / "bringup" /
        "launch_parts" / "workflow.py"
    ).read_text(encoding="utf-8")
    assert (
        '"/franka_motion/isaac/set_pick_cube_attached"'
        in workflow_source
    )
    gazebo_source = (
        _PACKAGE_ROOT / "franka_motion_runtime" / "bringup" /
        "gazebo_moveit.py"
    ).read_text(encoding="utf-8")
    assert "ISAAC_PHYSICAL_ATTACHMENT_SERVICE" not in gazebo_source
