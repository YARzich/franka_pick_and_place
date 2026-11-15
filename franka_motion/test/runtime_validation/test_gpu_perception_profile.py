"""
Test the Franka-specific GPU perception evidence profile.

Responsibility:
- Pin the scenario's stamped ROS chain, diagnostics, assets, and harness wiring.
- Prove that the generic probe policy remains robot-independent and fail-closed.

Does not own:
- GPU inference, Gazebo execution, model accuracy, or runner provisioning.
"""

from dataclasses import replace
import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import jsonschema
import pytest

from manipulation_perception.validation.contracts import load_contract

from franka_motion_runtime.validation.runtime_config import (
    ConfigError,
    EvidenceKind,
    load_runtime_matrix,
)
from franka_motion_runtime.validation.runtime_service import (
    ARTIFACT_DIRECTORY_VARIABLE,
    _profile_environment,
)


_PACKAGE_ROOT = Path(__file__).resolve().parents[2]
_RUNTIME_CONFIG = _PACKAGE_ROOT / "config" / "runtime_validation"
_MATRIX_PATH = _RUNTIME_CONFIG / "runtime_matrix.v3.json"
_CONTRACT_PATH = (
    _RUNTIME_CONFIG
    / "gpu_perception"
    / "gazebo_gigapose_cube.v1.json"
)


def _profile():
    """Load the one canonical GPU perception runtime profile."""
    return load_runtime_matrix(_MATRIX_PATH).profile(
        "gazebo_gpu_perception_v2"
    )


def test_scenario_contract_conforms_to_generic_installed_schema() -> None:
    """Robot-specific policy must use the reusable package's public schema."""
    schema_path = (
        Path(get_package_share_directory("manipulation_perception"))
        / "config"
        / "gpu_perception_probe_contract.schema.v1.json"
    )
    schema = json.loads(schema_path.read_text(encoding="utf-8"))
    value = json.loads(_CONTRACT_PATH.read_text(encoding="utf-8"))

    jsonschema.Draft202012Validator(schema).validate(value)
    contract = load_contract(_CONTRACT_PATH)

    assert contract.contract_id == "franka-gazebo-gigapose-cube-v1"


def test_scenario_contract_pins_the_complete_stamped_chain() -> None:
    """Every required topic must contribute to one exact-stamp proof."""
    contract = load_contract(_CONTRACT_PATH)
    topics = {topic.role: topic for topic in contract.topics}

    assert {role: topic.topic for role, topic in topics.items()} == {
        "disparity": "/table_stereo/disparity_fixed",
        "pose_image": "/table_stereo/foundationpose/image",
        "pose_camera_info": "/table_stereo/foundationpose/camera_info",
        "pose_depth": "/table_stereo/foundationpose/depth_clean",
        "detection_2d": "/table_stereo/cube_detection2d",
        "segmentation": "/table_stereo/foundationpose/segmentation",
        "pose_hypotheses": "/table_stereo/megapose/output",
        "object_pose": "/table_stereo/cube_pose",
    }
    optical_roles = set(topics) - {"object_pose"}
    assert {
        topics[role].frame_id for role in optical_roles
    } == {"table_stereo_left_optical_frame"}
    assert topics["object_pose"].frame_id == "world"
    assert contract.terminal_pose_role == "object_pose"
    assert len(contract.correlation_groups) == 1
    assert set(contract.correlation_groups[0].roles) == set(topics)
    assert all(topic.maximum_rate_hz > 0.0 for topic in topics.values())
    assert not any("points" in topic.topic for topic in topics.values())


def test_scenario_contract_pins_diagnostics_and_critical_assets() -> None:
    """Evidence must observe each pipeline owner and the files it executes."""
    contract = load_contract(_CONTRACT_PATH)

    assert {
        item.role: item.status_name for item in contract.diagnostics
    } == {
        "stereo": (
            "ess_depth_pointcloud: perception/stereo_depth_pointcloud"
        ),
        "detector": "yolo_visualizer: perception/yolo_detection",
        "pose_backend": (
            "megapose_detection: perception/megapose_detection"
        ),
        "pose_adapter": (
            "megapose_detection_to_pose: perception/pose_estimation"
        ),
    }
    assert {
        item.role: item.source_topic_role for item in contract.diagnostics
    } == {
        "stereo": "disparity",
        "detector": "detection_2d",
        "pose_backend": "pose_hypotheses",
        "pose_adapter": "object_pose",
    }
    assert {asset.name for asset in contract.assets} == {
        "ess_engine",
        "yolo_model",
        "gigapose_checkpoint",
        "megapose_coarse_checkpoint",
        "megapose_refiner_checkpoint",
        "cube_mesh",
    }


def test_runtime_profile_uses_headless_gigapose_and_same_asset_tokens() -> None:
    """The probe may hash only the exact files selected by bringup."""
    profile = _profile()
    launch_argv = profile.background[0].argv
    probe_argv = profile.validation[0].argv

    assert profile.evidence_kind is EvidenceKind.GPU_PERCEPTION
    assert profile.environment["ROS_DOMAIN_ID"] == "76"
    for argument in (
        "headless:=true",
        "start_rviz:=false",
        "start_transfer_server:=false",
        "use_sim_time:=true",
        "mode:=gigaPose",
    ):
        assert argument in launch_argv
    for token in (
        "${FRANKA_GPU_ESS_ENGINE}",
        "${FRANKA_GPU_YOLO_MODEL}",
        "${FRANKA_GPU_GIGAPOSE_CHECKPOINT}",
        "${FRANKA_GPU_CUBE_MESH}",
    ):
        assert any(token in argument for argument in launch_argv)
        assert token in probe_argv
    assert "${FRANKA_GPU_MEGAPOSE_DATA_DIR}" in " ".join(launch_argv)
    for mandatory_foundationpose_token in (
        "${FRANKA_GPU_FOUNDATIONPOSE_REFINE_MODEL}",
        "${FRANKA_GPU_FOUNDATIONPOSE_REFINE_ENGINE}",
        "${FRANKA_GPU_FOUNDATIONPOSE_SCORE_MODEL}",
        "${FRANKA_GPU_FOUNDATIONPOSE_SCORE_ENGINE}",
    ):
        assert any(
            mandatory_foundationpose_token in argument
            for argument in launch_argv
        )
    assert "${FRANKA_GPU_MEGAPOSE_COARSE_CHECKPOINT}" in probe_argv
    assert "${FRANKA_GPU_MEGAPOSE_REFINER_CHECKPOINT}" in probe_argv
    assert profile.validation[0].stdout_regex == ("GPU_PERCEPTION_PASS",)
    assert profile.preflight[1].argv[0] == "/usr/bin/nvidia-smi"
    assert "/home/" not in json.dumps(
        json.loads(_MATRIX_PATH.read_text(encoding="utf-8"))["profiles"][
            profile.profile_id
        ]
    )
    assert "/home/" not in _CONTRACT_PATH.read_text(encoding="utf-8")


def test_gpu_evidence_cannot_be_claimed_without_validation() -> None:
    """A bringup-only profile must never produce GPU perception evidence."""
    with pytest.raises(ConfigError, match="requires validation"):
        replace(_profile(), validation=())


def test_harness_reserves_absolute_artifact_directory(tmp_path: Path) -> None:
    """Profile/environment input cannot redirect the nested evidence artifact."""
    requested = tmp_path / "runtime"
    environment = _profile_environment(
        {ARTIFACT_DIRECTORY_VARIABLE: "/profile-controlled"},
        {ARTIFACT_DIRECTORY_VARIABLE: "/caller-controlled"},
        "run-token",
        requested,
    )

    assert environment[ARTIFACT_DIRECTORY_VARIABLE] == str(
        requested.resolve()
    )
    assert environment["FRANKA_RUNTIME_RUN_TOKEN"] == "run-token"
