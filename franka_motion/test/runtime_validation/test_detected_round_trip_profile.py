"""
Test the Franka detected-cube round-trip runtime evidence profile.

Responsibility:
- Pin the single camera-to-motion chain: GPU perception bringup with the
  transfer server plus a repeated detected round-trip validation client.
- Keep this claim separate from gpu_perception (no motion) and simulator_soak
  (no camera in the loop).

Does not own:
- GPU inference, Gazebo execution, transfer workflow internals, or runners.
"""

from dataclasses import replace
import json
from pathlib import Path

import pytest

from franka_motion_runtime.validation.runtime_config import (
    ConfigError,
    EvidenceKind,
    load_runtime_matrix,
)


_PACKAGE_ROOT = Path(__file__).resolve().parents[2]
_MATRIX_PATH = (
    _PACKAGE_ROOT / "config" / "runtime_validation" / "runtime_matrix.v3.json"
)


def _matrix():
    return load_runtime_matrix(_MATRIX_PATH)


def _profile():
    """Load the one canonical detected round-trip runtime profile."""
    return _matrix().profile("gazebo_detected_round_trip_v2")


def test_detected_round_trip_claims_its_own_evidence_kind() -> None:
    """The full chain is a separate claim with an isolated ROS domain."""
    matrix = _matrix()
    profile = _profile()

    assert profile.evidence_kind is EvidenceKind.DETECTED_ROUND_TRIP
    domain_ids = [
        item.environment["ROS_DOMAIN_ID"] for item in matrix.profiles.values()
    ]
    assert len(domain_ids) == len(set(domain_ids))


def test_detected_round_trip_reuses_gpu_perception_bringup_with_server() -> None:
    """Bringup must match the perception profile but start the V2 server."""
    matrix = _matrix()
    perception = matrix.profile("gazebo_gpu_perception_v2")
    profile = _profile()
    launch_argv = profile.background[0].argv
    perception_argv = perception.background[0].argv

    assert profile.requirements == perception.requirements
    assert launch_argv[:4] == (
        "ros2",
        "launch",
        "franka_motion",
        "franka_gazebo_moveit_table_camera.launch.py",
    )
    assert "start_transfer_server:=true" in launch_argv
    assert "start_transfer_server:=false" in perception_argv
    assert set(launch_argv[5:]) - {"start_transfer_server:=true"} == (
        set(perception_argv[5:]) - {"start_transfer_server:=false"}
    )


def test_detected_round_trip_waits_for_perception_action_and_health() -> None:
    """Readiness must cover the perception chain plus the transfer path."""
    matrix = _matrix()
    perception = matrix.profile("gazebo_gpu_perception_v2")
    readiness = {command.name: command for command in _profile().readiness}

    perception_topics = next(
        command
        for command in perception.readiness
        if command.name == "gpu_perception_topics_ready"
    )
    assert readiness["detected_round_trip_perception_topics_ready"].stdout_regex == (
        perception_topics.stdout_regex
    )
    action_probe = readiness["detected_round_trip_transfer_action_ready"]
    assert action_probe.stdout_regex == ("(?m)^/transfer_object_sequence_v2 ",)
    assert action_probe.retry_until_pass
    health_probe = readiness["detected_round_trip_health_ready_after_startup_grace"]
    assert any(
        "motion_ready: true" in pattern for pattern in health_probe.stdout_regex
    )
    assert health_probe.retry_until_pass


def test_detected_round_trip_repeats_the_detected_client_five_times() -> None:
    """The claim is five consecutive camera-driven transfer-and-return runs."""
    profile = _profile()

    assert len(profile.validation) == 1
    validation = profile.validation[0]
    assert validation.argv[:4] == (
        "ros2",
        "launch",
        "franka_motion",
        "franka_detected_cube_round_trip.launch.py",
    )
    assert "use_sim_time:=true" in validation.argv
    assert validation.repetitions == 5
    assert validation.expected_return_codes == (0,)


def test_detected_round_trip_profile_is_machine_neutral() -> None:
    """The matrix entry must reference assets only through explicit tokens."""
    profile_value = json.loads(_MATRIX_PATH.read_text(encoding="utf-8"))[
        "profiles"
    ]["gazebo_detected_round_trip_v2"]

    assert "/home/" not in json.dumps(profile_value)


def test_detected_round_trip_evidence_requires_validation() -> None:
    """A bringup-only profile must never claim the full-chain evidence."""
    with pytest.raises(ConfigError, match="requires validation"):
        replace(_profile(), validation=())
