"""
Test versioned runtime-validation configuration and command construction.

Responsibility: Verify evidence profiles, probes, fault sequences, and canonical ROS contracts.
"""

from dataclasses import replace
import json
from pathlib import Path
import re

import jsonschema
import pytest

from franka_motion_runtime.validation.runtime_config import (
    CommandSpec,
    ConfigError,
    EvidenceKind,
    load_runtime_matrix,
)

_PACKAGE_ROOT = Path(__file__).resolve().parents[2]
_RUNTIME_CONFIG = _PACKAGE_ROOT / "config" / "runtime_validation"
_MATRIX_PATH = _RUNTIME_CONFIG / "runtime_matrix.v3.json"
_SCHEMA_PATH = _RUNTIME_CONFIG / "runtime_matrix.schema.v3.json"
_CLOCK_DOMAIN_ROLES = {
    "transfer_server": "/franka_transfer_object_sequence_server",
    "scene_coordinator": "/planning_scene_coordinator",
    "move_group": "/move_group",
    "controller_manager": "/controller_manager",
}
_CLOCK_EVIDENCE_SUFFIXES = {
    *(f"{role}_sim_time" for role in _CLOCK_DOMAIN_ROLES),
    "joint_states_nonzero_stamp",
    "world_to_panda_base_tf",
}


def _matrix_value() -> dict[str, object]:
    return json.loads(_MATRIX_PATH.read_text(encoding="utf-8"))


def _write_matrix(tmp_path: Path, value: object) -> Path:
    path = tmp_path / "runtime_matrix.json"
    path.write_text(json.dumps(value), encoding="utf-8")
    return path


def test_versioned_matrix_exposes_only_runtime_evidence_profiles() -> None:
    """No profile may turn preflight-only success into validation evidence."""
    matrix = load_runtime_matrix(_MATRIX_PATH)

    assert matrix.schema_version == 3
    assert set(matrix.profiles) == {
        "gazebo_smoke_v2",
        "gazebo_active_cancel_v2",
        "gazebo_backend_death_v2",
        "gazebo_detected_round_trip_v2",
        "gazebo_gpu_perception_v2",
        "gazebo_health_fault_v2",
        "gazebo_soak_v2",
        "isaac_smoke_v2",
        "isaac_soak_v2",
    }
    assert {
        profile.evidence_kind for profile in matrix.profiles.values()
    } == {
        EvidenceKind.SIMULATOR_MOTION,
        EvidenceKind.DIAGNOSTIC_FAULT,
        EvidenceKind.GPU_PERCEPTION,
        EvidenceKind.SIMULATOR_SOAK,
        EvidenceKind.DETECTED_ROUND_TRIP,
    }


def test_fault_profile_uses_canonical_bringup_owners_once() -> None:
    """Fault validation must not duplicate scene or health ROS node owners."""
    matrix = load_runtime_matrix(_MATRIX_PATH)
    profile = matrix.profile("gazebo_health_fault_v2")

    assert tuple(command.name for command in profile.background) == (
        "fault_gazebo_bringup",
    )
    assert "franka_gazebo_moveit_table.launch.py" in profile.background[0].argv


def test_fault_readiness_allows_cold_scene_reconciliation() -> None:
    """Cold MoveIt startup must fit before fault injection can begin."""
    profile = load_runtime_matrix(_MATRIX_PATH).profile(
        "gazebo_health_fault_v2"
    )
    readiness = profile.readiness[0]

    assert readiness.name == "healthy_snapshot_after_startup_grace"
    assert readiness.retry_until_pass
    assert readiness.timeout_sec == 60


def test_gazebo_version_probes_accept_the_jazzy_cli_output_contract() -> None:
    """Gazebo Harmonic prints bare semantic versions for `sim --versions`."""
    matrix = load_runtime_matrix(_MATRIX_PATH)

    for profile_name in (
        "gazebo_smoke_v2",
        "gazebo_active_cancel_v2",
        "gazebo_backend_death_v2",
        "gazebo_detected_round_trip_v2",
        "gazebo_gpu_perception_v2",
        "gazebo_health_fault_v2",
        "gazebo_soak_v2",
    ):
        profile = matrix.profile(profile_name)
        version_probe = next(
            command for command in profile.preflight
            if command.argv == ("gz", "sim", "--versions")
        )
        assert any(
            re.search(expression, "8.11.0\n8.11.0\n")
            for expression in version_probe.stdout_regex
        )


def test_gazebo_smoke_exercises_canonical_transfer_contract() -> None:
    """Simulator smoke must validate the production action workflow."""
    matrix = load_runtime_matrix(_MATRIX_PATH)
    profile = matrix.profile("gazebo_smoke_v2")

    assert profile.preflight[-1].argv == (
        "ros2",
        "pkg",
        "prefix",
        "franka_motion",
    )
    assert "franka_gazebo_moveit_table.launch.py" in profile.background[0].argv
    assert profile.validation[0].argv[:4] == (
        "ros2",
        "launch",
        "franka_motion",
        "franka_gazebo_move_cube.launch.py",
    )
    action_probe = next(
        command for command in profile.readiness
        if command.name == "gazebo_actions_ready"
    )
    assert any(
        "/transfer_object_sequence_v2" in pattern
        for pattern in action_probe.stdout_regex
    )


@pytest.mark.parametrize(
    ("profile_name", "prefix"),
    [
        ("gazebo_smoke_v2", "gazebo"),
        ("isaac_smoke_v2", "isaac"),
    ],
)
def test_smoke_readiness_proves_one_sim_clock_and_world_base_tf(
    profile_name: str,
    prefix: str,
) -> None:
    """Smoke evidence must reject mixed clocks, zero stamps, or a missing base TF."""
    profile = load_runtime_matrix(_MATRIX_PATH).profile(profile_name)
    readiness = {command.name: command for command in profile.readiness}

    for role, node_name in _CLOCK_DOMAIN_ROLES.items():
        probe = readiness[f"{prefix}_{role}_sim_time"]
        assert probe.argv == (
            "ros2",
            "param",
            "get",
            "--timeout",
            "5",
            node_name,
            "use_sim_time",
        )
        assert probe.stdout_regex == (r"(?m)^Boolean value is: True$",)
        assert probe.retry_until_pass

    stamp_probe = readiness[f"{prefix}_joint_states_nonzero_stamp"]
    assert stamp_probe.argv == (
        "ros2",
        "topic",
        "echo",
        "--once",
        "--filter",
        "m.header.stamp.sec != 0 or m.header.stamp.nanosec != 0",
        "/joint_states",
        "sensor_msgs/msg/JointState",
    )
    assert stamp_probe.retry_until_pass

    tf_probe = readiness[f"{prefix}_world_to_panda_base_tf"]
    assert tf_probe.argv == (
        "timeout",
        "--signal=INT",
        "--kill-after=2s",
        "5s",
        "ros2",
        "run",
        "tf2_ros",
        "tf2_echo",
        "world",
        "panda_link0",
        "-r",
        "10",
    )
    assert tf_probe.expected_return_codes == (124,)
    representative_tf_output = (
        "At time 12.345\n"
        "- Translation: [0.000, 0.000, 0.000]\n"
    )
    assert all(
        re.search(expression, representative_tf_output)
        for expression in tf_probe.stdout_regex
    )
    assert tf_probe.retry_until_pass


def test_gazebo_active_cancel_pins_exact_runtime_evidence() -> None:
    """Active cancellation must run on an isolated canonical Gazebo stack."""
    matrix = load_runtime_matrix(_MATRIX_PATH)
    smoke = matrix.profile("gazebo_smoke_v2")
    profile = matrix.profile("gazebo_active_cancel_v2")

    assert profile.evidence_kind is EvidenceKind.SIMULATOR_MOTION
    assert profile.environment["ROS_DOMAIN_ID"] == "74"
    assert profile.requirements == smoke.requirements
    normalize = lambda commands: tuple(  # noqa: E731 - compact test projection.
        replace(command, name="normalized") for command in commands
    )
    assert normalize(profile.preflight) == normalize(smoke.preflight)
    assert normalize(profile.background) == normalize(smoke.background)
    smoke_shared_readiness = tuple(
        command
        for command in smoke.readiness
        if not any(
            command.name == f"gazebo_{suffix}"
            for suffix in _CLOCK_EVIDENCE_SUFFIXES
        )
    )
    assert normalize(profile.readiness) == normalize(smoke_shared_readiness)
    assert profile.validation[0].argv[:4] == (
        "ros2",
        "launch",
        "franka_motion",
        "franka_gazebo_active_cancel.launch.py",
    )
    assert profile.validation[0].stdout_regex == ("ACTIVE_CANCEL_PASS",)


def test_gazebo_backend_death_targets_one_v2_role_and_recovers() -> None:
    """Process-death evidence must isolate the V2 workflow and recover it."""
    profile = load_runtime_matrix(_MATRIX_PATH).profile(
        "gazebo_backend_death_v2"
    )

    assert profile.evidence_kind is EvidenceKind.DIAGNOSTIC_FAULT
    assert profile.environment["ROS_DOMAIN_ID"] == "75"
    assert profile.background[0].argv[3] == (
        "franka_gazebo_backend_death_validation.launch.py"
    )
    assert len(profile.faults) == 1
    fault = profile.faults[0]
    assert fault.fault_id == "action_backend_process"
    assert fault.inject[0].argv[-3:] == (
        "kill",
        "--role",
        "v2_transfer_workflow",
    )
    assert "PROCESS_DEATH_INJECTED" in fault.inject[0].stdout_regex[0]
    endpoint_absent = fault.observe[-1]
    assert endpoint_absent.name == "action_backend_endpoint_absent"
    assert endpoint_absent.argv[-2:] == (
        "info",
        "/transfer_object_sequence_v2",
    )
    assert endpoint_absent.retry_until_pass
    assert endpoint_absent.stdout_regex == (r"(?m)^Action servers: 0$",)
    assert fault.recover[0].argv[-3:] == (
        "restart",
        "--role",
        "v2_transfer_workflow",
    )
    assert profile.validation[0].name == "post_backend_restart_canonical_motion"
    matrix_source = _MATRIX_PATH.read_text(encoding="utf-8")
    assert "pkill" not in matrix_source
    assert "killall" not in matrix_source


@pytest.mark.parametrize(
    "profile_name",
    [
        "gazebo_smoke_v2",
        "gazebo_active_cancel_v2",
        "gazebo_soak_v2",
        "isaac_smoke_v2",
        "isaac_soak_v2",
    ],
)
def test_simulator_motion_waits_for_health_startup_grace(
    profile_name: str,
) -> None:
    """Action discovery must not race initial scene reconciliation."""
    profile = load_runtime_matrix(_MATRIX_PATH).profile(profile_name)
    health_probes = [
        command
        for command in profile.readiness
        if "/system_health_monitor/health" in command.argv
    ]

    assert len(health_probes) == 1
    health_probe = health_probes[0]
    assert profile.readiness[-1] == health_probe
    assert health_probe.retry_until_pass
    assert health_probe.timeout_sec >= 60
    assert any(
        "motion_ready: true" in expression
        for expression in health_probe.stdout_regex
    )
    assert any(
        "steady_uptime" in expression
        for expression in health_probe.stdout_regex
    )


def test_every_simulator_motion_profile_uses_canonical_transfer_contract() -> None:
    """Smoke and soak evidence must cover the maintained workflow, not legacy demos."""
    matrix = load_runtime_matrix(_MATRIX_PATH)

    expected_request_leaves = {
        "gazebo_smoke_v2": "franka_gazebo_move_cube.launch.py",
        "gazebo_active_cancel_v2": "franka_gazebo_active_cancel.launch.py",
        "gazebo_soak_v2": "franka_gazebo_move_cube_round_trip.launch.py",
        "isaac_smoke_v2": "franka_isaac_move_cube.launch.py",
        "isaac_soak_v2": "franka_isaac_move_cube_round_trip.launch.py",
    }
    for profile_name, request_leaf in expected_request_leaves.items():
        profile = matrix.profile(profile_name)
        assert profile.background[0].argv[2] == "franka_motion"
        assert any(
            argument.endswith("_moveit_table.launch.py")
            for argument in profile.background[0].argv
        )
        for command in profile.validation:
            assert command.argv[:4] == (
                "ros2",
                "launch",
                "franka_motion",
                request_leaf,
            )
        action_patterns = tuple(
            pattern
            for command in profile.readiness
            for pattern in command.stdout_regex
        )
        assert any(
            "/transfer_object_sequence_v2" in pattern
            for pattern in action_patterns
        )
        assert any(
            "/transfer_object_sequence_v2" in pattern
            for pattern in action_patterns
        )


def test_every_simulator_profile_has_a_fail_closed_version_probe() -> None:
    """Runtime evidence must retain an auditable simulator identity log."""
    matrix = load_runtime_matrix(_MATRIX_PATH)

    for profile in matrix.profiles.values():
        if profile.platform.value not in {"gazebo", "isaac"}:
            continue
        version_commands = [
            command for command in profile.preflight
            if "version" in command.name
        ]
        assert len(version_commands) == 1
        assert version_commands[0].stdout_regex


def test_gazebo_fault_profile_uses_five_ordered_recoverable_transactions() -> None:
    """Each dependency fault must block admission and have its own recovery."""
    matrix = load_runtime_matrix(_MATRIX_PATH)
    profile = matrix.profile("gazebo_health_fault_v2")

    assert profile.preflight[-1].argv == (
        "ros2",
        "pkg",
        "prefix",
        "manipulation",
    )
    assert tuple(fault.fault_id for fault in profile.faults) == (
        "controller",
        "joint_sensor",
        "moveit",
        "planning_scene",
        "tf",
    )
    controller = profile.faults[0]
    assert controller.inject[0].argv == (
        "ros2",
        "control",
        "switch_controllers",
        "--deactivate",
        "panda_arm_controller",
        "--strict",
    )
    assert controller.recover[0].argv == (
        "ros2",
        "control",
        "switch_controllers",
        "--activate",
        "panda_arm_controller",
        "--strict",
    )
    for fault in profile.faults:
        admission = next(
            command for command in fault.observe
            if command.name.endswith("motion_admission_blocked")
        )
        assert admission.argv[:4] == (
            "ros2",
            "launch",
            "franka_motion",
            "franka_gazebo_move_cube.launch.py",
        )
        assert admission.expected_return_codes == (1,)
        assert admission.stdout_regex == (
            "system health does not permit starting this transfer step",
        )
    assert profile.validation[0].name == "post_recovery_canonical_motion"
    assert profile.validation[0].expected_return_codes == (0,)


def test_fault_health_observations_pin_typed_wire_identity() -> None:
    """Health evidence must not infer a cause from motion_ready alone."""
    profile = load_runtime_matrix(_MATRIX_PATH).profile(
        "gazebo_health_fault_v2"
    )
    expected_wire_values = {
        "controller": (3, 2, (303,)),
        "joint_sensor": (1, 3, (102,)),
        "moveit": (5, 2, (500,)),
        "planning_scene": (7, 3, (703, 700)),
        "tf": (4, 2, (400,)),
    }

    for fault in profile.faults:
        patterns = "\n".join(fault.observe[0].stdout_regex)
        component_id, level, issue_codes = expected_wire_values[fault.fault_id]
        assert f"component_id: {component_id}" in patterns
        assert f"level: {level}" in patterns
        for issue_code in issue_codes:
            assert f"- {issue_code}" in patterns


def test_command_expansion_preserves_argv_boundaries() -> None:
    """Environment expansion must never reinterpret shell metacharacters."""
    command = CommandSpec(
        name="argv_contract",
        argv=(
            "program",
            "--config",
            "${PROFILE_PATH}",
            "literal;still-one-argument",
        ),
        timeout_sec=1.0,
    )

    argv = command.expanded_argv(
        {"PROFILE_PATH": "/path with spaces/profile.json"}
    )

    assert argv == (
        "program",
        "--config",
        "/path with spaces/profile.json",
        "literal;still-one-argument",
    )
    assert command.expected_return_codes == (0,)


def test_missing_environment_token_fails_before_process_start() -> None:
    """Unresolved placeholders must fail before any subprocess boundary."""
    command = CommandSpec(
        name="missing_environment",
        argv=("program", "${MISSING_PATH}"),
        timeout_sec=1.0,
    )

    with pytest.raises(ConfigError, match="MISSING_PATH"):
        command.expanded_argv({})


def test_empty_expanded_argument_fails_before_process_start() -> None:
    """An empty required path must not reach a child process."""
    command = CommandSpec(
        name="empty_environment",
        argv=("program", "${PROFILE_PATH}"),
        timeout_sec=1.0,
    )

    with pytest.raises(ConfigError, match="empty argument"):
        command.expanded_argv({"PROFILE_PATH": ""})


def test_string_command_is_rejected_instead_of_becoming_a_shell(tmp_path: Path) -> None:
    """A command string must never be accepted as implicit shell syntax."""
    matrix_value = _matrix_value()
    profile = matrix_value["profiles"]["gazebo_smoke_v2"]
    profile["preflight"][0]["argv"] = "gz sim --versions"

    with pytest.raises(ConfigError, match="must be an array"):
        load_runtime_matrix(_write_matrix(tmp_path, matrix_value))


def test_fault_profile_requires_recovery_verification(tmp_path: Path) -> None:
    """Fault evidence is incomplete without post-recovery observation."""
    matrix_value = _matrix_value()
    profile = matrix_value["profiles"]["gazebo_health_fault_v2"]
    del profile["faults"][0]["verify_recovered"]

    with pytest.raises(ConfigError, match="missing keys"):
        load_runtime_matrix(_write_matrix(tmp_path, matrix_value))


def test_moveit_fault_uses_canonical_motion_backend_parameter() -> None:
    """Keep the scenario fault wired to the generic health-monitor API."""
    profile = load_runtime_matrix(_MATRIX_PATH).profile(
        "gazebo_health_fault_v2"
    )
    fault = next(item for item in profile.faults if item.fault_id == "moveit")
    parameter = "fault_injection.force_motion_backend_unavailable"

    assert fault.inject[0].argv == (
        "ros2",
        "param",
        "set",
        "/system_health_monitor",
        parameter,
        "true",
    )
    assert fault.recover[0].argv == (
        "ros2",
        "param",
        "set",
        "/system_health_monitor",
        parameter,
        "false",
    )
    assert "force_move_group_unavailable" not in _MATRIX_PATH.read_text(
        encoding="utf-8"
    )


def test_v3_loader_rejects_removed_real_hardware_platform(
    tmp_path: Path,
) -> None:
    """The simulation-only matrix must not silently restore HIL profiles."""
    matrix_value = _matrix_value()
    profile = matrix_value["profiles"]["gazebo_smoke_v2"]
    profile["platform"] = "real_hardware"

    with pytest.raises(ConfigError, match="unsupported enum value"):
        load_runtime_matrix(_write_matrix(tmp_path, matrix_value))


def test_v3_loader_rejects_removed_safe_stop_field(tmp_path: Path) -> None:
    """HIL-only safe-stop commands are outside the simulation contract."""
    matrix_value = _matrix_value()
    matrix_value["profiles"]["gazebo_smoke_v2"]["safe_stop"] = {
        "name": "safe_stop",
        "argv": ["true"],
        "timeout_sec": 1,
    }

    with pytest.raises(ConfigError, match="unknown keys"):
        load_runtime_matrix(_write_matrix(tmp_path, matrix_value))


def test_unknown_schema_version_fails_closed(tmp_path: Path) -> None:
    """Unknown matrix semantics must not be interpreted as version three."""
    matrix_value = _matrix_value()
    matrix_value["schema_version"] = 4

    with pytest.raises(ConfigError, match="unsupported"):
        load_runtime_matrix(_write_matrix(tmp_path, matrix_value))


def test_runtime_matrix_conforms_to_its_json_schema() -> None:
    """The executable v3 matrix and documented schema must agree."""
    schema = json.loads(_SCHEMA_PATH.read_text(encoding="utf-8"))
    jsonschema.Draft202012Validator(schema).validate(_matrix_value())


@pytest.mark.parametrize(
    "mutation, expected_error",
    [
        (
            lambda value: value["profiles"]["gazebo_smoke_v2"][
                "preflight"
            ][0].update({"name": "команда"}),
            "invalid command name",
        ),
        (
            lambda value: value["profiles"]["gazebo_smoke_v2"].update(
                {"faults": []}
            ),
            "faults must not be empty",
        ),
    ],
)
def test_parser_and_schema_reject_the_same_boundary_cases(
    tmp_path: Path,
    mutation,
    expected_error: str,
) -> None:
    """Hand parsing must not accept values rejected by the public schema."""
    matrix_value = _matrix_value()
    mutation(matrix_value)
    schema = json.loads(_SCHEMA_PATH.read_text(encoding="utf-8"))

    with pytest.raises(jsonschema.ValidationError):
        jsonschema.Draft202012Validator(schema).validate(matrix_value)
    with pytest.raises(ConfigError, match=expected_error):
        load_runtime_matrix(_write_matrix(tmp_path, matrix_value))


def test_profile_deadline_covers_all_foreground_command_budgets(
    tmp_path: Path,
) -> None:
    """Retries and fault recovery cannot consume final validation time."""
    matrix_value = _matrix_value()
    profile = matrix_value["profiles"]["gazebo_smoke_v2"]
    profile["deadline_sec"] = 1
    profile["background"][0]["timeout_sec"] = 1

    with pytest.raises(ConfigError, match="foreground budget"):
        load_runtime_matrix(_write_matrix(tmp_path, matrix_value))


@pytest.mark.parametrize('profile_name', ['isaac_smoke_v2', 'isaac_soak_v2'])
def test_isaac_runtime_profiles_use_selected_python_for_preflight_and_bringup(profile_name):
    """The pip interpreter owns package discovery in both runtime stages."""
    profile = _matrix_value()['profiles'][profile_name]
    python_checks = [item for item in profile['preflight']
                     if item['name'].endswith(('_python', '_version'))]
    assert len(python_checks) == 2
    assert all(item['argv'][0] == '${FRANKA_ISAAC_PYTHON}' for item in python_checks)
    assert all('isaac_python:=${FRANKA_ISAAC_PYTHON}' in item['argv']
               for item in profile['background'])


@pytest.mark.parametrize('binary_layout', [False, True])
def test_isaac_version_preflight_reads_selected_package_layout(tmp_path, monkeypatch, capsys, binary_layout):
    """Both pip and explicit legacy roots expose VERSION without importing Kit."""
    from types import SimpleNamespace
    import importlib.util
    root = tmp_path / 'isaac'
    package = root / 'python_packages/isaacsim' if binary_layout else root
    package.mkdir(parents=True)
    (root / 'VERSION').write_text('6.2.0-rc.14')
    monkeypatch.setattr(importlib.util, 'find_spec', lambda name:
                        SimpleNamespace(origin=str(package / '__init__.py')))
    check = next(item for item in _matrix_value()['profiles']['isaac_smoke_v2']['preflight']
                 if item['name'] == 'isaac_version')
    exec(check['argv'][2], {})
    assert capsys.readouterr().out.strip() == '6.2.0-rc.14'
