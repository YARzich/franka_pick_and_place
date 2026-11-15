"""
Test the architectural boundary of the read-only GPU perception ROS probe.

Responsibility: Pin installation, dependencies, CLI, and absence of production side effects.
"""

import ast
import importlib.util
from pathlib import Path

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PROBE = PACKAGE_ROOT / "scripts" / "validation" / "gpu_perception_evidence_probe.py"


def _load_probe_module():
    spec = importlib.util.spec_from_file_location("gpu_perception_evidence_probe_test", PROBE)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_probe_is_installed_with_schemas_and_dynamic_message_dependency() -> None:
    cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
    manifest = (PACKAGE_ROOT / "package.xml").read_text(encoding="utf-8")

    assert "scripts/validation/gpu_perception_evidence_probe.py" in cmake
    assert "install(DIRECTORY config" in cmake
    assert "<exec_depend>rosidl_runtime_py</exec_depend>" in manifest
    assert (PACKAGE_ROOT / "config" / "gpu_perception_probe_contract.schema.v1.json").is_file()
    assert (PACKAGE_ROOT / "config" / "gpu-perception-evidence.schema.v1.json").is_file()


def test_probe_is_subscription_only_and_has_no_scenario_defaults() -> None:
    source = PROBE.read_text(encoding="utf-8")
    tree = ast.parse(source)
    called_attributes = {
        node.func.attr
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
    }

    assert "create_subscription" in called_attributes
    assert "create_publisher" not in called_attributes
    assert "create_service" not in called_attributes
    assert "create_client" not in called_attributes
    assert "ActionServer" not in source
    assert "/table_stereo" not in source
    assert "panda_" not in source
    assert "/home/" not in source
    assert "curl" not in source
    assert "wget" not in source


def test_probe_cli_requires_revision_contract_artifact_and_explicit_assets() -> None:
    source = PROBE.read_text(encoding="utf-8")

    for argument in (
        "--contract",
        "--artifact-dir",
        "--run-token",
        "--revision",
        "--asset",
    ):
        assert argument in source
    assert "capture_payload_digest" in source
    assert source.count("serialize_message(message)") == 1


def test_probe_reports_stable_post_write_marker_and_exit_code(capsys) -> None:
    probe = _load_probe_module()
    artifact = Path("/tmp/evidence/gpu-perception-evidence.v1.json")

    assert probe._report_result("pass", artifact) == 0
    assert capsys.readouterr().out == f"GPU_PERCEPTION_PASS artifact={artifact}\n"
    assert probe._report_result("fail", artifact) == 1
    assert capsys.readouterr().out == f"GPU_PERCEPTION_FAIL artifact={artifact}\n"


def test_probe_converts_ros_byte_diagnostic_level_to_int() -> None:
    """A received DiagnosticStatus.level is a ROS ``byte`` (bytes), not int."""
    probe = _load_probe_module()

    assert probe._diagnostic_level_to_int(b"\x00") == 0
    assert probe._diagnostic_level_to_int(b"\x02") == 2
    assert probe._diagnostic_level_to_int(bytearray(b"\x01")) == 1
    assert probe._diagnostic_level_to_int(3) == 3


def test_probe_rejects_working_directory_dependent_paths() -> None:
    probe = _load_probe_module()

    with pytest.raises(ValueError, match="--contract"):
        probe._require_absolute_cli_paths(Path("contract.json"), Path("/tmp/artifact"))
    with pytest.raises(ValueError, match="--artifact-dir"):
        probe._require_absolute_cli_paths(Path("/tmp/contract.json"), Path("artifact"))
