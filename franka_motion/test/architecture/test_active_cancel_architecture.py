"""
Test ownership boundaries of the active-cancel runtime evidence path.

Responsibility:
- Keep scenario geometry in the Franka leaf and transport code reusable.
- Pin the thin probe, pure validator, installed executable, and public cancel API.

Does not own:
- Simulator execution, action semantics, or runtime evidence outcomes.
"""

import ast
from pathlib import Path


_PACKAGE_ROOT = Path(__file__).resolve().parents[2]
_TASK_ROOT = _PACKAGE_ROOT / "franka_motion_runtime" / "tasks"


def _source(name: str) -> str:
    """Read one task module as the canonical architecture source."""
    return (_TASK_ROOT / name).read_text(encoding="utf-8")


def test_active_cancel_modules_are_bounded_by_single_responsibility() -> None:
    """Action orchestration, ROS readback, and pure policy remain separate."""
    module_names = (
        "active_cancel_probe.py",
        "active_cancel_motion.py",
        "active_cancel_evidence.py",
        "active_cancel_validation.py",
    )
    for module_name in module_names:
        assert len(_source(module_name).splitlines()) <= 300, module_name

    probe = _source("active_cancel_probe.py")
    assert "load_sequence_client_inputs" in probe
    assert "build_transfer_sequence_goal" in probe
    assert "cancel_exact_goal" in probe
    assert "wait_for_cleanup_evidence" in probe
    validation = _source("active_cancel_validation.py")
    assert "rclpy" not in validation
    assert "time.monotonic" not in validation
    evidence = _source("active_cancel_evidence.py")
    assert "Parameter.Type.STRING_ARRAY" in evidence
    assert "if not persistent_ids or not all(" in evidence


def test_generic_active_cancel_runtime_contains_no_franka_geometry() -> None:
    """Only the scenario leaf may know Panda frames, cube, or table fixtures."""
    generic_source = "\n".join(
        _source(name)
        for name in (
            "active_cancel_client.py",
            "active_cancel_probe.py",
            "active_cancel_motion.py",
            "active_cancel_evidence.py",
            "active_cancel_validation.py",
        )
    )
    for scenario_token in ("panda_link0", "panda_arm_controller", '"cube"', '"table"'):
        assert scenario_token not in generic_source

    leaf = (
        _PACKAGE_ROOT / "launch" / "franka_gazebo_active_cancel.launch.py"
    ).read_text(encoding="utf-8")
    assert 'object_id="cube"' in leaf
    assert 'frame_id="panda_link0"' in leaf
    assert 'PERSISTENT_WORLD_OBJECT_IDS = ("table",)' in leaf


def test_active_cancel_probe_is_installed_and_cancel_api_is_public() -> None:
    """The launch leaf must resolve to an installed, non-private cancel path."""
    install_source = (
        _PACKAGE_ROOT / "cmake" / "FrankaMotionRuntimeInstall.cmake"
    ).read_text(encoding="utf-8")
    executable = "franka_transfer_sequence_active_cancel_probe"
    assert f"scripts/tasks/{executable}" in install_source

    cancel_tree = ast.parse(_source("sequence_client_cancel.py"))
    function_names = {
        node.name
        for node in cancel_tree.body
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
    }
    assert "cancel_exact_goal" in function_names
    assert "_cancel_exact_goal" not in function_names
