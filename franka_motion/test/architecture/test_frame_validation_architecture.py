"""
Protect the single canonical frame-ID validation policy used by Franka code.

Responsibility:
- Prevent a private MTC frame validator from returning.
- Require active C++ and Python boundaries to use their shared policy helpers.

Does not own:
- TF graph semantics, runtime transform availability, or legacy cuRobo code.
"""

from pathlib import Path


_PACKAGE_ROOT = Path(__file__).resolve().parents[2]
_CPP_POLICY = 'manipulation/validation/frame_id.hpp'
_PYTHON_POLICY = 'franka_motion_runtime.frame_id'


def test_private_mtc_frame_validator_stays_removed() -> None:
    """MTC must consume the generic manipulation validation policy directly."""
    private_header = _PACKAGE_ROOT / "src" / "mtc" / "frame_validation.hpp"
    assert not private_header.exists()

    consumers = (
        "src/workflow_v2/process_config.cpp",
        "src/workflow_v2/provider_config_yaml.cpp",
        "src/workflow_v2/observation_providers.cpp",
    )
    for relative_path in consumers:
        source = (_PACKAGE_ROOT / relative_path).read_text(encoding="utf-8")
        assert _CPP_POLICY in source, relative_path
        assert "valid_frame_id(" not in source, relative_path


def test_active_python_frame_boundaries_use_one_helper() -> None:
    """Scenario and Isaac validators must not grow separate slash policies."""
    consumers = (
        "franka_motion_runtime/tasks/geometry.py",
        "franka_motion_runtime/simulation/isaac/runtime/validation.py",
    )
    forbidden_local_checks = (
        '.startswith("/")',
        '.endswith("/")',
        '"//" in',
    )
    for relative_path in consumers:
        source = (_PACKAGE_ROOT / relative_path).read_text(encoding="utf-8")
        assert _PYTHON_POLICY in source, relative_path
        for check in forbidden_local_checks:
            assert check not in source, relative_path
