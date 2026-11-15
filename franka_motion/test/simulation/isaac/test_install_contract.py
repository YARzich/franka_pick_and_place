"""
Test the installed Isaac runner and compatibility import surface.

Responsibility:
- Verify the stable runner path works with an isolated Python environment.
- Verify legacy imports resolve to the package-local canonical implementation.
"""

import os
from pathlib import Path
import subprocess
import sys

from ament_index_python.packages import get_package_share_directory

from franka_motion_runtime.simulation.isaac.runtime.config import (
    parse_runner_config as canonical_parse_runner_config,
)
from isaac_runtime.config import parse_runner_config as legacy_parse_runner_config


def test_installed_runner_help_works_with_clean_pythonpath(tmp_path: Path) -> None:
    """Isaac's isolated interpreter must find helpers beside the runner."""
    package_share = Path(get_package_share_directory("franka_motion"))
    runner = package_share / "scripts" / "run_franka_isaac_sim.py"
    adjacent_runtime = (
        runner.parent
        / "franka_motion_runtime"
        / "simulation"
        / "isaac"
        / "runtime"
    )
    adjacent_frame_policy = (
        runner.parent
        / "franka_motion_runtime"
        / "frame_id.py"
    )
    environment = {
        "HOME": os.environ.get("HOME", str(tmp_path)),
        "LANG": os.environ.get("LANG", "C.UTF-8"),
        "PATH": os.environ.get("PATH", ""),
        "PYTHONDONTWRITEBYTECODE": "1",
        "PYTHONNOUSERSITE": "1",
        "PYTHONPATH": "",
    }

    result = subprocess.run(
        [sys.executable, str(runner), "--help"],
        cwd=tmp_path,
        env=environment,
        check=False,
        capture_output=True,
        text=True,
    )

    assert runner.is_file()
    assert os.access(runner, os.X_OK)
    assert adjacent_runtime.is_dir()
    assert adjacent_frame_policy.is_file()
    assert result.returncode == 0, result.stderr
    assert "--robot-usd" in result.stdout
    assert "--stereo-left-frame-id" in result.stdout


def test_legacy_runtime_import_reexports_canonical_symbol() -> None:
    """Historical imports must not retain a second runtime implementation."""
    assert legacy_parse_runner_config is canonical_parse_runner_config
