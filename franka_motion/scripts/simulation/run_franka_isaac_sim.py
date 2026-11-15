#!/usr/bin/env python3
"""
Bootstrap the package-local Isaac runner from its stable share path.

Responsibility:
    Make the adjacent/source ``franka_motion_runtime`` package importable under
    Isaac's deliberately isolated Python environment, then invoke its runner.

Does not own:
    Isaac application setup, scene construction, or simulation shutdown.
"""

from __future__ import annotations

from pathlib import Path
import sys


def _add_runtime_package_root_to_path() -> None:
    """Find the canonical runtime in source or beside the installed runner."""
    for parent in Path(__file__).resolve().parents:
        runtime = (
            parent
            / "franka_motion_runtime"
            / "simulation"
            / "isaac"
            / "runner.py"
        )
        if runtime.is_file():
            parent_text = str(parent)
            if parent_text not in sys.path:
                sys.path.insert(0, parent_text)
            return
    raise RuntimeError("Cannot locate the franka_motion Isaac runtime package")


_add_runtime_package_root_to_path()

from franka_motion_runtime.simulation.isaac.runner import main  # noqa: E402


if __name__ == "__main__":
    main()
