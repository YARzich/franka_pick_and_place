#!/usr/bin/env python3


"""
Exec the canonical perception entrypoint behind a legacy package name.

Responsibility:
    Preserve an installed legacy executable route by resolving and replacing
    this process with the corresponding canonical perception executable.

Does not own:
    Perception configuration, ROS entities, or backend inference.

The wrapper intentionally imports no perception or ML backend module. `execv`
replaces this process, preserving signal delivery and the canonical exit code.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

from ament_index_python.packages import get_package_prefix


_CANONICAL_PACKAGE = "manipulation_perception"
_ENTRYPOINTS = frozenset(
    {
        "disparity_frame_fix_node.py",
        "foundationpose_depth_clean_node.py",
        "foundationpose_detection_to_pose_node.py",
        "foundationpose_input_frame_fix_node.py",
        "gigapose_refiner_worker.py",
        "gigapose_worker.py",
        "megapose_detection_node.py",
        "megapose_worker.py",
        "stereo_sgbm_pointcloud_node.py",
        "yolo_visualizer_node.py",
    }
)


def _canonical_executable(wrapper_name: str) -> Path:
    """Return the installed canonical executable for an allowed facade name."""
    if wrapper_name not in _ENTRYPOINTS:
        raise ValueError(f"unsupported franka_motion perception facade: {wrapper_name}")
    prefix = Path(get_package_prefix(_CANONICAL_PACKAGE))
    return prefix / "lib" / _CANONICAL_PACKAGE / wrapper_name


def main() -> int:
    """Replace the compatibility process with its canonical executable."""
    wrapper_name = Path(sys.argv[0]).name
    try:
        executable = _canonical_executable(wrapper_name)
        os.execv(executable, sys.argv)
    except (LookupError, OSError, ValueError) as error:
        print(
            f"franka_motion compatibility facade could not exec {wrapper_name} "
            f"from {_CANONICAL_PACKAGE}: {error}",
            file=sys.stderr,
        )
    return 127


if __name__ == "__main__":
    raise SystemExit(main())
