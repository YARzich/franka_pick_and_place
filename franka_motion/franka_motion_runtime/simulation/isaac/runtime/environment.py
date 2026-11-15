"""
Environment preparation for the Isaac Sim runtime entrypoint.

Responsibility:
- Keep system ROS 2 Python paths away from Isaac Sim's Python process.
- Bootstrap Isaac Sim Python paths and ROS 2 bridge library paths.
- Prepare small runtime environment cleanups before SimulationApp import.

Does not own:
- Launch-time environment actions.
- Isaac scene construction or ROS graph creation.
"""

from __future__ import annotations

import importlib.util
import os
from pathlib import Path
import sys
from typing import MutableMapping, MutableSequence, Sequence


ROS_PREFIX_VARIABLES = ("AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH", "ROS_PACKAGE_PATH")


def sanitize_ros_environment(
    *,
    environ: MutableMapping[str, str] | None = None,
    sys_path: MutableSequence[str] | None = None,
) -> None:
    """Remove system/workspace ROS paths that conflict with Isaac's bridge."""
    env = os.environ if environ is None else environ
    path_list = sys.path if sys_path is None else sys_path

    prefixes = ["/opt/ros"]
    for var in ROS_PREFIX_VARIABLES:
        prefixes.extend(part for part in env.pop(var, "").split(os.pathsep) if part)
    for name in ("LD_LIBRARY_PATH", "PYTHONPATH"):
        env[name] = os.pathsep.join(
            part for part in env.get(name, "").split(os.pathsep)
            if part and not _under_any_prefix(part, prefixes)
        )
    path_list[:] = [path for path in path_list if not _under_any_prefix(path, prefixes)]


def resolve_isaac_root(override: Path | None = None) -> Path:
    """Discover resources in the selected Python, without importing the simulator."""
    if override is not None:
        return override.expanduser().resolve()
    spec = importlib.util.find_spec("isaacsim")
    if spec is None or spec.origin is None:
        raise RuntimeError(
            "Isaac Sim is unavailable in this Python. Set isaac_python or "
            "FRANKA_ISAAC_PYTHON to its environment's interpreter."
        )
    package_root = Path(spec.origin).resolve().parent
    if package_root.parent.name == "python_packages":
        return package_root.parent.parent
    return package_root


def bootstrap_isaacsim_python(
    root_path: Path,
    *,
    sys_path: MutableSequence[str] | None = None,
) -> None:
    """Expose only the binary distribution's package; pip handles its own bootstrap."""
    path_list = sys.path if sys_path is None else sys_path
    for package_dir in isaac_python_candidate_paths(root_path):
        if package_dir.is_dir() and str(package_dir) not in path_list:
            path_list.append(str(package_dir))


def isaac_python_candidate_paths(root_path: Path) -> list[Path]:
    """Return the binary package entrypoint; Isaac owns extension/Kit bootstrapping."""
    return [root_path / "python_packages"]


def disable_warp(*, environ: MutableMapping[str, str] | None = None) -> None:
    """Disable Warp while preserving any pre-disabled Isaac extensions."""
    env = os.environ if environ is None else environ
    disabled = env.get("CARB_DISABLE_EXTENSIONS", "")
    disabled_set = {ext.strip() for ext in disabled.replace(";", ",").split(",") if ext.strip()}
    if "omni.warp" not in disabled_set:
        disabled_set.add("omni.warp")
        env["CARB_DISABLE_EXTENSIONS"] = ",".join(sorted(disabled_set))


def cleanup_previous_minidumps(root_path: Path) -> None:
    """Remove stale crash dumps that can confuse the next simulator run."""
    crash_root = root_path / "kit" / "data" / "Kit" / "Isaac-Sim Python"
    if not crash_root.exists():
        return
    for minidump_file in crash_root.rglob("*.dmp"):
        try:
            minidump_file.unlink()
        except OSError:
            continue


def ensure_ros_bridge_environment(
    root_path: Path,
    ros_distro: str,
    *,
    environ: MutableMapping[str, str] | None = None,
    sys_path: MutableSequence[str] | None = None,
) -> None:
    """Prepend Isaac's Jazzy bridge libraries and Python modules."""
    env = os.environ if environ is None else environ
    path_list = sys.path if sys_path is None else sys_path
    env.setdefault("ROS_DISTRO", ros_distro)
    env.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    env.setdefault("PYTHONNOUSERSITE", "1")

    bridge_root = root_path / "exts" / "isaacsim.ros2.core" / ros_distro
    if not bridge_root.is_dir():
        bridge_root = root_path / "exts" / "isaacsim.ros2.bridge" / ros_distro
    lib_path = bridge_root / "lib"
    rclpy_path = bridge_root / "rclpy"

    if lib_path.exists():
        prepend_env_path(env, "LD_LIBRARY_PATH", lib_path)
    if rclpy_path.exists():
        prepend_env_path(env, "PYTHONPATH", rclpy_path)
        _prepend_unique(path_list, str(rclpy_path))


def reexec_with_clean_env(
    root_path: Path,
    ros_distro: str,
    *,
    executable: str | None = None,
    argv: Sequence[str] | None = None,
    environ: MutableMapping[str, str] | None = None,
    sys_path: MutableSequence[str] | None = None,
    exec_func=None,
) -> bool:
    """Re-exec once with an Isaac-safe environment; report whether attempted."""
    env = os.environ if environ is None else environ
    if env.get("ISAAC_CLEAN_ENV_REEXEC") == "1":
        return False

    sanitize_ros_environment(environ=env, sys_path=sys_path)
    ensure_ros_bridge_environment(root_path, ros_distro, environ=env, sys_path=sys_path)
    env["ISAAC_CLEAN_ENV_REEXEC"] = "1"
    run_exec = os.execvpe if exec_func is None else exec_func
    run_exec(executable or sys.executable, [executable or sys.executable, *(argv or sys.argv)], env)
    return True


def prepend_env_path(env: MutableMapping[str, str], name: str, value: Path) -> None:
    """Prepend one unique filesystem entry to a path-like variable."""
    parts = [part for part in env.get(name, "").split(os.pathsep) if part]
    value_str = str(value)
    if value_str not in parts:
        parts.insert(0, value_str)
        env[name] = os.pathsep.join(parts)


def _under_any_prefix(value: str, prefixes: Sequence[str]) -> bool:
    path = os.path.abspath(value)
    return any(path == os.path.abspath(prefix) or
               path.startswith(os.path.abspath(prefix).rstrip(os.sep) + os.sep)
               for prefix in prefixes)


def _prepend_unique(path_list: MutableSequence[str], value: str) -> None:
    if value not in path_list:
        path_list.insert(0, value)
