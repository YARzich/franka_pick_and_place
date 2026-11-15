"""
Runtime environment helpers for MegaPose and GigaPose backends.

Responsibility:
- Build subprocess environment dictionaries for MegaPose/GigaPose workers and CLI.
- Resolve worker script locations for both source-tree and installed layouts.
- Preserve CUDA, EGL, CONDA_PREFIX and PYTHONPATH behavior from the node.

Does not own:
- Starting, reading or stopping subprocesses.
- Worker JSON request payloads.
- ROS logging or callbacks.
"""

from __future__ import annotations

from collections.abc import Mapping
import os
from pathlib import Path


def _base_env(base_env: Mapping[str, str] | None) -> dict[str, str]:
    return dict(os.environ if base_env is None else base_env)


def _apply_python_env(env: dict[str, str], python_path: str) -> None:
    python = Path(python_path).expanduser()
    if not python.is_file():
        return
    env_bin_dir = python.parent
    env_prefix = env_bin_dir.parent
    env.setdefault("CONDA_PREFIX", str(env_prefix))
    env["PATH"] = f"{env_bin_dir}{os.pathsep}{env.get('PATH', '')}"


def make_megapose_env(
    megapose_data_dir: Path,
    megapose_python: str,
    *,
    base_env: Mapping[str, str] | None = None,
) -> dict[str, str]:
    """Build a MegaPose child environment and create its writable cache path."""
    env = _base_env(base_env)
    matplotlib_cache_dir = megapose_data_dir / "tmp" / "matplotlib"
    matplotlib_cache_dir.mkdir(parents=True, exist_ok=True)
    env["MEGAPOSE_DATA_DIR"] = str(megapose_data_dir)
    env["MPLCONFIGDIR"] = str(matplotlib_cache_dir)
    env["PYTORCH_CUDA_ALLOC_CONF"] = "expandable_segments:True"
    env.setdefault("CUDA_VISIBLE_DEVICES", "0")
    env.setdefault("EGL_VISIBLE_DEVICES", env.get("CUDA_VISIBLE_DEVICES", "0"))
    _apply_python_env(env, megapose_python)
    return env


def make_gigapose_env(
    gigapose_data_dir: Path,
    gigapose_repo_dir: Path,
    gigapose_python: str,
    *,
    base_env: Mapping[str, str] | None = None,
) -> dict[str, str]:
    """
    Build a GigaPose child environment with repository-local imports.

    The returned dictionary is detached from ``base_env``. This function creates
    the Matplotlib cache directory but does not start a subprocess.
    """
    env = _base_env(base_env)
    matplotlib_cache_dir = gigapose_data_dir / "tmp" / "matplotlib"
    matplotlib_cache_dir.mkdir(parents=True, exist_ok=True)
    env["GIGAPOSE_DATA_DIR"] = str(gigapose_data_dir)
    # GigaPose imports MegaPose's config, whose import-time directories and
    # joblib cache must not fall back to repository/shared model storage.
    env["MEGAPOSE_DATA_DIR"] = str(gigapose_data_dir)
    env["TORCH_HOME"] = str(gigapose_data_dir / "torch")
    env["MPLCONFIGDIR"] = str(matplotlib_cache_dir)
    env["PYTORCH_CUDA_ALLOC_CONF"] = "expandable_segments:True"
    env.setdefault("CUDA_VISIBLE_DEVICES", "0")

    repo_paths = [str(gigapose_repo_dir), str(gigapose_repo_dir / "src")]
    existing_pythonpath = env.get("PYTHONPATH", "")
    env["PYTHONPATH"] = os.pathsep.join([*repo_paths, existing_pythonpath])
    _apply_python_env(env, gigapose_python)
    return env


def resolve_worker_script_path(current_file: Path, script_name: str) -> Path:
    """
    Resolve a worker in source-tree or flattened ROS install layout.

    Source workers live in `perception/workers`; CMake installs workers beside
    the node so an explicitly configured backend Python interpreter can execute
    them without installing this workspace package into that environment.
    """
    script_dir = current_file.resolve().parent
    candidates = [
        script_dir / script_name,
        script_dir / "workers" / script_name,
    ]
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    return candidates[-1]
