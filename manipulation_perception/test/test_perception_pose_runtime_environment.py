"""
Test isolated runtime environments and worker-script resolution for pose backends.

Responsibility: Verify CUDA, EGL, conda, Python paths, and source-versus-install lookup.
"""

import os
from pathlib import Path
import sys


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from manipulation_perception.pose.pose_runtime_environment import (  # noqa: E402,I100,I101,I201
    make_gigapose_env,
    make_megapose_env,
    resolve_worker_script_path,
)


def _fake_python(tmp_path: Path) -> Path:
    python = tmp_path / 'env' / 'bin' / 'python'
    python.parent.mkdir(parents=True)
    python.write_text('# fake python\n')
    return python


def test_make_megapose_env_sets_cuda_egl_and_conda_paths(tmp_path: Path):
    python = _fake_python(tmp_path)
    data_dir = tmp_path / 'megapose_data'
    env = make_megapose_env(
        data_dir,
        str(python),
        base_env={'PATH': 'old-path', 'CUDA_VISIBLE_DEVICES': '2'},
    )

    assert env['MEGAPOSE_DATA_DIR'] == str(data_dir)
    assert env['MPLCONFIGDIR'] == str(data_dir / 'tmp' / 'matplotlib')
    assert env['PYTORCH_CUDA_ALLOC_CONF'] == 'expandable_segments:True'
    assert env['CUDA_VISIBLE_DEVICES'] == '2'
    assert env['EGL_VISIBLE_DEVICES'] == '2'
    assert env['CONDA_PREFIX'] == str(tmp_path / 'env')
    assert env['PATH'].startswith(str(python.parent) + os.pathsep)


def test_make_gigapose_env_prepends_repo_paths_and_keeps_existing_pythonpath(tmp_path: Path):
    python = _fake_python(tmp_path)
    data_dir = tmp_path / 'gigapose_data'
    repo_dir = tmp_path / 'gigaPose'
    env = make_gigapose_env(
        data_dir,
        repo_dir,
        str(python),
        base_env={'PATH': 'old-path', 'PYTHONPATH': '/existing',
                  'MEGAPOSE_DATA_DIR': '/shared/model/assets'},
    )

    assert env['GIGAPOSE_DATA_DIR'] == str(data_dir)
    assert env['MEGAPOSE_DATA_DIR'] == str(data_dir)
    assert env['TORCH_HOME'] == str(data_dir / 'torch')
    assert env['PYTHONPATH'].split(os.pathsep)[:3] == [
        str(repo_dir),
        str(repo_dir / 'src'),
        '/existing',
    ]
    assert env['CUDA_VISIBLE_DEVICES'] == '0'
    assert env['CONDA_PREFIX'] == str(tmp_path / 'env')


def test_resolve_worker_script_path_supports_source_and_installed_layouts(tmp_path: Path):
    source_file = tmp_path / 'perception' / 'megapose_detection_node.py'
    source_file.parent.mkdir()
    worker_dir = source_file.parent / 'workers'
    worker_dir.mkdir()
    source_worker = worker_dir / 'megapose_worker.py'
    source_worker.write_text('# worker\n')

    assert resolve_worker_script_path(source_file, 'megapose_worker.py') == source_worker

    installed_worker = source_file.parent / 'gigapose_worker.py'
    installed_worker.write_text('# installed worker\n')
    assert resolve_worker_script_path(source_file, 'gigapose_worker.py') == installed_worker
