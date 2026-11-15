"""Responsibility: Verify portable skill paths, explicit GPU inputs and owned cleanup."""

import importlib.util
import json
import os
from pathlib import Path
import shlex
import shutil
import signal
import subprocess

import pytest


_PACKAGE_ROOT = Path(__file__).resolve().parents[2]
_FAMILY_ROOT = _PACKAGE_ROOT.parent
_REPOSITORY_ROOT = (
    _FAMILY_ROOT
    if (_FAMILY_ROOT / '.agents/skills/validate-franka-manipulation/SKILL.md').is_file()
    else _FAMILY_ROOT.parent
)
_SKILLS_ROOT = _REPOSITORY_ROOT / '.agents' / 'skills'
_VALIDATION = Path('.agents/skills/validate-franka-manipulation/scripts')
_DIAGNOSIS = Path('.agents/skills/diagnose-franka-perception/scripts')


def _checkout(tmp_path, *, monorepo=False, inside_workspace=False):
    """Copy only portable tooling and minimal package markers to a new location."""
    parent = tmp_path / 'workspace with spaces' / 'src' if inside_workspace else tmp_path
    root = parent / 'checkout with spaces'
    for skill in ('validate-franka-manipulation', 'diagnose-franka-perception'):
        shutil.copytree(_SKILLS_ROOT / skill, root / '.agents' / 'skills' / skill)
    family = root / 'manipulation' if monorepo else root
    package = family / 'franka_motion'
    matrix = package / 'config/runtime_validation/runtime_matrix.v3.json'
    matrix.parent.mkdir(parents=True)
    matrix.write_text(json.dumps({'profiles': {
        'gazebo_smoke_v2': {'platform': 'gazebo', 'evidence_kind': 'smoke'},
    }}), encoding='utf-8')
    (package / 'package.xml').write_text('<package/>', encoding='utf-8')
    return root, family


def _environment(tmp_path, **values):
    """Avoid accidentally testing against the developer's ROS or GPU environment."""
    return {'PATH': '/usr/bin:/bin', 'HOME': str(tmp_path), 'LANG': 'C.UTF-8', **values}


def _run(script, environment, *args):
    return subprocess.run(
        ['/bin/bash', str(script), *args], env=environment,
        text=True, capture_output=True, timeout=10, check=False,
    )


def _helper(root, environment, command):
    script = root / _VALIDATION / 'runtime_environment.sh'
    return subprocess.run(
        ['/bin/bash', '--noprofile', '--norc', '-c',
         f'set -eu; source {shlex.quote(str(script))}; resolve_franka_paths; {command}'],
        env=environment, text=True, capture_output=True, timeout=10, check=False,
    )


@pytest.mark.parametrize('monorepo', [False, True])
def test_profile_list_uses_the_checkout_without_ros(tmp_path, monorepo):
    """A fresh standalone or monorepo checkout can list its own profiles offline."""
    root, _ = _checkout(tmp_path, monorepo=monorepo)
    result = _run(
        root / _VALIDATION / 'run_profile.sh',
        _environment(tmp_path, ROS_SETUP_FILE='/absent/setup.bash'), '--list',
    )
    assert result.returncode == 0, result.stderr
    assert result.stdout == 'gazebo_smoke_v2\tgazebo\tsmoke\n'


@pytest.mark.parametrize('monorepo', [False, True])
def test_workspace_detection_uses_src_ancestor(tmp_path, monorepo):
    """Both repository layouts preserve workspace discovery, including spaces."""
    root, family = _checkout(tmp_path, monorepo=monorepo, inside_workspace=True)
    result = _helper(root, _environment(tmp_path), 'printf "%s\\n%s" "$FAMILY_ROOT" "$WS_DIR"')
    assert result.returncode == 0, result.stderr
    assert result.stdout.splitlines() == [str(family), str(root.parent.parent)]


def test_build_outside_workspace_requires_explicit_workspace(tmp_path):
    """A detached checkout must not guess a parent directory and build it."""
    root, _ = _checkout(tmp_path)
    result = _run(root / _VALIDATION / 'run_profile.sh', _environment(tmp_path))
    assert result.returncode == 2
    assert 'Set WS_DIR' in result.stderr


def test_explicit_workspace_takes_priority(tmp_path):
    root, _ = _checkout(tmp_path, inside_workspace=True)
    workspace = tmp_path / 'selected workspace'
    workspace.mkdir()
    result = _helper(root, _environment(tmp_path, WS_DIR=str(workspace)), 'printf "%s" "$WS_DIR"')
    assert result.returncode == 0, result.stderr
    assert result.stdout == str(workspace)


@pytest.mark.parametrize('inside_workspace', [False, True])
def test_build_uses_only_selected_underlay_and_includes_dependencies(tmp_path, inside_workspace):
    """An already sourced overlay must not leak into the build subprocess."""
    root, family = _checkout(tmp_path, inside_workspace=inside_workspace)
    workspace = root.parent.parent if inside_workspace else tmp_path / 'build workspace'
    (workspace / 'src').mkdir(parents=True, exist_ok=True)
    setup = tmp_path / 'underlay.bash'
    setup.write_text(
        '[[ -z "${AMENT_PREFIX_PATH:-}" && -z "${PYTHONPATH:-}" ]] || exit 98\n'
        'colcon() { printf "%s\\n" "$@"; exit 77; }\n',
        encoding='utf-8',
    )
    result = _run(root / _VALIDATION / 'run_profile.sh', _environment(
        tmp_path, ROS_SETUP_FILE=str(setup), WS_DIR=str(workspace),
        AMENT_PREFIX_PATH='/unrelated/overlay', PYTHONPATH='/unrelated/python',
    ))
    assert result.returncode == 77, result.stderr
    source_paths = [str(workspace / 'src')] if inside_workspace else [str(family), str(workspace / 'src')]
    assert result.stdout.splitlines() == [
        'build', '--base-paths', *source_paths, '--packages-up-to', 'franka_motion', '--cmake-args',
        '-DCMAKE_EXPORT_COMPILE_COMMANDS=ON', '-G', 'Ninja',
    ]


def test_unbuilt_workspace_does_not_fall_back_to_another_overlay(tmp_path):
    """A valid underlay alone cannot make a selected, unbuilt workspace ready."""
    root, _ = _checkout(tmp_path, inside_workspace=True)
    setup = tmp_path / 'setup.bash'
    setup.write_text('export UNDERLAY_LOADED=1\n', encoding='utf-8')
    result = _helper(root, _environment(tmp_path, ROS_SETUP_FILE=str(setup)), 'source_franka_setup')
    assert result.returncode == 2
    assert 'Workspace is not built' in result.stderr


def test_wrong_installed_package_is_rejected(tmp_path):
    root, _ = _checkout(tmp_path, inside_workspace=True)
    result = _helper(
        root, _environment(tmp_path),
        'ros2() { printf "%s\\n" /unrelated/install/franka_motion/share/franka_motion; }; '
        'franka_package_share',
    )
    assert result.returncode == 2
    assert 'outside the selected workspace' in result.stderr


def test_gpu_profile_reports_missing_inputs_before_loading_ros(tmp_path):
    root, _ = _checkout(tmp_path)
    result = _run(root / _DIAGNOSIS / 'run_gazebo_evidence.sh', _environment(tmp_path))
    assert result.returncode == 2
    assert 'FRANKA_GPU_YOLO_PYTHON' in result.stderr
    assert 'FRANKA_GPU_GIGAPOSE_DATA_DIR' in result.stderr
    assert '/opt/ros' not in result.stderr


def test_gpu_profile_uses_explicit_directories_and_checkout_assets(tmp_path):
    root, family = _checkout(tmp_path)
    runner = root / _VALIDATION / 'run_profile.sh'
    runner.write_text('#!/usr/bin/env bash\nenv\n', encoding='utf-8')
    bindings = {
        name: str(tmp_path / name.lower()) for name in (
            'FRANKA_GPU_ESS_ENGINE', 'FRANKA_GPU_YOLO_PYTHON',
            'FRANKA_GPU_MEGAPOSE_PYTHON', 'FRANKA_GPU_MEGAPOSE_DATA_DIR',
            'FRANKA_GPU_GIGAPOSE_PYTHON', 'FRANKA_GPU_GIGAPOSE_REPO_DIR',
            'FRANKA_GPU_DINOV2_REPO_DIR', 'FRANKA_GPU_GIGAPOSE_DATA_DIR',
            'FRANKA_GPU_FOUNDATIONPOSE_REFINE_MODEL', 'FRANKA_GPU_FOUNDATIONPOSE_REFINE_ENGINE',
            'FRANKA_GPU_FOUNDATIONPOSE_SCORE_MODEL', 'FRANKA_GPU_FOUNDATIONPOSE_SCORE_ENGINE',
        )
    }
    bindings['FRANKA_GPU_MEGAPOSE_REFINER_CHECKPOINT'] = '/explicit/refiner.pth'
    result = _run(root / _DIAGNOSIS / 'run_gazebo_evidence.sh', _environment(tmp_path, **bindings))
    assert result.returncode == 0, result.stderr
    actual = dict(line.split('=', 1) for line in result.stdout.splitlines() if '=' in line)
    assert actual['FRANKA_GPU_GIGAPOSE_CHECKPOINT'] == (
        bindings['FRANKA_GPU_GIGAPOSE_DATA_DIR'] + '/pretrained/gigaPose_v1.ckpt'
    )
    assert actual['FRANKA_GPU_MEGAPOSE_REFINER_CHECKPOINT'] == '/explicit/refiner.pth'
    assert actual['FRANKA_GPU_CUBE_MESH'] == str(family / 'franka_motion/models/foundationpose/cube_5cm.obj')


@pytest.fixture
def cleanup_module():
    """Load the optional runtime test without starting any simulator or ROS node."""
    pytest.importorskip('rclpy')
    spec = importlib.util.spec_from_file_location(
        'franka_shutdown_cleanup_under_test', _PACKAGE_ROOT / 'test/test_shutdown_cleanup.py',
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_shutdown_failure_never_kills_processes_by_name(cleanup_module, monkeypatch):
    """An owned scenario failure must not trigger a system-wide pkill fallback."""
    calls = []
    monkeypatch.setattr(cleanup_module, '_assert_no_residual_processes', lambda: None)

    def fail(_command):
        raise AssertionError('owned launch failed')

    monkeypatch.setattr(cleanup_module, '_run_launch_until_ready_then_shutdown', fail)
    monkeypatch.setattr(subprocess, 'run', lambda *args, **kwargs: calls.append(args))
    with pytest.raises(AssertionError, match='owned launch failed'):
        cleanup_module.test_gazebo_moveit_ctrl_c_shutdown_can_run_twice()
    assert calls == []


def test_cleanup_signals_and_reaps_only_its_owned_process(cleanup_module, monkeypatch):
    calls = []

    class Process:
        pid = 12345

        def poll(self):
            return None

        def wait(self, timeout):
            calls.append(('wait', timeout))
            if len([entry for entry in calls if entry[0] == 'wait']) == 1:
                raise subprocess.TimeoutExpired('owned', timeout)
            return -signal.SIGKILL

    monkeypatch.setattr(os, 'killpg', lambda pid, sig: calls.append(('signal', pid, sig)))
    launch = cleanup_module._LaunchRun(['unused'])
    launch._process = Process()
    launch.terminate_for_cleanup()
    assert calls == [
        ('signal', 12345, signal.SIGTERM), ('wait', 5.0),
        ('signal', 12345, signal.SIGKILL), ('wait', 5.0),
    ]
