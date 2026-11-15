"""
Test construction of the isolated Isaac Sim ROS bridge environment.

Responsibility: Verify path sanitization, bridge injection, re-exec, and extension disabling.
"""

import os
from pathlib import Path

from franka_motion_runtime.simulation.isaac.runtime.environment import (
    disable_warp,
    ensure_ros_bridge_environment,
    reexec_with_clean_env,
    sanitize_ros_environment,
)


def test_sanitize_ros_environment_removes_system_ros_paths():
    env = {
        'AMENT_PREFIX_PATH': '/opt/ros/jazzy',
        'COLCON_PREFIX_PATH': '/workspace/ros/install',
        'ROS_PACKAGE_PATH': '/some/pkg',
        'LD_LIBRARY_PATH': os.pathsep.join(['/keep/lib', '/opt/ros/jazzy/lib', '/workspace/ros/install/lib']),
        'PYTHONPATH': os.pathsep.join(['/keep/python', '/opt/ros/jazzy/lib/python3.12/site-packages']),
    }
    sys_path = ['/keep/site', '/opt/ros/jazzy/lib/python3.12/site-packages', '/workspace/ros/install/x']

    sanitize_ros_environment(environ=env, sys_path=sys_path)

    assert 'AMENT_PREFIX_PATH' not in env
    assert 'COLCON_PREFIX_PATH' not in env
    assert 'ROS_PACKAGE_PATH' not in env
    assert env['LD_LIBRARY_PATH'] == '/keep/lib'
    assert env['PYTHONPATH'] == '/keep/python'
    assert sys_path == ['/keep/site']


def test_ensure_ros_bridge_environment_prepends_existing_bridge_paths(tmp_path: Path):
    root = tmp_path / 'isaac'
    bridge_root = root / 'exts' / 'isaacsim.ros2.bridge' / 'jazzy'
    lib_path = bridge_root / 'lib'
    rclpy_path = bridge_root / 'rclpy'
    lib_path.mkdir(parents=True)
    rclpy_path.mkdir()
    env = {'LD_LIBRARY_PATH': '/old/lib', 'PYTHONPATH': '/old/python'}
    sys_path = ['/old/site', '/usr/lib/python3.12/site-packages']

    ensure_ros_bridge_environment(root, 'jazzy', environ=env, sys_path=sys_path)

    assert env['ROS_DISTRO'] == 'jazzy'
    assert env['RMW_IMPLEMENTATION'] == 'rmw_fastrtps_cpp'
    assert env['PYTHONNOUSERSITE'] == '1'
    assert env['LD_LIBRARY_PATH'].split(os.pathsep)[:2] == [str(lib_path), '/old/lib']
    assert env['PYTHONPATH'].split(os.pathsep)[:2] == [str(rclpy_path), '/old/python']
    assert sys_path == [str(rclpy_path), '/old/site', '/usr/lib/python3.12/site-packages']


def test_reexec_with_clean_env_prepares_env_before_exec(tmp_path: Path):
    root = tmp_path / 'isaac'
    (root / 'exts' / 'isaacsim.ros2.bridge' / 'jazzy' / 'lib').mkdir(parents=True)
    (root / 'exts' / 'isaacsim.ros2.bridge' / 'jazzy' / 'rclpy').mkdir()
    env = {
        'LD_LIBRARY_PATH': os.pathsep.join(['/opt/ros/jazzy/lib', '/keep/lib']),
        'PYTHONPATH': '/opt/ros/jazzy/lib/python3.12/site-packages',
    }
    sys_path = ['/opt/ros/jazzy/lib/python3.12/site-packages', '/keep/site']
    calls = []

    def fake_exec(executable, argv, exec_env):
        calls.append((executable, argv, dict(exec_env)))

    did_reexec = reexec_with_clean_env(
        root,
        'jazzy',
        executable='/isaac/python.sh',
        argv=['runner.py', '--headless'],
        environ=env,
        sys_path=sys_path,
        exec_func=fake_exec,
    )

    assert did_reexec is True
    assert calls[0][0] == '/isaac/python.sh'
    assert calls[0][1] == ['/isaac/python.sh', 'runner.py', '--headless']
    assert calls[0][2]['ISAAC_CLEAN_ENV_REEXEC'] == '1'
    assert '/opt/ros' not in calls[0][2]['LD_LIBRARY_PATH']
    assert sys_path[0].endswith('isaacsim.ros2.bridge/jazzy/rclpy')


def test_disable_warp_preserves_existing_disabled_extensions():
    env = {'CARB_DISABLE_EXTENSIONS': 'omni.foo;omni.bar'}

    disable_warp(environ=env)

    assert env['CARB_DISABLE_EXTENSIONS'].split(',') == ['omni.bar', 'omni.foo', 'omni.warp']


def test_sanitize_preserves_python312_and_removes_arbitrarily_named_ros_prefix():
    paths = ['/usr/lib/python3.12', '/venv/lib/python3.12/site-packages',
             '/deployment/install/pkg/lib/python3.12/site-packages', '/deployment/install-other']
    env = {'AMENT_PREFIX_PATH': '/deployment/install/pkg', 'PYTHONPATH': os.pathsep.join(paths)}
    sanitize_ros_environment(environ=env, sys_path=paths)
    assert paths == ['/usr/lib/python3.12', '/venv/lib/python3.12/site-packages',
                     '/deployment/install-other']
    assert env['PYTHONPATH'].split(os.pathsep) == paths


def test_pip_root_discovery_uses_package_not_executable(monkeypatch):
    from types import SimpleNamespace
    from franka_motion_runtime.simulation.isaac.runtime import environment
    monkeypatch.setattr(environment.importlib.util, 'find_spec',
                        lambda name: SimpleNamespace(origin='/venv/lib/python3.12/site-packages/isaacsim/__init__.py'))
    assert environment.resolve_isaac_root() == Path('/venv/lib/python3.12/site-packages/isaacsim')


def test_core_bridge_layout_takes_precedence(tmp_path):
    core = tmp_path / 'exts/isaacsim.ros2.core/jazzy'
    (core / 'lib').mkdir(parents=True)
    (core / 'rclpy').mkdir()
    env = {}
    paths = ['/usr/lib/python3.12']
    ensure_ros_bridge_environment(tmp_path, 'jazzy', environ=env, sys_path=paths)
    assert env['LD_LIBRARY_PATH'] == str(core / 'lib')
    assert paths == [str(core / 'rclpy'), '/usr/lib/python3.12']


def test_missing_isaac_python_reports_actionable_configuration(monkeypatch):
    import pytest
    from franka_motion_runtime.simulation.isaac.runtime import environment
    monkeypatch.setattr(environment.importlib.util, 'find_spec', lambda name: None)
    with pytest.raises(RuntimeError, match='FRANKA_ISAAC_PYTHON'):
        environment.resolve_isaac_root()


def test_launch_environment_removes_ros_overlay_without_guessing_interpreter_root(monkeypatch):
    from franka_motion_runtime.simulation.isaac.environment import create_isaac_process_environment
    monkeypatch.setenv('AMENT_PREFIX_PATH', '/project/install/pkg')
    monkeypatch.setenv('LD_LIBRARY_PATH', '/project/install/pkg/lib:/keep/lib')
    monkeypatch.setenv('PYTHONPATH', '/opt/ros/jazzy/lib/python3.12/site-packages:/venv/lib/python3.12/site-packages')
    env = create_isaac_process_environment('/venv/bin/python', 'rmw_fastrtps_cpp')
    assert env['PYTHONPATH'] == '/venv/lib/python3.12/site-packages'
    assert env['LD_LIBRARY_PATH'] == '/keep/lib'
    assert env['AMENT_PREFIX_PATH'] == ''
    assert 'ISAAC_SIM_ROOT' not in env


def test_binary_root_discovery_normalizes_python_packages(monkeypatch):
    from types import SimpleNamespace
    from franka_motion_runtime.simulation.isaac.runtime import environment
    monkeypatch.setattr(environment.importlib.util, 'find_spec',
                        lambda name: SimpleNamespace(origin='/binary/python_packages/isaacsim/__init__.py'))
    assert environment.resolve_isaac_root() == Path('/binary')
    assert environment.resolve_isaac_root(Path('/explicit')) == Path('/explicit')


def test_legacy_environment_facades_keep_their_exports_importable():
    """Historical module paths remain valid without starting Kit or ROS."""
    import importlib.util
    root = Path(__file__).resolve().parents[3]
    for relative in ('scripts/isaac_runtime/environment.py',
                     'launch/franka_motion_parts/isaac/environment.py'):
        spec = importlib.util.spec_from_file_location('legacy_environment', root / relative)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        assert all(callable(getattr(module, name)) for name in module.__all__)


def test_legacy_root_helper_rejects_venv_root_guessing():
    import pytest
    from franka_motion_runtime.simulation.isaac.environment import isaac_root_from_python
    assert isaac_root_from_python('/binary/python.sh') == Path('/binary')
    with pytest.raises(ValueError, match='selected interpreter'):
        isaac_root_from_python('/venv/bin/python')
