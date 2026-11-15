"""Responsibility: Guard the V2/Servo package boundary and V1 removal."""

from pathlib import Path
import re
import xml.etree.ElementTree as et

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
COMPATIBILITY_INSTALL = PACKAGE_ROOT / 'cmake' / 'CompatibilityInstall.cmake'
PRODUCTION_ROOTS = (
    PACKAGE_ROOT / 'include',
    PACKAGE_ROOT / 'src',
    PACKAGE_ROOT / 'config',
    PACKAGE_ROOT / 'cmake',
)
FORBIDDEN_PATTERNS = {
    'franka identifier': re.compile(r'\bfranka(?:_|\b)', re.IGNORECASE),
    'Panda identifier': re.compile(r'\bpanda(?:_|\b)', re.IGNORECASE),
    'table stereo scenario': re.compile(r'\btable_stereo\b', re.IGNORECASE),
    'cube object': re.compile(r'\bcube\b', re.IGNORECASE),
    'legacy package': re.compile(r'\bfranka_motion(?:_|\b)', re.IGNORECASE),
}
LEGACY_V1_PATTERNS = {
    'V1 action type': re.compile(r'\bTransferObject\b'),
    'V1 action include': re.compile(
        r'manipulation_msgs/action/transfer_object\.hpp'
    ),
    'V1 backend target': re.compile(r'\btransfer_backend\b'),
    'V1 executable': re.compile(r'\bmoveit_transfer_object_server\b'),
    'V1 host': re.compile(r'\bmoveit_motion_host\b'),
    'V1 private endpoint': re.compile(
        r'/franka_moveit_transfer_object_server/(?:transfer_object|set_task_paused)'
    ),
}


def production_files() -> list[Path]:
    """Return deterministic text inputs owned by the production package."""
    files = [
        PACKAGE_ROOT / 'CMakeLists.txt',
        PACKAGE_ROOT / 'package.xml',
        PACKAGE_ROOT / 'fail_closed_octomap_updater_plugin_description.xml',
    ]
    for root in PRODUCTION_ROOTS:
        files.extend(path for path in root.rglob('*') if path.is_file())
    return sorted(files)


def test_production_files_have_no_robot_or_scenario_identifiers() -> None:
    """Keep examples and robot profiles outside the reusable backend."""
    violations: list[str] = []
    for path in production_files():
        if path == COMPATIBILITY_INSTALL:
            continue
        text = path.read_text(encoding='utf-8')
        for label, pattern in FORBIDDEN_PATTERNS.items():
            if pattern.search(text):
                violations.append(f'{path.relative_to(PACKAGE_ROOT)}: {label}')

    assert not violations, 'generic boundary violations:\n' + '\n'.join(violations)


def test_manifest_has_no_legacy_package_dependency() -> None:
    """Ensure the adapter depends only on the new stable manipulation packages."""
    root = et.parse(PACKAGE_ROOT / 'package.xml').getroot()
    dependencies = {
        element.text.strip()
        for element in root
        if element.tag.endswith('depend') and element.text
    }

    legacy = sorted(name for name in dependencies if name.startswith('franka_motion'))
    assert not legacy, f'legacy package dependencies: {legacy}'
    assert {'manipulation', 'manipulation_msgs'} <= dependencies


def test_frame_policy_has_one_generic_owner() -> None:
    """Keep the remaining Octomap gate on the shared frame validator."""
    local_validator = PACKAGE_ROOT / 'src' / 'frame_validation.hpp'
    assert not local_validator.exists()

    frame_consumers = ('stamped_cloud_tf_gate.cpp',)
    generic_include = '#include "manipulation/validation/frame_id.hpp"'
    for filename in frame_consumers:
        source = (PACKAGE_ROOT / 'src' / filename).read_text(encoding='utf-8')
        assert generic_include in source, filename
        assert 'frame_validation.hpp' not in source, filename


def test_v1_backend_surface_is_absent() -> None:
    """Prevent the removed lower action backend from returning by accident."""
    violations: list[str] = []
    for path in production_files():
        if path == COMPATIBILITY_INSTALL:
            continue
        text = path.read_text(encoding='utf-8')
        for label, pattern in LEGACY_V1_PATTERNS.items():
            if pattern.search(text):
                violations.append(f'{path.relative_to(PACKAGE_ROOT)}: {label}')

    removed_headers = (
        'moveit_motion_host.hpp',
        'moveit_transfer_task.hpp',
        'transfer_port_types.hpp',
        'transfer_ports.hpp',
        'transfer_state.hpp',
    )
    include_root = PACKAGE_ROOT / 'include' / 'manipulation_moveit'
    violations.extend(
        f'include/manipulation_moveit/{name}: removed V1 header exists'
        for name in removed_headers
        if (include_root / name).exists()
    )

    assert not violations, 'V1 backend surface violations:\n' + '\n'.join(
        violations
    )


def test_incremental_install_has_no_v1_backend_artifacts() -> None:
    """An upgraded overlay must expose only the current V2/Servo surface."""
    from ament_index_python.packages import get_package_prefix

    prefix = Path(get_package_prefix('manipulation_moveit'))
    package_include = (
        prefix / 'include' / 'manipulation_moveit' / 'manipulation_moveit'
    )
    legacy_headers = (
        'moveit_motion_host.hpp',
        'moveit_transfer_task.hpp',
        'operation_id.hpp',
        'ros_arm_motion_port.hpp',
        'ros_gripper_port.hpp',
        'ros_health_port.hpp',
        'ros_occupancy_map_port.hpp',
        'ros_scene_ports.hpp',
        'ros_transform_port.hpp',
        'scene_response_adapter.hpp',
        'transfer_geometry.hpp',
        'transfer_port_types.hpp',
        'transfer_ports.hpp',
        'transfer_state.hpp',
    )
    assert not any((package_include / name).exists() for name in legacy_headers)
    assert not (
        prefix
        / 'lib'
        / 'manipulation_moveit'
        / 'moveit_transfer_object_server'
    ).exists()
    assert not any(
        (prefix / 'lib' / name).exists()
        for name in (
            'libmoveit_motion_host.a',
            'libmoveit_motion_host.so',
            'libmoveit_transfer_task.a',
            'libmoveit_transfer_task.so',
            'libtransfer_backend.a',
            'libtransfer_backend.so',
        )
    )
    assert not (
        prefix
        / 'share'
        / 'manipulation_moveit'
        / 'config'
        / 'transfer_object.defaults.yaml'
    ).exists()


def test_cmake_exports_only_current_motion_targets() -> None:
    """Freeze the Servo-only host and narrow Octomap support split."""
    cmake = (PACKAGE_ROOT / 'cmake' / 'Targets.cmake').read_text(
        encoding='utf-8'
    )
    root_cmake = (PACKAGE_ROOT / 'CMakeLists.txt').read_text(encoding='utf-8')

    for target in ('moveit_servo_task_host', 'octomap_tf_gate_support'):
        assert re.search(rf'add_library\(\s*{target}\b', cmake)
        assert target in root_cmake
    assert 'moveit_servo_task_server' in cmake
