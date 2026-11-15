"""
Protect the optional plugin boundary from robot and legacy task APIs.

Responsibility: Keep the plugin robot-independent, optional, and V1-free.
"""

from pathlib import Path
import re
import xml.etree.ElementTree as ElementTree


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PRODUCTION_ROOTS = (
    PACKAGE_ROOT / 'action',
    PACKAGE_ROOT / 'include',
    PACKAGE_ROOT / 'manipulation_curobo_runtime',
    PACKAGE_ROOT / 'scripts',
    PACKAGE_ROOT / 'src',
)
TEXT_SUFFIXES = {'.action', '.cpp', '.hpp', '.py'}
ROBOT_SPECIFIC = re.compile(r'\b(?:franka|panda)\b', re.IGNORECASE)


def test_production_sources_have_no_robot_profile_defaults() -> None:
    """Robot names and profile values must be injected by a leaf package."""
    violations = []
    for root in PRODUCTION_ROOTS:
        for path in sorted(root.rglob('*')):
            if not path.is_file() or path.suffix not in TEXT_SUFFIXES:
                continue
            for line_number, line in enumerate(
                path.read_text(encoding='utf-8').splitlines(), start=1
            ):
                if ROBOT_SPECIFIC.search(line):
                    violations.append(
                        f'{path.relative_to(PACKAGE_ROOT)}:{line_number}: {line.strip()}'
                    )

    assert not violations, '\n'.join(violations)


def test_backend_has_no_task_action_or_generic_dependency() -> None:
    """The optional planner is selected only through MoveIt's plugin API."""
    manifest = ElementTree.parse(PACKAGE_ROOT / 'package.xml').getroot()
    dependencies = {
        element.text
        for element in manifest
        if element.tag.endswith('depend') and element.text
    }
    production_text = '\n'.join(
        path.read_text(encoding='utf-8')
        for root in PRODUCTION_ROOTS
        for path in root.rglob('*')
        if path.is_file() and path.suffix in TEXT_SUFFIXES
    )

    assert 'manipulation_msgs' not in dependencies
    assert not (PACKAGE_ROOT / 'action').exists()
    assert 'CuRoboPlanPose' not in production_text
    assert '/plan_curobo_pose' not in production_text
    assert 'planner_server' not in production_text


def test_plugin_description_has_one_canonical_id() -> None:
    root = ElementTree.parse(
        PACKAGE_ROOT / 'curobo_moveit_plugin_description.xml'
    ).getroot()

    assert [entry.attrib['name'] for entry in root.findall('class')] == [
        'manipulation_curobo/CuRoboPlanner'
    ]


def test_installed_surface_has_no_stale_package_action() -> None:
    """Incremental installs must remove the former package-qualified type."""
    from ament_index_python.packages import get_package_prefix

    prefix = Path(get_package_prefix('manipulation_curobo'))

    assert not (
        prefix / 'share' / 'manipulation_curobo' / 'action'
    ).exists()
    legacy_modules = {
        'goal_admission',
        'planner_server',
        'planning_contract',
        'planning_worker',
        'reference_runtime',
        'world_config',
    }
    installed_python_roots = tuple(prefix.glob('lib/python*/site-packages')) + tuple(
        prefix.glob('local/lib/python*/dist-packages')
    )
    for python_root in installed_python_roots:
        assert not list(
            python_root.glob('manipulation_curobo_runtime-0.1.0*.egg-info')
        )
        runtime = python_root / 'manipulation_curobo_runtime'
        assert not any((runtime / f'{module}.py').exists() for module in legacy_modules)
        assert not any(
            list((runtime / '__pycache__').glob(f'{module}*.pyc'))
            for module in legacy_modules
        )
    assert not (
        prefix / 'lib' / 'manipulation_curobo' / 'curobo_planner_server.py'
    ).exists()
    assert not (
        prefix
        / 'include'
        / 'manipulation_curobo'
        / 'manipulation_curobo'
        / 'curobo_planner.hpp'
    ).exists()
    assert not (
        prefix / 'share' / 'manipulation_curobo' / 'rust'
    ).exists()
    assert not list(
        (prefix / 'lib').glob('libmanipulation_curobo__rosidl*')
    )
    assert not (prefix / 'include' / 'manipulation_curobo').exists()
    assert not (prefix / 'lib' / 'libcurobo_planning_io.a').exists()
    assert not list((prefix / 'lib').glob('libcurobo_process_support.*'))
    assert not list(
        (prefix / 'share' / 'manipulation_curobo' / 'cmake').glob(
            'export_manipulation_curobo*'
        )
    )
    assert not list(
        (prefix / 'share' / 'manipulation_curobo' / 'cmake').glob(
            'ament_cmake_export_*-extras.cmake'
        )
    )
    assert not (
        prefix
        / 'share'
        / 'ament_index'
        / 'resource_index'
        / 'rosidl_interfaces'
        / 'manipulation_curobo'
    ).exists()


def test_plugin_is_the_only_exported_cmake_integration_surface() -> None:
    cmake = (PACKAGE_ROOT / 'CMakeLists.txt').read_text(encoding='utf-8')

    assert 'ament_export_targets' not in cmake
    assert 'ament_export_include_directories' not in cmake
    assert 'ament_export_dependencies' not in cmake
    assert 'TARGETS curobo_moveit_plugin' in cmake
