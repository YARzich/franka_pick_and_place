"""
Enforce the dependency direction and robot-agnostic boundary of manipulation.

Responsibility: Catch package, namespace, and scenario-specific coupling before
it reaches a replaceable planner backend or robot application.
"""

from pathlib import Path
import re
from xml.etree import ElementTree


_PACKAGE_ROOT = Path(__file__).resolve().parents[1]
_TEXT_SUFFIXES = {'.cmake', '.cpp', '.hpp', '.py', '.xml', '.yaml', '.md'}
_PRODUCTION_ROOTS = ('include', 'src', 'config', 'launch', 'cmake')
_ALLOWED_RUNTIME_DEPENDENCIES = {
    'ament_index_python',
    'builtin_interfaces',
    'controller_manager_msgs',
    'control_msgs',
    'diagnostic_msgs',
    'diagnostic_updater',
    'geometry_msgs',
    'libssl-dev',
    'launch',
    'launch_ros',
    'manipulation_msgs',
    'moveit_msgs',
    'object_recognition_msgs',
    'rclcpp',
    'rclcpp_action',
    'rosgraph_msgs',
    'sensor_msgs',
    'shape_msgs',
    'std_msgs',
    'tf2',
    'tf2_geometry_msgs',
    'tf2_ros',
    'unique_identifier_msgs',
    'urdf',
}


def _production_text() -> dict[Path, str]:
    """Return UTF-8 production sources without generated or test artifacts."""
    result = {
        Path('CMakeLists.txt'): (
            _PACKAGE_ROOT / 'CMakeLists.txt'
        ).read_text(encoding='utf-8')
    }
    for directory_name in _PRODUCTION_ROOTS:
        directory = _PACKAGE_ROOT / directory_name
        for path in directory.rglob('*'):
            if path.is_file() and path.suffix in _TEXT_SUFFIXES:
                result[path.relative_to(_PACKAGE_ROOT)] = path.read_text(encoding='utf-8')
    return result


def test_manifest_points_only_toward_stable_or_ros_dependencies() -> None:
    """The reusable package must never depend on an application or backend package."""
    manifest = ElementTree.parse(_PACKAGE_ROOT / 'package.xml').getroot()
    dependency_tags = ('depend', 'build_depend', 'build_export_depend', 'exec_depend')
    dependencies = {
        element.text.strip()
        for tag in dependency_tags
        for element in manifest.findall(tag)
        if element.text and element.text.strip()
    }

    assert 'manipulation_msgs' in dependencies
    assert dependencies <= _ALLOWED_RUNTIME_DEPENDENCIES, (
        'Unexpected application, backend, or undeclared runtime layer: '
        f'{sorted(dependencies - _ALLOWED_RUNTIME_DEPENDENCIES)}'
    )


def test_production_has_no_robot_or_scenario_identity() -> None:
    """Robot names and example-object workflows belong to application packages."""
    forbidden = re.compile(
        r'\b(?:franka|panda|megapose|yolo|static_table|scripted_transfer)\b',
        flags=re.IGNORECASE,
    )
    offenders = {
        str(path): sorted({match.group(0) for match in forbidden.finditer(text)})
        for path, text in _production_text().items()
        if forbidden.search(text)
    }
    assert not offenders


def test_headers_expose_only_manipulation_namespaces() -> None:
    """Installed headers must not leak namespaces or includes from superseded packages."""
    offenders: dict[str, list[str]] = {}
    for path, text in _production_text().items():
        if not str(path).startswith('include/'):
            continue
        found = [
            token
            for token in ('franka_motion_core', 'franka_motion_workflows', 'franka_motion::')
            if token in text
        ]
        if found:
            offenders[str(path)] = found
    assert not offenders


def test_v1_transfer_runtime_is_not_installed_or_built() -> None:
    """The reusable package exposes only the V2 transfer facade and neutral core."""
    removed_trees = (
        _PACKAGE_ROOT / 'include' / 'manipulation' / 'client',
        _PACKAGE_ROOT / 'src' / 'client',
        _PACKAGE_ROOT / 'test' / 'client',
    )
    remaining_tree_files = {
        str(path.relative_to(_PACKAGE_ROOT))
        for tree in removed_trees
        for path in tree.rglob('*')
        if path.is_file() and path.suffix in _TEXT_SUFFIXES
    }
    assert not remaining_tree_files

    old_session = (
        _PACKAGE_ROOT / 'include' / 'manipulation' / 'workflow' / 'detail'
        / 'transfer_object_session.hpp'
    )
    neutral_session = (
        _PACKAGE_ROOT / 'include' / 'manipulation' / 'workflow' / 'detail'
        / 'transfer_sequence_session.hpp'
    )
    assert not old_session.is_file()
    assert neutral_session.is_file()

    forbidden_tokens = (
        'TransferObjectActionServer',
        'TransferObjectTaskContext',
        'TransferObjectStopCause',
        'TransferObjectSequenceActionServer',
        'PerceptionResolvedTransferSequence',
        'StaticBoxTransferSequence',
        'manipulation_transfer_object_client',
        'manipulation_msgs/action/transfer_object.hpp',
        'manipulation_msgs/action/transfer_object_sequence.hpp',
        'manipulation_msgs/action/perception_resolved_transfer_sequence.hpp',
    )
    offenders = {
        str(path): [token for token in forbidden_tokens if token in text]
        for path, text in _production_text().items()
        if path != Path('cmake/ManipulationCompatibilityInstall.cmake')
        if any(token in text for token in forbidden_tokens)
    }
    assert not offenders


def test_scenario_entrypoints_are_not_installed_here() -> None:
    """Generic runtime exposes infrastructure nodes, never concrete task requests."""
    forbidden_basenames = {
        'static_table_scene.cpp',
        'static_table_scene_node.cpp',
        'scripted_transfer_main.cpp',
        'cube_grasp_pose_policy.cpp',
    }
    present = {
        path.name
        for path in _PACKAGE_ROOT.rglob('*')
        if path.is_file() and path.name in forbidden_basenames
    }
    assert not present
