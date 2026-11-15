"""
Test the directed dependency graph of Franka architecture packages.

Responsibility: Enforce acyclicity, ownership boundaries, and declared direct dependencies.

Does not own: Runtime graph discovery or simulator behavior.
"""

from pathlib import Path
import re
from xml.etree import ElementTree


_PACKAGE_FAMILY_ROOT = Path(__file__).resolve().parents[3]
_DEPENDENCY_TAGS = (
    "depend",
    "build_depend",
    "build_export_depend",
    "exec_depend",
)
_CANONICAL_PACKAGES = {
    "franka_motion",
    "manipulation",
    "manipulation_curobo",
    "manipulation_moveit",
    "manipulation_msgs",
    "manipulation_perception",
    "manipulation_perception_isaac_ros",
}
_RETIRED_COMPATIBILITY_PACKAGES = {
    "franka_motion_core",
    "franka_motion_monitoring",
    "franka_motion_moveit",
    "franka_motion_scene",
    "franka_motion_workflows",
}
_REMOVED_SCENARIO_PACKAGES = {
    "franka_motion_bringup",
    "franka_motion_control",
    "franka_motion_description",
    "franka_motion_moveit_config",
    "franka_motion_sim_gazebo",
    "franka_motion_sim_isaac",
    "franka_motion_curobo",
    "franka_motion_curobo_bringup",
}
_REMOVED_PERCEPTION_PACKAGES = {
    "franka_motion_perception",
    "franka_motion_perception_pose",
    "franka_motion_perception_yolo",
}


def _package_dependencies() -> dict[str, set[str]]:
    manifests: dict[str, set[str]] = {}
    parsed_dependencies: dict[str, set[str]] = {}

    manifest_paths = {
        *_PACKAGE_FAMILY_ROOT.glob("franka_motion*/package.xml"),
        *_PACKAGE_FAMILY_ROOT.glob("manipulation*/package.xml"),
    }
    for manifest_path in sorted(manifest_paths):
        manifest = ElementTree.parse(manifest_path).getroot()
        package_name = manifest.findtext("name")
        assert package_name
        assert package_name not in manifests, package_name
        manifests[package_name] = set()
        parsed_dependencies[package_name] = {
            element.text.strip()
            for tag in _DEPENDENCY_TAGS
            for element in manifest.findall(tag)
            if element.text and element.text.strip()
        }

    package_names = set(manifests)
    return {
        package_name: dependencies & package_names
        for package_name, dependencies in parsed_dependencies.items()
    }


def _manifest_dependencies(package_name: str) -> set[str]:
    """Return non-test dependencies declared directly by one package."""
    manifest_path = _PACKAGE_FAMILY_ROOT / package_name / "package.xml"
    manifest = ElementTree.parse(manifest_path).getroot()
    return {
        element.text.strip()
        for tag in _DEPENDENCY_TAGS
        for element in manifest.findall(tag)
        if element.text and element.text.strip()
    }


def _cmake_call_tokens(package_name: str, command: str) -> set[str]:
    """Return tokens from every simple call of one CMake command."""
    package_root = _PACKAGE_FAMILY_ROOT / package_name
    cmake_paths = [
        package_root / "CMakeLists.txt",
        *sorted((package_root / "cmake").glob("*.cmake")),
    ]
    cmake_source = "\n".join(
        path.read_text(encoding="utf-8") for path in cmake_paths
    )
    call_bodies = re.findall(
        rf"\b{re.escape(command)}\s*\((.*?)\)",
        cmake_source,
        flags=re.DOTALL,
    )
    return {
        token
        for call_body in call_bodies
        for token in re.findall(r"[A-Za-z0-9_.:+-]+", call_body)
    }


def _find_cycle(graph: dict[str, set[str]]) -> list[str]:
    active: list[str] = []
    active_indices: dict[str, int] = {}
    complete: set[str] = set()

    def visit(package_name: str) -> list[str]:
        if package_name in active_indices:
            cycle_start = active_indices[package_name]
            return active[cycle_start:] + [package_name]
        if package_name in complete:
            return []

        active_indices[package_name] = len(active)
        active.append(package_name)
        for dependency in sorted(graph[package_name]):
            cycle = visit(dependency)
            if cycle:
                return cycle
        active.pop()
        active_indices.pop(package_name)
        complete.add(package_name)
        return []

    for package_name in sorted(graph):
        cycle = visit(package_name)
        if cycle:
            return cycle
    return []


def test_internal_package_dependencies_are_acyclic() -> None:
    """Canonical manifests must form a directed acyclic graph."""
    cycle = _find_cycle(_package_dependencies())
    assert not cycle, " -> ".join(cycle)


def test_repository_contains_exactly_the_canonical_architecture_packages() -> None:
    """Keep the completed migration free of duplicate package owners."""
    assert set(_package_dependencies()) == _CANONICAL_PACKAGES


def test_generic_packages_do_not_depend_on_scenario() -> None:
    """Keep reusable production packages independent of the Panda scenario."""
    graph = _package_dependencies()
    consumers = {
        package_name
        for package_name, dependencies in graph.items()
        if package_name != "franka_motion" and "franka_motion" in dependencies
    }
    assert not consumers, sorted(consumers)


def test_retired_compatibility_packages_stay_absent_from_manifests() -> None:
    """Prevent canonical packages from regaining a retired package dependency."""
    graph = _package_dependencies()
    assert _RETIRED_COMPATIBILITY_PACKAGES.isdisjoint(graph)
    for package_name in _CANONICAL_PACKAGES:
        dependencies = _manifest_dependencies(package_name)
        assert dependencies.isdisjoint(
            _RETIRED_COMPATIBILITY_PACKAGES
        ), package_name

    for package_name in _RETIRED_COMPATIBILITY_PACKAGES:
        package_root = _PACKAGE_FAMILY_ROOT / package_name
        assert not (package_root / "package.xml").exists(), package_name
        assert not (package_root / "CMakeLists.txt").exists(), package_name


def test_perception_split_packages_stay_removed() -> None:
    """Keep generic perception implementation under its canonical package."""
    graph = _package_dependencies()
    assert _REMOVED_PERCEPTION_PACKAGES.isdisjoint(graph)
    for package_name in _REMOVED_PERCEPTION_PACKAGES:
        package_root = _PACKAGE_FAMILY_ROOT / package_name
        assert not (package_root / "package.xml").exists(), package_name
        assert not (package_root / "CMakeLists.txt").exists(), package_name


def test_optional_curobo_backend_has_no_reverse_or_required_dependency() -> None:
    """Keep runtime discovery optional and the plugin robot-independent."""
    graph = _package_dependencies()
    assert "manipulation_curobo" not in graph["franka_motion"]
    assert "franka_motion" not in graph["manipulation_curobo"]
    assert "franka_motion_curobo" not in graph
    assert "franka_motion_curobo_bringup" not in graph


def test_runtime_harness_declares_common_runtime_and_cli_dependencies() -> None:
    """The installed matrix must not rely on an already-populated workspace."""
    dependencies = _manifest_dependencies("franka_motion")
    required = {
        "manipulation",
        "manipulation_msgs",
        "ros2action",
        "ros2cli",
        "ros2controlcli",
        "ros2launch",
        "ros2param",
        "ros2pkg",
        "ros2run",
        "ros2topic",
        "rosgraph_msgs",
        "sensor_msgs",
    }
    assert required <= dependencies

    target_specific = {"manipulation_perception_isaac_ros"}
    assert not (dependencies & target_specific)


def test_scenario_declares_moveit_config_plugin_dependencies() -> None:
    """A clean install must include every plugin selected by scenario YAML."""
    dependencies = _manifest_dependencies("franka_motion")
    required_plugins = {
        "moveit_kinematics",
        "moveit_planners_ompl",
        "moveit_ros_perception",
        "moveit_simple_controller_manager",
    }
    assert required_plugins <= dependencies


def test_scenario_split_packages_stay_removed() -> None:
    """Panda resources and launch code must have one package owner."""
    existing = {
        package_name
        for package_name in _REMOVED_SCENARIO_PACKAGES
        if (_PACKAGE_FAMILY_ROOT / package_name / "package.xml").exists()
    }
    assert not existing, sorted(existing)


def test_pre_migration_scene_helpers_stay_removed() -> None:
    """Keep scene mutation policy in the robot-neutral manipulation package."""
    scenario_include = (
        _PACKAGE_FAMILY_ROOT / "franka_motion" / "include" / "franka_motion"
    )
    retired_headers = {
        "box_collision.hpp",
        "payload_collision.hpp",
        "planning_scene_utils.hpp",
    }
    existing = {
        header_name
        for header_name in retired_headers
        if (scenario_include / header_name).exists()
    }
    assert not existing, sorted(existing)

    canonical_box_helper = (
        _PACKAGE_FAMILY_ROOT
        / "manipulation"
        / "include"
        / "manipulation"
        / "scene"
        / "box_collision.hpp"
    )
    assert canonical_box_helper.is_file()


def test_direct_cpp_dependencies_are_declared_linked_and_public_ones_exported() -> None:
    """Direct APIs are explicit; only dependencies of public targets are exported."""
    contracts = {
        "manipulation": {
            "manifest": {"builtin_interfaces", "tf2"},
            "find": {"builtin_interfaces", "tf2"},
            "target": {"builtin_interfaces", "tf2"},
            "export": {"builtin_interfaces", "tf2"},
        },
        "manipulation_curobo": {
            "manifest": {"eigen"},
            "find": {"Eigen3"},
            "target": {"Eigen3::Eigen"},
            "export": set(),
        },
    }

    for package_name, expected in contracts.items():
        target_tokens = _cmake_call_tokens(
            package_name, "ament_target_dependencies"
        ) | _cmake_call_tokens(package_name, "target_link_libraries")
        assert expected["manifest"] <= _manifest_dependencies(package_name)
        assert expected["find"] <= _cmake_call_tokens(
            package_name, "find_package"
        )
        assert expected["target"] <= target_tokens
        assert expected["export"] <= _cmake_call_tokens(
            package_name, "ament_export_dependencies"
        )
