"""
Test ownership and readability contracts of the package-local bringup.

Responsibility:
- Keep production launch code documented, self-contained, and orchestration-only.

Does not own:
- Runtime simulator behavior or optional cuRobo reference implementation.
"""

import ast
from pathlib import Path
import xml.etree.ElementTree as ElementTree


_PACKAGE_ROOT = Path(__file__).resolve().parents[2]
_CPP_SOURCE_ROOT = _PACKAGE_ROOT / "src"
_RUNTIME_ROOT = _PACKAGE_ROOT / "franka_motion_runtime" / "bringup"
_TASKS_ROOT = _PACKAGE_ROOT / "franka_motion_runtime" / "tasks"
_TASK_TESTS_ROOT = _PACKAGE_ROOT / "test" / "tasks"
_REPOSITORY_ROOT = _PACKAGE_ROOT.parent
_LAUNCH_DIRECTORY = _PACKAGE_ROOT / "launch"
_LAUNCH_PARTS_DIRECTORY = _RUNTIME_ROOT / "launch_parts"
_PRODUCTION_LAUNCHES = {
    "franka_gazebo_moveit.launch.py",
    "franka_gazebo_moveit_table.launch.py",
    "franka_gazebo_moveit_table_camera.launch.py",
    "franka_gazebo_active_cancel.launch.py",
    "franka_gazebo_move_cube.launch.py",
    "franka_gazebo_move_cube_round_trip.launch.py",
    "franka_isaac_move_cube.launch.py",
    "franka_isaac_move_cube_round_trip.launch.py",
    "franka_isaac_moveit.launch.py",
    "franka_isaac_moveit_table.launch.py",
    "franka_isaac_moveit_table_camera.launch.py",
    "franka_motion_bringup.launch.py",
}
_TRANSITIONAL_PACKAGES = {
    "franka_motion_bringup",
    "franka_motion_control",
    "franka_motion_description",
    "franka_motion_moveit_config",
    "franka_motion_monitoring",
    "franka_motion_scene",
    "franka_motion_sim_gazebo",
    "franka_motion_sim_isaac",
    "franka_motion_workflows",
}
_REMOVED_PERCEPTION_PACKAGES = {
    "franka_motion_perception",
    "franka_motion_perception_pose",
    "franka_motion_perception_yolo",
}


def _production_sources() -> list[Path]:
    """Return maintained bringup sources without optional reference leaves."""
    launch_sources = [
        _LAUNCH_DIRECTORY / name for name in sorted(_PRODUCTION_LAUNCHES)
    ]
    wrapper_sources = sorted(
        (_PACKAGE_ROOT / "scripts" / "runtime").glob("*.py")
    )
    task_entrypoints = sorted(
        path
        for path in (_PACKAGE_ROOT / "scripts" / "tasks").glob("*")
        if path.is_file()
    )
    return (
        sorted(_RUNTIME_ROOT.rglob("*.py"))
        + sorted(_TASKS_ROOT.rglob("*.py"))
        + launch_sources
        + wrapper_sources
        + task_entrypoints
    )


def _parse(source_path: Path) -> ast.Module:
    """Parse one source while retaining its path in syntax errors."""
    return ast.parse(
        source_path.read_text(encoding="utf-8"),
        filename=str(source_path),
    )


def _call_name(call: ast.Call) -> str | None:
    """Return a simple or attribute call's final identifier."""
    if isinstance(call.func, ast.Name):
        return call.func.id
    if isinstance(call.func, ast.Attribute):
        return call.func.attr
    return None


def test_production_launch_file_set_is_explicit_and_complete() -> None:
    """All production entrypoints must remain package-local."""
    installed_names = {
        path.name for path in _LAUNCH_DIRECTORY.glob("*.launch.py")
    }
    assert _PRODUCTION_LAUNCHES <= installed_names


def test_production_modules_explain_ownership() -> None:
    """Every maintained bringup module must state both sides of its boundary."""
    for source_path in _production_sources():
        module_docstring = ast.get_docstring(_parse(source_path)) or ""
        assert "Responsibility:" in module_docstring, source_path
        assert "Does not own:" in module_docstring, source_path


def test_removed_pick_place_aliases_stay_absent() -> None:
    """Keep configured and detected requests on the one V2 client surface."""
    dead_sources = (
        "franka_moveit_pick_place.cpp",
        "scripted_pick_place_scenario.cpp",
        "scripted_pick_place_scenario.hpp",
        "static_cube_planning_scene.cpp",
        "static_cube_planning_scene.hpp",
    )
    for source_name in dead_sources:
        assert not (_CPP_SOURCE_ROOT / source_name).exists(), source_name

    compatibility_cmake = (
        _PACKAGE_ROOT / "cmake" / "FrankaMotionCompatibilityTargets.cmake"
    ).read_text(encoding="utf-8")
    runtime_install = (
        _PACKAGE_ROOT / "cmake" / "FrankaMotionRuntimeInstall.cmake"
    ).read_text(encoding="utf-8")
    assert "add_executable(\n  franka_moveit_pick_place" not in compatibility_cmake
    assert "scripts/tasks/franka_transfer_sequence_client" in runtime_install

    removed_paths = (
        "franka_motion_runtime/tasks/detected_pick_place_client.py",
        "franka_motion_runtime/tasks/detected_pick_place_launch.py",
        "franka_motion_runtime/tasks/detected_pick_place_request.py",
        "franka_motion_runtime/tasks/legacy_pick_place_client.py",
        "franka_motion_runtime/tasks/legacy_pick_place_request.py",
        "scripts/tasks/franka_moveit_detected_pick_place",
        "scripts/tasks/franka_moveit_pick_place",
    )
    for relative_path in removed_paths:
        assert not (_PACKAGE_ROOT / relative_path).exists(), relative_path
        assert relative_path not in runtime_install


def test_production_sources_do_not_reference_transitional_split_packages() -> None:
    """The local MoveIt path must not be masked by an old workspace overlay."""
    for source_path in _production_sources():
        source = source_path.read_text(encoding="utf-8")
        referenced = {
            package for package in _TRANSITIONAL_PACKAGES if package in source
        }
        assert not referenced, (source_path, sorted(referenced))


def test_retired_perception_packages_have_no_source_files() -> None:
    """Prevent duplicate robot-qualified perception implementations returning."""
    for package_name in _REMOVED_PERCEPTION_PACKAGES:
        package_root = _REPOSITORY_ROOT / package_name
        source_files = {
            path
            for suffix in ("*.cpp", "*.hpp", "*.py")
            for path in package_root.rglob(suffix)
        }
        assert not source_files, (package_name, sorted(source_files))


def test_manifest_uses_generic_runtime_packages_for_bringup() -> None:
    """Pin the new dependency direction while legacy C++ demos still migrate."""
    root = ElementTree.parse(_PACKAGE_ROOT / "package.xml").getroot()
    dependencies = {
        element.text
        for element in root
        if element.tag.endswith("depend") and element.text
    }
    assert {
        "manipulation",
        "manipulation_moveit",
        "manipulation_perception",
    } <= dependencies
    assert "manipulation_perception_isaac_ros" not in dependencies
    assert not (_TRANSITIONAL_PACKAGES & dependencies)


def test_launch_entrypoints_do_not_mutate_python_path() -> None:
    """Installed launch modules must be imported through the package install."""
    for launch_name in _PRODUCTION_LAUNCHES:
        tree = _parse(_LAUNCH_DIRECTORY / launch_name)
        attributes = {
            (node.value.id, node.attr)
            for node in ast.walk(tree)
            if isinstance(node, ast.Attribute)
            if isinstance(node.value, ast.Name)
        }
        assert ("sys", "path") not in attributes, launch_name


def test_launch_descriptions_are_named_before_return() -> None:
    """Keep launch-description construction visible and inspectable."""
    for source_path in _production_sources():
        tree = _parse(source_path)
        parent_by_child = {
            child: parent
            for parent in ast.walk(tree)
            for child in ast.iter_child_nodes(parent)
        }
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            if _call_name(node) != "LaunchDescription":
                continue
            assert isinstance(
                parent_by_child[node],
                (ast.Assign, ast.AnnAssign),
            ), source_path


def test_canonical_ros_actions_have_explicit_clock_policy() -> None:
    """Every ROS action created by bringup must receive use_sim_time."""
    accepted_parameter_sources = {
        "table_camera_parameters",
        "use_sim_time",
    }
    for source_path in _LAUNCH_PARTS_DIRECTORY.rglob("*.py"):
        for node in ast.walk(_parse(source_path)):
            if not isinstance(node, ast.Call):
                continue
            if _call_name(node) not in {
                "ComposableNode",
                "ComposableNodeContainer",
                "Node",
            }:
                continue
            parameters = next(
                (
                    keyword.value
                    for keyword in node.keywords
                    if keyword.arg == "parameters"
                ),
                None,
            )
            assert parameters is not None, source_path
            parameter_source = ast.unparse(parameters)
            assert any(
                marker in parameter_source
                for marker in accepted_parameter_sources
            ), (source_path, parameter_source)


def test_every_moveit_graph_exposes_the_same_optional_servo_layer() -> None:
    """Cover standard and table-camera Gazebo/Isaac assembly paths."""
    assembly_sources = [
        _RUNTIME_ROOT / "gazebo_moveit.py",
        _RUNTIME_ROOT / "isaac_moveit.py",
        _RUNTIME_ROOT / "isaac_table_camera_moveit.py",
        _LAUNCH_DIRECTORY / "franka_gazebo_moveit_table_camera.launch.py",
    ]
    for source_path in assembly_sources:
        source = source_path.read_text(encoding="utf-8")
        assert "declare_start_servo_argument" in source, source_path
        assert "create_moveit_servo_node" in source, source_path
        assert "create_servo_task_server_node" in source, source_path


def test_standard_leaves_only_select_policy_and_delegate() -> None:
    """Empty/table leaves must not rebuild simulator or controller graphs."""
    expected_factory = {
        "franka_gazebo_moveit.launch.py": "create_gazebo_moveit_description",
        "franka_gazebo_moveit_table.launch.py": (
            "create_gazebo_moveit_description"
        ),
        "franka_isaac_moveit.launch.py": "create_isaac_moveit_description",
        "franka_isaac_moveit_table.launch.py": (
            "create_isaac_moveit_description"
        ),
    }
    forbidden_calls = {
        "ExecuteProcess",
        "Node",
        "create_gazebo_controller_spawners",
        "create_isaac_controller_spawners",
    }
    for launch_file, factory_name in expected_factory.items():
        source_path = _LAUNCH_DIRECTORY / launch_file
        call_names = {
            _call_name(node)
            for node in ast.walk(_parse(source_path))
            if isinstance(node, ast.Call)
        }
        assert factory_name in call_names, source_path
        assert forbidden_calls.isdisjoint(call_names), source_path


def test_every_moveit_graph_has_only_v2_transfer_and_standalone_servo_hosts() -> None:
    """Each graph exposes one V2 workflow and one separately gated Servo host."""
    source_paths = [
        _RUNTIME_ROOT / "gazebo_moveit.py",
        _RUNTIME_ROOT / "isaac_moveit.py",
        _RUNTIME_ROOT / "isaac_table_camera_moveit.py",
        _LAUNCH_DIRECTORY / "franka_gazebo_moveit_table_camera.launch.py",
    ]
    for source_path in source_paths:
        source = source_path.read_text(encoding="utf-8")
        assert source.count("create_servo_task_server_node(") == 1, source_path
        assert source.count("create_transfer_sequence_server_node(") == 1, source_path
        assert "create_transfer_server_node" not in source, source_path
        assert "moveit_transfer_object_server" not in source, source_path


def test_table_camera_default_mode_remains_gigapose() -> None:
    """Package ownership changes must not alter the public camera default."""
    source_path = _LAUNCH_PARTS_DIRECTORY / "camera" / "core_arguments.py"
    mode_defaults = []
    for node in ast.walk(_parse(source_path)):
        if not isinstance(node, ast.Call):
            continue
        if _call_name(node) != "add_launch_argument" or len(node.args) < 4:
            continue
        if (
            isinstance(node.args[2], ast.Constant)
            and node.args[2].value == "mode"
            and isinstance(node.args[3], ast.Constant)
        ):
            mode_defaults.append(node.args[3].value)
    assert mode_defaults == ["gigaPose"]


def test_core_modules_remain_reviewable() -> None:
    """Unexpected file growth must trigger an intentional responsibility split."""
    maximum_lines = {
        "contracts.py": 180,
        "gazebo_moveit.py": 190,
        "isaac_moveit.py": 210,
        "isaac_table_camera_moveit.py": 250,
        "launch_factory.py": 220,
        "legacy_routes.py": 220,
    }
    for filename, limit in maximum_lines.items():
        source_path = _RUNTIME_ROOT / filename
        line_count = len(source_path.read_text(encoding="utf-8").splitlines())
        assert line_count <= limit, (source_path, line_count, limit)

    for filename, limit in {
        "sequence_client_test_support.py": 180,
        "test_task_sequence_client_runtime.py": 300,
        "test_task_sequence_goal_guard.py": 300,
    }.items():
        source_path = _TASK_TESTS_ROOT / filename
        line_count = len(source_path.read_text(encoding="utf-8").splitlines())
        assert line_count <= limit, (source_path, line_count, limit)

    for filename, limit in {
        "arguments.py": 190,
        "gazebo_stack.py": 210,
        "isaac_stack.py": 220,
    }.items():
        source_path = _LAUNCH_PARTS_DIRECTORY / filename
        line_count = len(source_path.read_text(encoding="utf-8").splitlines())
        assert line_count <= limit, (source_path, line_count, limit)

    for filename, limit in {
        "launch.py": 210,
        "model.py": 120,
        "request.py": 80,
        "request_parsing.py": 110,
        "rigid_transform.py": 180,
        "sequence_client.py": 80,
        "sequence_client_cancel.py": 230,
        "sequence_client_config.py": 160,
        "sequence_client_feedback.py": 80,
        "sequence_client_policy.py": 150,
        "sequence_client_process.py": 80,
        "sequence_client_runtime.py": 250,
        "sequence_client_waits.py": 130,
        "sequence_goal_guard.py": 260,
        "sequence_messages.py": 110,
        "sequence_result_decode.py": 60,
        "sequence_terminal.py": 100,
    }.items():
        source_path = _TASKS_ROOT / filename
        line_count = len(source_path.read_text(encoding="utf-8").splitlines())
        assert line_count <= limit, (source_path, line_count, limit)

    for filename, limit in {
        "franka_moveit_sequence.cpp": 150,
        "moveit_parameter_snapshot.cpp": 260,
        "moveit_parameter_sync.cpp": 180,
        "parameter_snapshot_apply.cpp": 100,
        "parameter_snapshot_client.cpp": 210,
        "moveit_sequence_scenario.cpp": 150,
        "moveit_sequence_scenario.hpp": 90,
    }.items():
        source_path = _CPP_SOURCE_ROOT / filename
        line_count = len(source_path.read_text(encoding="utf-8").splitlines())
        assert line_count <= limit, (source_path, line_count, limit)


def test_mtc_sources_explain_responsibility_and_remain_reviewable() -> None:
    """Keep the scenario-owned MTC implementation split into readable roles."""
    source_groups = {
        _PACKAGE_ROOT / "include" / "franka_motion": "mtc_*.hpp",
        _CPP_SOURCE_ROOT / "mtc": "*.[ch]pp",
        _PACKAGE_ROOT / "test" / "mtc": "*.[ch]pp",
    }
    for directory, pattern in source_groups.items():
        for source_path in sorted(directory.glob(pattern)):
            lines = source_path.read_text(encoding="utf-8").splitlines()
            assert lines and lines[0].startswith("// Responsibility:"), source_path
            assert len(lines) <= 220, (source_path, len(lines), 220)
