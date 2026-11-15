"""
Test source structure of the canonical gripper ROS adapter.

Responsibility: Keep translation units reviewable and the executable entrypoint orchestration-only.
"""

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[2]
SOURCE_ROOT = PACKAGE_ROOT / 'src' / 'control'
DRIVER_SOURCES = (
    'panda_gripper_commands.cpp',
    'panda_gripper_driver_action.cpp',
    'panda_gripper_driver_config.cpp',
    'panda_gripper_driver_main.cpp',
    'panda_gripper_driver_monitor.cpp',
    'panda_gripper_driver_node.cpp',
)


def test_driver_translation_units_remain_reviewable() -> None:
    """Require another responsibility review before any source exceeds 300 lines."""
    for file_name in DRIVER_SOURCES:
        source_path = SOURCE_ROOT / file_name
        line_count = len(source_path.read_text(encoding='utf-8').splitlines())
        assert line_count < 300, (source_path, line_count)


def test_main_remains_process_orchestration_only() -> None:
    """Keep callbacks and ROS communication out of the executable entrypoint."""
    source = (SOURCE_ROOT / 'panda_gripper_driver_main.cpp').read_text(
        encoding='utf-8'
    )
    assert 'makePandaGripperDriverNode()' in source
    for forbidden_api in (
        'create_publisher',
        'create_server',
        'create_subscription',
        'create_wall_timer',
    ):
        assert forbidden_api not in source


def test_executable_compiles_every_driver_responsibility() -> None:
    """Prevent a split callback implementation from being omitted in CMake."""
    cmake = (
        PACKAGE_ROOT / 'cmake' / 'FrankaMotionControlTargets.cmake'
    ).read_text(encoding='utf-8')
    for file_name in DRIVER_SOURCES:
        assert f'src/control/{file_name}' in cmake
