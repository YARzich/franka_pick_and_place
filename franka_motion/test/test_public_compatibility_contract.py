"""
Protect the supported public package surface after the V2 task migration.

Responsibility:
    Keep current executable, launch, interface, plugin, and resource contracts.
"""

import json
import os
from pathlib import Path
import subprocess
import sys
from xml.etree import ElementTree

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONTRACT_PATH = PACKAGE_ROOT / 'test' / 'contracts' / 'public_contract.json'


def _load_contract() -> dict:
    """Load the source-controlled compatibility manifest."""
    return json.loads(CONTRACT_PATH.read_text(encoding='utf-8'))


def _install_only_python_environment() -> dict[str, str]:
    """Drop repository paths so a child process can only import installed code."""
    environment = os.environ.copy()
    repository_root = PACKAGE_ROOT.parent.resolve()
    python_paths = []
    for raw_path in environment.get('PYTHONPATH', '').split(os.pathsep):
        if not raw_path:
            continue
        path = Path(raw_path).resolve()
        if path == repository_root or repository_root in path.parents:
            continue
        python_paths.append(raw_path)
    environment['PYTHONPATH'] = os.pathsep.join(python_paths)
    environment['PYTHONNOUSERSITE'] = '1'
    return environment


def test_installed_executable_and_launch_names_remain_available() -> None:
    """Ensure old ros2 run and ros2 launch basenames remain installed."""
    contract = _load_contract()
    prefix = Path(get_package_prefix(contract['package']))
    share = Path(get_package_share_directory(contract['package']))

    for executable_name in contract['executables']:
        executable = prefix / 'lib' / contract['package'] / executable_name
        assert executable.is_file(), f'missing compatibility executable: {executable}'
        assert os.access(executable, os.X_OK), f'not executable: {executable}'

    for launch_name in contract['launch_files']:
        launch_file = share / 'launch' / launch_name
        assert launch_file.is_file(), f'missing compatibility launch: {launch_file}'

    for resource_directory in contract['resource_directories']:
        resource_path = share / resource_directory
        assert resource_path.is_dir(), f'missing compatibility resources: {resource_path}'


def test_runtime_harness_imports_only_from_the_install_prefix(
    tmp_path: Path,
) -> None:
    """Prove the public CLI works without repository Python paths or cwd help."""
    prefix = Path(get_package_prefix('franka_motion')).resolve()
    environment = _install_only_python_environment()
    import_result = subprocess.run(
        [
            sys.executable,
            '-c',
            (
                'from pathlib import Path; '
                'import franka_motion_runtime; '
                'print(Path(franka_motion_runtime.__file__).resolve())'
            ),
        ],
        cwd=tmp_path,
        env=environment,
        check=True,
        capture_output=True,
        text=True,
    )
    imported_path = Path(import_result.stdout.strip())
    assert imported_path.is_relative_to(prefix), imported_path
    assert imported_path.parent.name == 'franka_motion_runtime'
    assert imported_path.name == '__init__.py'

    harness = prefix / 'lib/franka_motion/franka_runtime_harness'
    help_result = subprocess.run(
        [str(harness), '--help'],
        cwd=tmp_path,
        env=environment,
        check=False,
        capture_output=True,
        text=True,
    )
    assert help_result.returncode == 0, help_result.stderr
    assert 'Run bounded Franka runtime validation profiles.' in help_result.stdout


def test_transfer_sequence_v2_has_one_generic_public_client() -> None:
    """Configured and detected intents share one goal-only V2 client."""
    snapshot = _load_contract()
    contract = snapshot['transfer_sequence_v2']

    assert contract['server_executable'] == 'franka_transfer_sequence_v2_server'
    assert contract['public_client_executable'] == (
        'franka_transfer_sequence_client'
    )
    assert contract['implementation'] == (
        'scripts/tasks/franka_transfer_sequence_client'
    )
    assert contract['action_type'] == (
        'manipulation_msgs/action/TransferObjectSequenceV2'
    )
    assert contract['sequence_action'] == '/transfer_object_sequence_v2'
    assert contract['pause_service'] == '/set_transfer_object_sequence_v2_paused'
    assert contract['consumer_paths'] == ['configured', 'detected']
    assert contract['client_owned_step_fields'] == [
        'object_id',
        'target_mode',
        'target_object_pose',
        'target_translation',
        'return_home',
        'recovery_policy',
    ]
    assert set(contract['forbidden_goal_fields']) == {
        'geometry',
        'source_object_pose',
        'grasp_pose',
        'object_profile_id',
        'observation_provider_id',
        'grasp_provider_id',
        'strategy_id',
        'planner_backend_id',
        'whole_goal_timeout',
    }

    removed_executables = {
        'franka_moveit_pick_place',
        'franka_moveit_detected_pick_place',
    }
    assert removed_executables.isdisjoint(snapshot['executables'])
    assert 'fixed_cube_pick_place_adapter' not in snapshot
    assert 'detected_cube_pick_place_adapter' not in snapshot
    for executable in removed_executables:
        assert not (PACKAGE_ROOT / 'scripts' / 'tasks' / executable).exists()


def test_moveit_sequence_stays_a_low_level_diagnostic() -> None:
    """Do not present the arm-only MoveGroup smoke as an object workflow."""
    contract = _load_contract()['moveit_sequence_diagnostic']

    assert contract['public_executable'] == 'franka_moveit_sequence'
    assert contract['node_name'] == 'franka_moveit_sequence_node'
    assert contract['classification'] == 'low_level_moveit_diagnostic'
    assert contract['owns_task_workflow'] is False
    assert contract['stages'] == [
        'target_pose_1',
        'target_pose_2',
        'initial_joint_state',
    ]
    assert set(contract['parameters']) == {
        'planning_group',
        'reference_frame',
        'planning_time',
        'velocity_scaling',
        'acceleration_scaling',
        'current_state_timeout_sec',
        'joint_state_max_age',
    }


def test_static_table_scene_routes_only_to_coordinator() -> None:
    """Pin the public selector to the single coordinated implementation."""
    contract = _load_contract()
    route = contract['static_table_scene_routing']
    coordinator_target = route['coordinator_target']
    coordinator_prefix = Path(
        get_package_prefix(coordinator_target['package'])
    )

    assert route['public_executable'] == 'static_table_scene'
    assert route['parameter'] == 'use_scene_coordinator'
    assert route['default'] is True
    assert 'legacy_direct_target' not in route
    assert route['coordinator_target'] == {
        'package': 'franka_motion',
        'executable': 'static_table_scene_coordinated',
        'install_directory': 'libexec/franka_motion',
    }

    coordinator_executable = (
        coordinator_prefix
        / coordinator_target['install_directory']
        / coordinator_target['executable']
    )
    assert coordinator_executable.is_file(), coordinator_executable
    assert os.access(coordinator_executable, os.X_OK), coordinator_executable


def test_canonical_static_table_bootstrap_has_no_direct_writer() -> None:
    """Prevent the compatibility escape hatch from returning to scene code."""
    source = (
        PACKAGE_ROOT
        / 'src'
        / 'scene'
        / 'static_table_scene_node.cpp'
    ).read_text(encoding='utf-8')

    assert 'apply_direct' not in source
    assert 'ApplyPlanningScene' not in source
    assert 'apply_planning_scene_service' not in source


def test_legacy_simulator_resources_forward_to_canonical_owners() -> None:
    """Keep old package URIs valid without maintaining duplicate asset copies."""
    legacy_share = Path(get_package_share_directory('franka_motion'))
    resource_pairs = (
        (
            legacy_share / 'description' / 'table_camera.sdf',
            legacy_share / 'description' / 'gazebo' / 'table_camera.sdf',
        ),
        (
            legacy_share / 'worlds' / 'panda_empty.world.sdf',
            legacy_share / 'worlds' / 'gazebo' / 'panda_empty.world.sdf',
        ),
        (
            legacy_share / 'worlds' / 'table_world.sdf',
            legacy_share / 'worlds' / 'gazebo' / 'table_world.sdf',
        ),
        (
            legacy_share / 'worlds' / 'table_world_camera.sdf',
            legacy_share / 'worlds' / 'gazebo' / 'table_world_camera.sdf',
        ),
        (
            legacy_share / 'worlds' / 'table_world_usd.usd',
            legacy_share / 'worlds' / 'isaac' / 'table_world_usd.usd',
        ),
        (
            legacy_share / 'worlds' / 'table_world_usd_clean.usd',
            legacy_share / 'worlds' / 'isaac' / 'table_world_usd_clean.usd',
        ),
        (
            legacy_share / 'worlds' / 'FrankaPanda',
            legacy_share / 'worlds' / 'isaac' / 'FrankaPanda',
        ),
    )

    for legacy_path, canonical_path in resource_pairs:
        assert legacy_path.is_symlink(), legacy_path
        assert legacy_path.resolve() == canonical_path.resolve()

    assert not (legacy_share / 'worlds' / 'materials').exists()
    runner = legacy_share / 'scripts' / 'run_franka_isaac_sim.py'
    assert runner.is_file()
    assert os.access(runner, os.X_OK)


def test_table_camera_resources_are_owned_locally() -> None:
    """Keep historical paths while preventing external split-package owners."""
    package_share = Path(get_package_share_directory('franka_motion'))
    aliases = (
        (
            'config/franka_table_stereo_pointcloud.rviz',
            'config/rviz/franka_table_stereo_pointcloud.rviz',
        ),
        (
            'config/table_camera_perception.yaml',
            'config/perception/table_camera_perception.yaml',
        ),
    )
    for public_name, canonical_name in aliases:
        public_path = package_share / public_name
        canonical_path = package_share / canonical_name
        assert public_path.is_symlink(), public_path
        assert public_path.resolve() == canonical_path.resolve()

    for relative_path in (
        'models/cube_yolo26s_isaac_seg_v5s.pt',
        'models/foundationpose/cube_5cm.obj',
        'models/foundationpose/cube_5cm_texture.ppm',
    ):
        asset = package_share / relative_path
        assert asset.is_file(), asset
        assert not asset.is_symlink(), asset


def test_removed_curobo_action_and_parallel_lifecycle_stay_absent() -> None:
    """Keep planner selection behind TransferObjectSequenceV2 and MoveGroup."""
    contract = _load_contract()
    assert 'legacy_action' not in contract
    assert not any((PACKAGE_ROOT / 'action').glob('*'))
    assert not any((PACKAGE_ROOT / 'scripts' / 'curobo').glob('*.py'))
    for path in (
        PACKAGE_ROOT / 'src',
        PACKAGE_ROOT / 'franka_motion_runtime',
        PACKAGE_ROOT / 'launch',
    ):
        source = '\n'.join(
            item.read_text(encoding='utf-8', errors='ignore')
            for item in path.rglob('*')
            if item.is_file()
            and item.suffix in {'.cpp', '.hpp', '.py'}
        )
        assert 'CuRoboPlanPose' not in source
        assert '/plan_curobo_pose' not in source


def test_installed_tree_has_no_retired_headers_bytecode_or_metadata() -> None:
    """The installed tree must not expose the pre-V2 accidental surface."""
    prefix = Path(get_package_prefix('franka_motion'))
    retired_headers = {
        'detected_grasp_geometry.hpp',
        'detected_grasp_types.hpp',
        'detected_grasp_utils.hpp',
        'detected_pose_buffer.hpp',
        'moveit_parameter_snapshot_contract.hpp',
        'moveit_parameter_sync.hpp',
        'mtc_grasp_generator.hpp',
        'mtc_grasp_generator_command.hpp',
        'mtc_grasp_output.hpp',
        'mtc_grasp_pose_config.hpp',
        'mtc_grasp_task.hpp',
        'mtc_pose_policy.hpp',
    }
    installed_include = prefix / 'include' / 'franka_motion'
    assert not {
        header for header in retired_headers
        if (installed_include / header).exists()
    }

    launch_cache = (
        prefix / 'share' / 'franka_motion' / 'launch' / '__pycache__'
    )
    retired_launch_stems = (
        'franka_gazebo_curobo.launch',
        'franka_gazebo_curobo_table.launch',
        'franka_isaac_curobo.launch',
        'franka_isaac_curobo_table.launch',
    )
    for stem in retired_launch_stems:
        assert not tuple(launch_cache.glob(f'{stem}.cpython-*.pyc'))

    site_packages = tuple((prefix / 'lib').glob('python*/site-packages'))
    retired_distributions = (
        'franka_motion-0.1.0-*.egg-info',
        'franka_motion_runtime-0.1.0-*.egg-info',
        'isaac_runtime-0.1.0-*.egg-info',
    )
    for site_package in site_packages:
        for pattern in retired_distributions:
            assert not tuple(site_package.glob(pattern))
        runtime = site_package / 'franka_motion_runtime'
        assert not (runtime / 'bringup' / 'curobo').exists()
        assert not (
            runtime / 'bringup' / 'launch_parts' / 'camera' / 'motion.py'
        ).exists()
        assert not tuple(
            (runtime / 'bringup' / 'launch_parts' / 'camera' / '__pycache__')
            .glob('motion.cpython-*.pyc')
        )
        retired_task_modules = (
            'catalog',
            'detected_pick_place_client',
            'detected_pick_place_launch',
            'detected_pick_place_request',
            'legacy_pick_place_client',
            'legacy_pick_place_request',
            'resolution',
            'sequence_client_clock',
        )
        for module in retired_task_modules:
            assert not (runtime / 'tasks' / f'{module}.py').exists()
            assert not tuple(
                (runtime / 'tasks' / '__pycache__').glob(
                    f'{module}.cpython-*.pyc'
                )
            )


def test_curobo_scenario_configs_have_one_local_owner() -> None:
    """Keep only Panda values local while the generic plugin owns planning."""
    scenario_share = Path(get_package_share_directory('franka_motion'))
    config_directory = scenario_share / 'config' / 'curobo'
    assert {
        path.name for path in config_directory.glob('*.yaml')
    } == {'panda_curobo_motion_gen.yaml'}
    planning = yaml.safe_load(
        (scenario_share / 'config/moveit/curobo_planning.yaml').read_text(
            encoding='utf-8'
        )
    )
    assert planning['planning_plugins'] == [
        'manipulation_curobo/CuRoboPlanner'
    ]
    assert planning['motion_gen_config'] == (
        'package://franka_motion/config/curobo/'
        'panda_curobo_motion_gen.yaml'
    )
    public_pipeline = scenario_share / 'config/curobo_planning.yaml'
    assert public_pipeline.is_symlink()
    assert public_pipeline.resolve() == (
        scenario_share / 'config/moveit/curobo_planning.yaml'
    ).resolve()

    package_prefix = Path(get_package_prefix('franka_motion'))
    stale_plugin_index = (
        package_prefix
        / 'share'
        / 'ament_index'
        / 'resource_index'
        / 'moveit_core__pluginlib__plugin'
        / 'franka_motion'
    )
    assert not stale_plugin_index.exists(), stale_plugin_index

    assert not (
        PACKAGE_ROOT / 'include' / 'franka_motion' / 'curobo_planner.hpp'
    ).exists()
    installed_include = (
        Path(get_package_prefix('franka_motion'))
        / 'include'
        / 'franka_motion'
        / 'curobo_planner.hpp'
    )
    assert not installed_include.exists(), installed_include


def test_curobo_backend_dependency_is_generic_and_runtime_only() -> None:
    """Discover the optional plugin at runtime without a package dependency."""
    manifest = ElementTree.parse(PACKAGE_ROOT / 'package.xml').getroot()
    dependency_tags = (
        'depend',
        'build_depend',
        'build_export_depend',
        'exec_depend',
    )
    dependencies = {
        element.text.strip()
        for tag in dependency_tags
        for element in manifest.findall(tag)
        if element.text
    }
    assert 'manipulation_curobo' not in dependencies
    assert 'franka_motion_curobo' not in dependencies
    assert 'franka_motion_curobo_bringup' not in dependencies

    cmake = (PACKAGE_ROOT / 'CMakeLists.txt').read_text(encoding='utf-8')
    assert 'find_package(manipulation_curobo' not in cmake


def test_gripper_config_mirror_matches_local_canonical_profile() -> None:
    """Prevent the historical YAML path from drifting from its local owner."""
    package_share = Path(get_package_share_directory('franka_motion'))
    legacy = yaml.safe_load(
        (package_share / 'config' / 'panda_gripper_driver.yaml').read_text(
            encoding='utf-8'
        )
    )
    canonical = yaml.safe_load(
        (
            package_share / 'config' / 'control' / 'panda_gripper_driver.yaml'
        ).read_text(
            encoding='utf-8'
        )
    )

    assert legacy == canonical


def test_ros2_control_config_mirrors_match_local_profiles_semantically() -> None:
    """Keep public controller paths semantically equal to their local owner."""
    public_config = (
        Path(get_package_share_directory('franka_motion')) / 'config'
    )
    canonical_config = public_config / 'control'
    file_names = (
        'panda_ros2_control_base.yaml',
        'panda_ros2_control_gazebo.yaml',
        'panda_ros2_control_isaac.yaml',
        'initial_positions.yaml',
    )

    for file_name in file_names:
        public_profile = yaml.safe_load(
            (public_config / file_name).read_text(encoding='utf-8')
        )
        canonical_profile = yaml.safe_load(
            (canonical_config / file_name).read_text(encoding='utf-8')
        )
        assert public_profile == canonical_profile


def test_moveit_config_mirrors_match_local_canonical_profile_exactly() -> None:
    """Keep historical MoveIt paths synchronized with the local canonical profile."""
    public_config = Path(get_package_share_directory('franka_motion')) / 'config'
    canonical_config = public_config / 'moveit'
    file_pairs = (
        ('panda_no_virtual_joint.srdf', 'panda.srdf'),
        ('panda_kinematics.yaml', 'kinematics.yaml'),
        (
            'panda_moveit_isaac_trajectory_execution.yaml',
            'isaac_trajectory_execution.yaml',
        ),
    )

    for legacy_name, canonical_name in file_pairs:
        legacy_bytes = (public_config / legacy_name).read_bytes()
        canonical_bytes = (canonical_config / canonical_name).read_bytes()
        assert legacy_bytes == canonical_bytes, (
            f'compatibility mirror drifted: {legacy_name}'
        )
