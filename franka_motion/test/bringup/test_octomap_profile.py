"""
Test scenario-owned MoveIt occupancy-map profiles statically.

Responsibility: Verify profile ownership, TF admission, collision links, and leaf selection.
"""

from pathlib import Path
import re
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory

from franka_motion_runtime.bringup.launch_parts.moveit import (
    OCTOMAP_TF_REQUIRED_FRAMES,
    OctomapProfile,
)
from franka_motion_runtime.simulation.isaac.scenarios import IsaacMoveItScenario


_PACKAGE_ROOT = Path(__file__).resolve().parents[2]
_LAUNCH_DIRECTORY = _PACKAGE_ROOT / 'launch'
_DESCRIPTION_SHARE = Path(
    get_package_share_directory('franka_motion')
)


def test_profile_resources_are_explicit_and_owned_by_moveit_config():
    """The MoveIt config must own both explicit sensor-profile resources."""
    assert OctomapProfile.DISABLED.value == (
        'config/moveit/sensors_3d_disabled.yaml'
    )
    assert OctomapProfile.TABLE_CAMERA.value == (
        'config/moveit/sensors_3d_table_camera.yaml'
    )
    moveit_source = (
        _PACKAGE_ROOT / 'franka_motion_runtime' / 'bringup' /
        'launch_parts' / 'moveit.py'
    ).read_text(encoding='utf-8')
    assert moveit_source.count(
        '.sensors_3d(file_path=octomap_profile.value)'
    ) == 2


def test_each_leaf_selects_exactly_one_scenario_profile():
    """Every public MoveIt leaf must choose one occupancy-map policy."""
    expected = {
        'franka_gazebo_moveit.launch.py': 'DISABLED',
        'franka_gazebo_moveit_table.launch.py': 'DISABLED',
        'franka_gazebo_moveit_table_camera.launch.py': 'TABLE_CAMERA',
        'franka_isaac_moveit.launch.py': 'DISABLED',
        'franka_isaac_moveit_table.launch.py': 'DISABLED',
    }
    for launch_file, profile_name in expected.items():
        source = (_LAUNCH_DIRECTORY / launch_file).read_text(encoding='utf-8')
        selected_profiles = set(
            re.findall(r'octomap_profile=OctomapProfile\.(\w+)', source)
        )
        assert selected_profiles == {profile_name}

    isaac_camera_source = (
        _PACKAGE_ROOT / 'franka_motion_runtime' / 'bringup' /
        'isaac_table_camera_moveit.py'
    ).read_text(encoding='utf-8')
    selected_profiles = set(
        re.findall(
            r'octomap_profile=OctomapProfile\.(\w+)',
            isaac_camera_source,
        )
    )
    assert selected_profiles == {'TABLE_CAMERA'}


def test_table_camera_profiles_start_the_exact_stamp_tf_gate():
    """Every table-camera MoveIt backend must protect its Octomap input."""
    gazebo_source = (
        _LAUNCH_DIRECTORY / 'franka_gazebo_moveit_table_camera.launch.py'
    ).read_text(encoding='utf-8')
    isaac_source = (
        _PACKAGE_ROOT / 'franka_motion_runtime' / 'bringup' /
        'isaac_table_camera_moveit.py'
    ).read_text(encoding='utf-8')

    assert gazebo_source.count('create_octomap_tf_gate_node(') == 1
    assert isaac_source.count('create_octomap_tf_gate_node(') == 1


def test_table_camera_leaves_use_v2_scene_snapshot_without_v1_barrier():
    """Camera leaves use the V2 scene adapter and expose no retired V1 knob."""
    gazebo_source = (
        _LAUNCH_DIRECTORY / 'franka_gazebo_moveit_table_camera.launch.py'
    ).read_text(encoding='utf-8')
    isaac_source = (
        _PACKAGE_ROOT / 'franka_motion_runtime' / 'bringup' /
        'isaac_table_camera_moveit.py'
    ).read_text(encoding='utf-8')

    for source in (gazebo_source, isaac_source):
        assert source.count('create_transfer_sequence_server_node(') == 1
        assert source.count('create_servo_task_server_node(') == 1
        assert 'create_transfer_server_node(' not in source
        assert 'occupancy_map_enabled' not in source


def test_octomap_gate_leaves_cover_every_collision_link():
    """Keep the gate's leaf-frame contract aligned with the Panda model."""
    result = subprocess.run(
        [
            'xacro',
            str(
                _DESCRIPTION_SHARE / 'description' / 'urdf' /
                'panda_nomimic.urdf.xacro'
            ),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    robot = ET.fromstring(result.stdout)
    links = {link.attrib['name'] for link in robot.findall('link')}
    collision_links = {
        link.attrib['name']
        for link in robot.findall('link')
        if link.find('collision') is not None
    }
    parents = {}
    for joint in robot.findall('joint'):
        child = joint.find('child')
        parent = joint.find('parent')
        assert child is not None and parent is not None
        child_name = child.attrib['link']
        assert child_name not in parents, f'duplicate parent for {child_name}'
        parents[child_name] = parent.attrib['link']

    covered_links = set()
    for leaf in OCTOMAP_TF_REQUIRED_FRAMES:
        assert leaf in links, f'unknown Octomap TF leaf: {leaf}'
        chain = set()
        current = leaf
        while True:
            assert current not in chain, f'cycle in robot tree at {current}'
            chain.add(current)
            covered_links.add(current)
            if current not in parents:
                break
            current = parents[current]

    uncovered = collision_links - covered_links
    assert not uncovered, (
        'Octomap TF gate does not cover collision links: '
        f'{sorted(uncovered)}'
    )


def test_simulator_scenario_has_no_moveit_monitor_ownership():
    """Simulator policy must not absorb MoveIt monitor configuration."""
    assert 'use_planning_scene_monitor' not in IsaacMoveItScenario.__dataclass_fields__
