"""
Test semantic contracts of canonical Panda ros2_control profiles.

Responsibility: Preserve controller and joint identity, backend tolerances, and initial positions.
"""

from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[2]
CONFIG_ROOT = PACKAGE_ROOT / 'config' / 'control'
ARM_JOINTS = [f'panda_joint{index}' for index in range(1, 8)]
HAND_JOINTS = ['panda_finger_joint1', 'panda_finger_joint2']
CONTROLLER_TYPES = {
    'joint_state_broadcaster':
        'joint_state_broadcaster/JointStateBroadcaster',
    'panda_arm_controller':
        'joint_trajectory_controller/JointTrajectoryController',
    'panda_hand_controller':
        'position_controllers/JointGroupPositionController',
}


def _load(file_name: str) -> dict:
    """Parse one canonical YAML resource."""
    return yaml.safe_load(
        (CONFIG_ROOT / file_name).read_text(encoding='utf-8')
    )


def test_base_profile_only_sets_controller_manager_rate() -> None:
    """Keep the xacro plugin profile free of simulator controller details."""
    assert _load('panda_ros2_control_base.yaml') == {
        'controller_manager': {
            'ros__parameters': {
                'update_rate': 100,
            },
        },
    }


def test_simulator_profiles_keep_controller_and_joint_identity() -> None:
    """Preserve the controller API shared by Gazebo and Isaac backends."""
    for file_name in (
        'panda_ros2_control_gazebo.yaml',
        'panda_ros2_control_isaac.yaml',
    ):
        config = _load(file_name)
        manager = config['controller_manager']['ros__parameters']
        assert manager['use_sim_time'] is True
        assert manager['update_rate'] == 100
        assert {
            name: manager[name]['type']
            for name in CONTROLLER_TYPES
        } == CONTROLLER_TYPES

        arm = config['panda_arm_controller']['ros__parameters']
        assert arm['joints'] == ARM_JOINTS
        assert arm['command_interfaces'] == ['position']
        assert arm['state_interfaces'] == ['position', 'velocity']
        assert arm['allow_partial_joints_goal'] is False

        hand = config['panda_hand_controller']['ros__parameters']
        assert hand['joints'] == HAND_JOINTS
        assert hand['command_interfaces'] == ['position']


def test_backend_specific_controller_tolerances_remain_explicit() -> None:
    """Keep both simulator endpoints strict while preserving path policy."""
    gazebo = _load('panda_ros2_control_gazebo.yaml')
    gazebo_manager = gazebo['controller_manager']['ros__parameters']
    gazebo_arm = gazebo['panda_arm_controller']['ros__parameters']
    assert 'switch_timeout' not in gazebo_manager
    assert gazebo_arm['interface_name'] == 'position'
    assert gazebo_arm['constraints'] == {
        'goal_time': 0.5,
        'stopped_velocity_tolerance': 0.01,
    }
    assert gazebo_arm['allow_nonzero_velocity_at_trajectory_end'] is True

    isaac = _load('panda_ros2_control_isaac.yaml')
    isaac_manager = isaac['controller_manager']['ros__parameters']
    isaac_arm = isaac['panda_arm_controller']['ros__parameters']
    assert isaac_manager['switch_timeout'] == 15.0
    assert isaac_arm['allow_nonzero_velocity_at_trajectory_end'] is True
    assert isaac_arm['constraints']['goal_time'] == 10.0
    assert isaac_arm['constraints']['stopped_velocity_tolerance'] == 0.01
    for joint_name in ARM_JOINTS:
        assert isaac_arm['constraints'][joint_name] == {
            'trajectory': 0.5,
            'goal': 0.01,
        }


def test_initial_positions_cover_exactly_the_arm_contract() -> None:
    """Keep a deterministic seven-joint startup pose for both simulators."""
    positions = _load('initial_positions.yaml')['initial_positions']
    assert positions == {
        'panda_joint1': 0.0,
        'panda_joint2': -0.785,
        'panda_joint3': 0.0,
        'panda_joint4': -2.356,
        'panda_joint5': 0.0,
        'panda_joint6': 1.571,
        'panda_joint7': 0.785,
    }
