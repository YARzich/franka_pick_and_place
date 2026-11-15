"""
Test scenario-owned gripper limits against the canonical driver profile.

Responsibility: Keep Gazebo command clamping compatible with the backend-neutral
gripper action adapter without duplicating the core controller-profile tests.
"""

from pathlib import Path
import sys

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[2]
CONFIG_ROOT = PACKAGE_ROOT / 'config' / 'control'
sys.path.insert(0, str(PACKAGE_ROOT / 'launch'))

from franka_motion_parts.constants import GAZEBO_GRIPPER_COMMAND_LIMITS  # noqa: E402


def test_gripper_driver_keeps_backend_neutral_defaults_and_gazebo_limits():
    """Keep simulator clamping outside the generic position-only adapter."""
    params = _load('panda_gripper_driver.yaml')[
        'panda_gripper_driver'
    ]['ros__parameters']

    assert 'command_joint_min_position' not in params
    assert 'command_joint_max_position' not in params
    assert params['finger_symmetry_tolerance'] < 0.04
    assert 0.0 <= params['stall_finger_reverse_tolerance'] < 0.04
    assert GAZEBO_GRIPPER_COMMAND_LIMITS == {
        'command_joint_min_position': 0.001,
        'command_joint_max_position': 0.039,
    }


def _load(file_name: str) -> dict:
    """Load one canonical control profile."""
    return yaml.safe_load(
        (CONFIG_ROOT / file_name).read_text(encoding='utf-8')
    )
