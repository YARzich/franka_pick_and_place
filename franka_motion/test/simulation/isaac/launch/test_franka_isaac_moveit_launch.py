"""
Exercise the empty-world Isaac Sim and MoveIt compatibility launch.

Responsibility: Verify the launched graph, controller readiness, motion, and clean shutdown.
"""

from pathlib import Path
import sys
import unittest

import pytest
import rclpy

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'launch'))

from franka_launch_test_utils import (  # noqa: E402
    ISAAC_RUNTIME_TIMEOUT,
    assert_moveit_robot_ready,
    create_launch_description,
)


@pytest.mark.launch_test
def generate_test_description():
    launch_arguments = {
        'use_sim_time': 'true',
        'headless': 'true',
        'time_steps': '0',
        'wait_for_clock': 'true',
    }
    return create_launch_description(
        'franka_isaac_moveit.launch.py',
        launch_arguments,
    ), {}


class TestIsaacMoveItLaunch(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_isaac_moveit')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_isaac_moveit_robot_starts_in_default_state(self):
        assert_moveit_robot_ready(self, self.node, ISAAC_RUNTIME_TIMEOUT)
