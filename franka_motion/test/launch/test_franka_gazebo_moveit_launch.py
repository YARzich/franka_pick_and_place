"""
Exercise the empty-world Gazebo and MoveIt compatibility launch.

Responsibility: Verify the launched graph, controller readiness, motion, and clean shutdown.
"""

from pathlib import Path
import sys
import unittest

import pytest
import rclpy

sys.path.insert(0, str(Path(__file__).resolve().parent))

from franka_launch_test_utils import (  # noqa: E402
    DEFAULT_RUNTIME_TIMEOUT,
    assert_moveit_robot_ready,
    create_launch_description,
)


@pytest.mark.launch_test
def generate_test_description():
    launch_arguments = {
        'use_sim_time': 'true',
        'headless': 'true',
    }
    return create_launch_description(
        'franka_gazebo_moveit.launch.py',
        launch_arguments,
    ), {}


class TestGazeboMoveItLaunch(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_gazebo_moveit')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_gazebo_moveit_robot_starts_in_default_state(self):
        assert_moveit_robot_ready(self, self.node, DEFAULT_RUNTIME_TIMEOUT)
