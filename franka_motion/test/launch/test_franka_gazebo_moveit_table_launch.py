"""
Exercise the Gazebo table and MoveIt compatibility launch.

Responsibility: Verify table-scene planning, controller readiness, motion, and clean shutdown.
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
    assert_table_collision_ready,
    create_launch_description,
)


@pytest.mark.launch_test
def generate_test_description():
    launch_arguments = {
        'use_sim_time': 'true',
        'headless': 'true',
    }
    return create_launch_description(
        'franka_gazebo_moveit_table.launch.py',
        launch_arguments,
    ), {}


class TestGazeboMoveItTableLaunch(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_gazebo_moveit_table')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_gazebo_moveit_table_robot_starts_in_default_state(self):
        assert_moveit_robot_ready(self, self.node, DEFAULT_RUNTIME_TIMEOUT)
        assert_table_collision_ready(
            self,
            self.node,
            DEFAULT_RUNTIME_TIMEOUT,
            expected_dimensions=(1.5532, 2.3937, 0.05),
            expected_position=(0.6669, 0.0, -0.027),
            expected_world_position=(0.6669, 0.0, 0.973),
        )
