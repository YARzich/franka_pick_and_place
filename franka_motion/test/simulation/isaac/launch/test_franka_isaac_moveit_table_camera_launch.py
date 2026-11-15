"""
Exercise the Isaac Sim table-camera and MoveIt compatibility launch.

Responsibility: Verify camera, planning, control, and shutdown contracts of the full scenario.
"""

from pathlib import Path
import sys
import unittest

import pytest
import rclpy

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'launch'))

from franka_launch_test_utils import (  # noqa: E402
    ISAAC_CAMERA_RUNTIME_TIMEOUT,
    assert_moveit_robot_ready,
    assert_stereo_camera_ready,
    assert_table_collision_ready,
    create_launch_description,
)


@pytest.mark.launch_test
def generate_test_description():
    launch_arguments = {
        'use_sim_time': 'true',
        'headless': 'true',
        'start_rviz': 'false',
        'mode': 'none',
        'time_steps': '0',
        'wait_for_clock': 'true',
        'point_cloud_stride': '8',
        'run_every_n_frames': '2',
    }
    return create_launch_description(
        'franka_isaac_moveit_table_camera.launch.py',
        launch_arguments,
    ), {}


class TestIsaacMoveItTableCameraLaunch(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_isaac_moveit_table_camera')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_isaac_moveit_table_camera_runtime(self):
        assert_moveit_robot_ready(self, self.node, ISAAC_CAMERA_RUNTIME_TIMEOUT)
        assert_table_collision_ready(
            self,
            self.node,
            ISAAC_CAMERA_RUNTIME_TIMEOUT,
            expected_dimensions=(2.0, 1.0, 0.1),
            expected_position=(0.0, 0.0, -0.052),
            expected_world_position=(0.0, 0.0, -0.052),
        )
        assert_stereo_camera_ready(self, self.node, ISAAC_CAMERA_RUNTIME_TIMEOUT)
