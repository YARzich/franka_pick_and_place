"""
Exercise the Gazebo table-camera and MoveIt compatibility launch.

Responsibility: Verify camera, planning, control, and shutdown contracts of the full scenario.
"""

from pathlib import Path
import sys
import unittest

import pytest
import rclpy

sys.path.insert(0, str(Path(__file__).resolve().parent))

from franka_launch_test_utils import (  # noqa: E402
    CAMERA_RUNTIME_TIMEOUT,
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
        'point_cloud_stride': '8',
        'run_every_n_frames': '2',
    }
    return create_launch_description(
        'franka_gazebo_moveit_table_camera.launch.py',
        launch_arguments,
    ), {}


class TestGazeboMoveItTableCameraLaunch(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_gazebo_moveit_table_camera')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_gazebo_moveit_table_camera_runtime(self):
        assert_moveit_robot_ready(self, self.node, CAMERA_RUNTIME_TIMEOUT)
        assert_table_collision_ready(
            self,
            self.node,
            CAMERA_RUNTIME_TIMEOUT,
            expected_dimensions=(1.5532, 2.3937, 0.05),
            expected_position=(0.6669, 0.0, -0.027),
            expected_world_position=(0.6669, 0.0, 0.973),
        )
        assert_stereo_camera_ready(self, self.node, CAMERA_RUNTIME_TIMEOUT)
