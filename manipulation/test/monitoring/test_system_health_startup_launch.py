"""
Exercise typed health startup behavior without live robot dependencies.

Responsibility: Verify startup grace, initial component state, and fail-closed transitions.
"""

import time
import unittest

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
from manipulation_msgs.msg import HealthComponentStatus
from manipulation_msgs.msg import SystemHealthSnapshot
import pytest
import rclpy


@pytest.mark.launch_test
def generate_test_description():
    """Start an isolated monitor whose configured dependencies have not appeared."""
    monitor_node = launch_ros.actions.Node(
        package='manipulation',
        executable='system_health_monitor',
        name='startup_health_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'startup_grace_sec': 30.0,
            'require_sim_clock': True,
            'joint_states_topic': '/typed_health_startup/no_joint_states',
            'controller_manager_service': '/typed_health_startup/no_controllers',
            'planning_scene_state_service': '/typed_health_startup/no_scene',
            'motion_backend_action': '/typed_health_startup/no_motion_backend',
            'motion_backend_action_type': 'manipulation_msgs/action/TransferObjectSequenceV2',
            'gripper_action': '/typed_health_startup/no_gripper',
            'base_frame': 'typed_health_startup_base',
            'end_effector_frame': 'typed_health_startup_tool',
            'required_joint_names': ['test_joint'],
            'required_controllers': ['test_arm_controller'],
            'diagnostic_updater.period': 0.1,
            'health_publish_period_sec': 0.1,
        }],
    )

    launch_description = launch.LaunchDescription()
    launch_description.add_action(monitor_node)
    launch_description.add_action(launch_testing.actions.ReadyToTest())
    return launch_description, {'monitor_node': monitor_node}


class TestSystemHealthStartup(unittest.TestCase):
    """Check that startup WARN is typed and never advertised as ready."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('system_health_startup_contract_test')
        cls.latest_health = None
        cls.statuses = {}
        cls.health_subscription = cls.node.create_subscription(
            SystemHealthSnapshot,
            '/startup_health_monitor/health',
            cls._health_callback,
            10,
        )
        cls.diagnostics_subscription = cls.node.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            cls._diagnostics_callback,
            10,
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _health_callback(cls, message):
        cls.latest_health = message

    @classmethod
    def _diagnostics_callback(cls, message):
        for status in message.status:
            owner, separator, task = status.name.rpartition(': ')
            if separator and owner.endswith('startup_health_monitor'):
                cls.statuses[task] = status

    def _spin_until(self, predicate, timeout, failure_message):
        deadline = time.monotonic() + timeout
        while not predicate():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                self.fail(failure_message)
            rclpy.spin_once(self.node, timeout_sec=min(0.05, remaining))

    def test_unseen_dependencies_are_startup_warn(self):
        self._spin_until(
            lambda: self.latest_health is not None,
            10.0,
            'did not receive startup health snapshot',
        )
        snapshot = self.latest_health
        components = {
            component.component_id: component
            for component in snapshot.components
        }

        self.assertFalse(snapshot.motion_ready)
        self.assertEqual(
            snapshot.overall_level,
            HealthComponentStatus.LEVEL_WARN,
        )
        expected_stream_issues = {
            HealthComponentStatus.COMPONENT_JOINT_STATES:
                HealthComponentStatus.ISSUE_JOINT_STATES_UNAVAILABLE,
            HealthComponentStatus.COMPONENT_SIMULATION_CLOCK:
                HealthComponentStatus.ISSUE_SIMULATION_CLOCK_UNAVAILABLE,
            HealthComponentStatus.COMPONENT_CONTROLLERS:
                HealthComponentStatus.ISSUE_CONTROLLER_STATE_UNAVAILABLE,
            HealthComponentStatus.COMPONENT_PLANNING_SCENE:
                HealthComponentStatus.ISSUE_PLANNING_SCENE_STATE_UNAVAILABLE,
        }
        for component_id, issue in expected_stream_issues.items():
            component = components[component_id]
            self.assertEqual(component.level, HealthComponentStatus.LEVEL_WARN)
            self.assertFalse(component.observed)
            self.assertTrue(component.startup_grace_active)
            self.assertEqual(
                component.age_semantics,
                HealthComponentStatus.AGE_SINCE_MONITOR_START,
            )
            self.assertIn(issue, component.issue_codes)

        expected_probe_issues = {
            HealthComponentStatus.COMPONENT_ROBOT_TF:
                HealthComponentStatus.ISSUE_ROBOT_TF_UNAVAILABLE,
            # Wire constants keep their legacy names while the API is neutral.
            HealthComponentStatus.COMPONENT_MOVE_GROUP:
                HealthComponentStatus.ISSUE_MOVE_GROUP_UNAVAILABLE,
            HealthComponentStatus.COMPONENT_GRIPPER:
                HealthComponentStatus.ISSUE_GRIPPER_UNAVAILABLE,
        }
        for component_id, issue in expected_probe_issues.items():
            component = components[component_id]
            self.assertEqual(component.level, HealthComponentStatus.LEVEL_WARN)
            self.assertTrue(component.observed)
            self.assertEqual(
                component.age_semantics,
                HealthComponentStatus.AGE_NOT_APPLICABLE,
            )
            self.assertIn(issue, component.issue_codes)

    def test_standard_diagnostics_use_the_same_startup_levels(self):
        expected_names = {
            'Joint states', 'Simulation clock', 'Controllers',
            'Robot TF', 'Motion backend action', 'Gripper action',
            'PlanningScene coordinator',
        }
        self._spin_until(
            lambda: expected_names.issubset(self.statuses),
            10.0,
            f'did not receive startup diagnostics; latest={self.statuses}',
        )
        for name in expected_names:
            self.assertEqual(self.statuses[name].level, DiagnosticStatus.WARN)
