"""
Exercise perception health filtering and freshness over ROS interfaces.

Responsibility: Verify exact diagnostic selection, steady-clock expiry, and shutdown.
"""

import time
import unittest

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
from manipulation_msgs.msg import HealthComponentStatus, SystemHealthSnapshot
import pytest
import rclpy


_TASK_NAME = 'perception/stereo_depth_pointcloud'
_PRIMARY_STATUS_NAME = 'stereo_sgbm_pointcloud: perception/stereo_depth_pointcloud'
_ALTERNATIVE_STATUS_NAME = 'ess_depth_pointcloud: perception/stereo_depth_pointcloud'


@pytest.mark.launch_test
def generate_test_description():
    """Start one required task with two exact alternative producer bindings."""
    monitor_node = launch_ros.actions.Node(
        package='manipulation',
        executable='system_health_monitor',
        name='test_perception_health_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'startup_grace_sec': 0.0,
            'require_sim_clock': False,
            'joint_states_topic': '/test_perception_health/no_joint_states',
            'controller_manager_service': '/test_perception_health/no_controllers',
            'motion_backend_action': '/test_perception_health/no_motion_backend',
            'motion_backend_action_type': 'manipulation_msgs/action/TransferObjectSequenceV2',
            'gripper_action': '/test_perception_health/no_gripper',
            'planning_scene_state_service': '/test_perception_health/no_scene',
            'base_frame': 'test_perception_health_base',
            'end_effector_frame': 'test_perception_health_tool',
            'required_joint_names': ['test_joint'],
            'required_controllers': ['test_arm_controller'],
            'perception_diagnostics.allowed_tasks': [_TASK_NAME, _TASK_NAME],
            'perception_diagnostics.component_ids': [
                HealthComponentStatus.COMPONENT_STEREO_PERCEPTION,
                HealthComponentStatus.COMPONENT_STEREO_PERCEPTION,
            ],
            'perception_diagnostics.status_names': [
                _PRIMARY_STATUS_NAME,
                _ALTERNATIVE_STATUS_NAME,
            ],
            'perception_diagnostics.required_tasks': [_TASK_NAME],
            'perception_diagnostics.warn_after_sec': 0.2,
            'perception_diagnostics.stale_after_sec': 0.5,
            'diagnostic_updater.period': 0.1,
            'health_publish_period_sec': 0.1,
        }],
    )
    description = launch.LaunchDescription()
    description.add_action(monitor_node)
    description.add_action(launch_testing.actions.ReadyToTest())
    return description, {'monitor_node': monitor_node}


class TestPerceptionHealthAdapter(unittest.TestCase):
    """Drive the standard bus and inspect only the typed health component."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('perception_health_contract_test')
        cls.health_messages = []
        cls.health_subscription = cls.node.create_subscription(
            SystemHealthSnapshot,
            '/test_perception_health_monitor/health',
            cls.health_messages.append,
            10,
        )
        cls.diagnostics_publisher = cls.node.create_publisher(
            DiagnosticArray,
            '/diagnostics',
            10,
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _component(self):
        if not self.health_messages:
            return None
        return next(
            (
                component
                for component in self.health_messages[-1].components
                if component.component_id
                == HealthComponentStatus.COMPONENT_STEREO_PERCEPTION
            ),
            None,
        )

    def _spin_until(self, predicate, timeout, failure_message, publish=None):
        deadline = time.monotonic() + timeout
        while not predicate():
            if publish is not None:
                publish()
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                self.fail(failure_message)
            rclpy.spin_once(self.node, timeout_sec=min(0.05, remaining))

    def _publish_many(self, entries):
        array = DiagnosticArray()
        # ROS time is deliberately unrelated to steady receipt time.
        array.header.stamp.sec = 1
        for name, level, message in entries:
            status = DiagnosticStatus()
            status.name = name
            status.level = level
            status.message = message
            array.status.append(status)
        self.diagnostics_publisher.publish(array)

    def _publish(self, name, level, message):
        self._publish_many([(name, level, message)])

    def test_alternative_exact_producers_share_one_typed_component(self):
        self._spin_until(
            lambda: self.diagnostics_publisher.get_subscription_count() > 0,
            5.0,
            'monitor did not subscribe to /diagnostics',
        )
        self._spin_until(
            lambda: self._component() is not None,
            5.0,
            'typed perception component was not published',
        )

        component = self._component()
        stereo_components = [
            item
            for item in self.health_messages[-1].components
            if item.component_id
            == HealthComponentStatus.COMPONENT_STEREO_PERCEPTION
        ]
        self.assertEqual(len(stereo_components), 1)
        self.assertTrue(component.required)
        self.assertFalse(component.observed)
        self.assertIn(
            HealthComponentStatus.ISSUE_PERCEPTION_DIAGNOSTIC_UNAVAILABLE,
            component.issue_codes,
        )

        initial_sequence = self.health_messages[-1].sequence
        self._spin_until(
            lambda: self.health_messages[-1].sequence >= initial_sequence + 3,
            2.0,
            'health did not remain unobserved across disallowed diagnostics',
            publish=lambda: self._publish_many([
                (_TASK_NAME, DiagnosticStatus.OK, 'missing node prefix'),
                (
                    f'foreign: {_ALTERNATIVE_STATUS_NAME}',
                    DiagnosticStatus.OK,
                    'extra prefix',
                ),
                (
                    f'{_ALTERNATIVE_STATUS_NAME}/suffix',
                    DiagnosticStatus.OK,
                    'extra suffix',
                ),
            ]),
        )
        self.assertFalse(self._component().observed)

        self._spin_until(
            lambda: (
                self._component() is not None
                and self._component().observed
                and self._component().level == HealthComponentStatus.LEVEL_OK
            ),
            5.0,
            'alternative stereo producer diagnostic was not accepted',
            publish=lambda: self._publish(
                _ALTERNATIVE_STATUS_NAME, DiagnosticStatus.OK, 'ready'
            ),
        )

        self._spin_until(
            lambda: (
                self._component().level == HealthComponentStatus.LEVEL_WARN
                and HealthComponentStatus.ISSUE_PERCEPTION_PIPELINE_REPORTED_WARN
                in self._component().issue_codes
            ),
            5.0,
            'primary stereo producer WARN was not mapped into typed health',
            publish=lambda: self._publish(
                _PRIMARY_STATUS_NAME, DiagnosticStatus.WARN, 'processing delayed'
            ),
        )

        self._spin_until(
            lambda: (
                self._component().level == HealthComponentStatus.LEVEL_STALE
                and HealthComponentStatus.ISSUE_PERCEPTION_DIAGNOSTIC_STALE
                in self._component().issue_codes
            ),
            5.0,
            'perception freshness did not become stale on steady time',
        )


@launch_testing.post_shutdown_test()
class TestPerceptionHealthShutdown(unittest.TestCase):
    """Verify the monitor exits cleanly after the launch contract."""

    def test_exit_code(self, proc_info, monitor_node):
        launch_testing.asserts.assertExitCodes(proc_info, process=monitor_node)
