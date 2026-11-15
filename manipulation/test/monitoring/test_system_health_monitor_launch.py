"""
Exercise the system-health ROS adapter against live diagnostic messages.

Responsibility: Verify typed component status and diagnostic-level publication.
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
from manipulation_msgs.srv import GetSystemHealth
import pytest
import rclpy
from sensor_msgs.msg import JointState


@pytest.mark.launch_test
def generate_test_description():
    """Start the monitor with explicit robot bindings and deterministic faults."""
    monitor_node = launch_ros.actions.Node(
        package='manipulation',
        executable='system_health_monitor',
        name='test_system_health_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'startup_grace_sec': 0.0,
            'require_sim_clock': False,
            'joint_states_topic': '/test_health/joint_states',
            'controller_manager_service': '/test_health/no_controllers',
            'motion_backend_action': '/test_health/no_motion_backend',
            'motion_backend_action_type': 'manipulation_msgs/action/TransferObjectSequenceV2',
            'gripper_action': '/test_health/no_gripper',
            'planning_scene_state_service': '/test_health/no_scene',
            'base_frame': 'test_health_base',
            'end_effector_frame': 'test_health_tool',
            'required_joint_names': ['health_test_joint'],
            'required_controllers': ['test_arm_controller'],
            'fault_injection.force_joint_states_stale': True,
            'fault_injection.force_clock_stale': True,
            'fault_injection.force_controllers_unavailable': True,
            'fault_injection.force_tf_unavailable': True,
            'fault_injection.force_motion_backend_unavailable': True,
            'fault_injection.force_gripper_unavailable': True,
            'fault_injection.force_planning_scene_unavailable': True,
            'fault_injection.force_perception_unavailable': True,
            'diagnostic_updater.period': 0.1,
            'health_publish_period_sec': 0.1,
        }],
    )

    launch_description = launch.LaunchDescription()
    launch_description.add_action(monitor_node)
    launch_description.add_action(launch_testing.actions.ReadyToTest())
    return launch_description, {'monitor_node': monitor_node}


class TestSystemHealthMonitor(unittest.TestCase):
    """Inspect the standard and typed health contracts from the launched node."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('system_health_monitor_contract_test')
        cls.statuses = {}
        cls.health_messages = []
        cls.subscription = cls.node.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            cls._diagnostics_callback,
            10,
        )
        cls.health_subscription = cls.node.create_subscription(
            SystemHealthSnapshot,
            '/test_system_health_monitor/health',
            cls._health_callback,
            10,
        )
        cls.health_client = cls.node.create_client(
            GetSystemHealth,
            '/test_system_health_monitor/get_health',
        )
        cls.joint_state_publisher = cls.node.create_publisher(
            JointState,
            '/test_health/joint_states',
            10,
        )
        cls.joint_publish_timer = cls.node.create_timer(
            0.05,
            cls._publish_joint_state,
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _diagnostics_callback(cls, message):
        for status in message.status:
            owner, separator, task = status.name.rpartition(': ')
            if separator and owner.endswith('test_system_health_monitor'):
                cls.statuses[task] = status

    @classmethod
    def _health_callback(cls, message):
        cls.health_messages.append(message)

    @classmethod
    def _publish_joint_state(cls):
        message = JointState()
        message.header.stamp = cls.node.get_clock().now().to_msg()
        message.name = ['health_test_joint']
        message.position = [0.0]
        cls.joint_state_publisher.publish(message)

    def _spin_until(self, predicate, timeout, failure_message):
        deadline = time.monotonic() + timeout
        while not predicate():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                self.fail(failure_message)
            rclpy.spin_once(self.node, timeout_sec=min(0.05, remaining))

    def test_expected_health_contract_is_published(self):
        expected_levels = {
            'Joint states': DiagnosticStatus.STALE,
            'Simulation clock': DiagnosticStatus.OK,
            'Controllers': DiagnosticStatus.STALE,
            'Robot TF': DiagnosticStatus.ERROR,
            'Motion backend action': DiagnosticStatus.ERROR,
            'Gripper action': DiagnosticStatus.ERROR,
            'PlanningScene coordinator': DiagnosticStatus.STALE,
        }
        self._spin_until(
            lambda: all(
                name in self.statuses and self.statuses[name].level == level
                for name, level in expected_levels.items()
            ),
            10.0,
            f'did not receive expected diagnostics; latest={self.statuses}',
        )

        clock_values = {
            value.key: value.value
            for value in self.statuses['Simulation clock'].values
        }
        self.assertEqual(clock_values['required'].lower(), 'false')

        request_keys = {
            'request_in_flight',
            'request_generation',
            'request_id',
            'request_steady_age_sec',
            'request_timeout_sec',
            'request_timeout_count',
        }
        for diagnostic_name in ('Controllers', 'PlanningScene coordinator'):
            request_values = {
                value.key: value.value
                for value in self.statuses[diagnostic_name].values
            }
            self.assertTrue(request_keys.issubset(request_values))
            self.assertEqual(request_values['request_in_flight'].lower(), 'false')
            self.assertEqual(request_values['request_timeout_count'], '0')

    def test_typed_topic_and_service_report_the_same_fault_classes(self):
        def expected_snapshot_received():
            if not self.health_messages:
                return False
            components = {
                component.component_id: component
                for component in self.health_messages[-1].components
            }
            joints = components.get(HealthComponentStatus.COMPONENT_JOINT_STATES)
            return (
                joints is not None
                and joints.observed
                and joints.level == HealthComponentStatus.LEVEL_STALE
                and HealthComponentStatus.ISSUE_JOINT_STATES_STALE
                in joints.issue_codes
            )

        self._spin_until(
            expected_snapshot_received,
            10.0,
            'did not receive a typed snapshot after observing JointState',
        )
        topic_snapshot = self.health_messages[-1]
        components = {
            component.component_id: component
            for component in topic_snapshot.components
        }

        self.assertFalse(topic_snapshot.motion_ready)
        self.assertEqual(
            topic_snapshot.overall_level,
            HealthComponentStatus.LEVEL_STALE,
        )
        expected = {
            HealthComponentStatus.COMPONENT_CONTROLLERS: (
                HealthComponentStatus.LEVEL_STALE,
                HealthComponentStatus.ISSUE_CONTROLLER_STATE_UNAVAILABLE,
            ),
            HealthComponentStatus.COMPONENT_ROBOT_TF: (
                HealthComponentStatus.LEVEL_ERROR,
                HealthComponentStatus.ISSUE_ROBOT_TF_UNAVAILABLE,
            ),
            HealthComponentStatus.COMPONENT_MOVE_GROUP: (
                HealthComponentStatus.LEVEL_ERROR,
                HealthComponentStatus.ISSUE_MOVE_GROUP_UNAVAILABLE,
            ),
            HealthComponentStatus.COMPONENT_GRIPPER: (
                HealthComponentStatus.LEVEL_ERROR,
                HealthComponentStatus.ISSUE_GRIPPER_UNAVAILABLE,
            ),
            HealthComponentStatus.COMPONENT_PLANNING_SCENE: (
                HealthComponentStatus.LEVEL_STALE,
                HealthComponentStatus.ISSUE_PLANNING_SCENE_COORDINATOR_UNAVAILABLE,
            ),
        }
        for component_id, (level, issue) in expected.items():
            self.assertEqual(components[component_id].level, level)
            self.assertIn(issue, components[component_id].issue_codes)

        self.assertTrue(self.health_client.wait_for_service(timeout_sec=5.0))
        future = self.health_client.call_async(GetSystemHealth.Request())
        self._spin_until(
            future.done,
            5.0,
            'typed health snapshot service did not respond',
        )
        service_snapshot = future.result().snapshot
        self.assertFalse(service_snapshot.motion_ready)
        self.assertEqual(
            service_snapshot.overall_level,
            topic_snapshot.overall_level,
        )
        self.assertGreaterEqual(service_snapshot.sequence, topic_snapshot.sequence)

    def test_snapshot_sequence_is_monotonic(self):
        self._spin_until(
            lambda: len(self.health_messages) >= 2,
            10.0,
            'did not receive two typed health snapshots',
        )
        sequences = [message.sequence for message in self.health_messages]
        self.assertEqual(sequences, sorted(sequences))
        self.assertEqual(len(sequences), len(set(sequences)))
