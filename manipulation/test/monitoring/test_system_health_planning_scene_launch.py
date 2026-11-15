"""
Exercise asynchronous PlanningScene health polling over ROS interfaces.

Responsibility: Verify typed health state, stuck detection, and non-blocking service polling.
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
from manipulation_msgs.srv import GetPlanningSceneState
import pytest
import rclpy


SCENE_SERVICE = '/typed_health_scene/get_state'


@pytest.mark.launch_test
def generate_test_description():
    """Start a monitor with a short, deterministic scene-operation deadline."""
    monitor_node = launch_ros.actions.Node(
        package='manipulation',
        executable='system_health_monitor',
        name='planning_scene_health_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'startup_grace_sec': 0.0,
            'require_sim_clock': False,
            'joint_states_topic': '/typed_health_scene/no_joint_states',
            'controller_manager_service': '/typed_health_scene/no_controllers',
            'motion_backend_action': '/typed_health_scene/no_motion_backend',
            'motion_backend_action_type': 'manipulation_msgs/action/TransferObjectSequenceV2',
            'gripper_action': '/typed_health_scene/no_gripper',
            'planning_scene_state_service': SCENE_SERVICE,
            'base_frame': 'typed_health_scene_base',
            'end_effector_frame': 'typed_health_scene_tool',
            'required_joint_names': ['test_joint'],
            'required_controllers': ['test_arm_controller'],
            'planning_scene_warn_after_sec': 2.0,
            'planning_scene_stale_after_sec': 5.0,
            'planning_scene_operation_stuck_after_sec': 0.5,
            'health_publish_period_sec': 0.1,
            'diagnostic_updater.period': 0.1,
        }],
    )

    description = launch.LaunchDescription()
    description.add_action(monitor_node)
    description.add_action(launch_testing.actions.ReadyToTest())
    return description, {'monitor_node': monitor_node}


class TestPlanningSceneHealth(unittest.TestCase):
    """Serve coordinator state and inspect typed and standard classifications."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('planning_scene_health_contract_test')
        cls.health_messages = []
        cls.scene_diagnostic = None
        cls.request_count = 0
        cls.inject_consistency_fault = False
        cls.scene_service = cls.node.create_service(
            GetPlanningSceneState,
            SCENE_SERVICE,
            cls._serve_scene_state,
        )
        cls.health_subscription = cls.node.create_subscription(
            SystemHealthSnapshot,
            '/planning_scene_health_monitor/health',
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
    def _serve_scene_state(cls, request, response):
        del request
        cls.request_count += 1
        response.state.session_id = 'typed-health-test-session'
        response.state.revision = 7
        response.state.operation_in_progress = not cls.inject_consistency_fault
        response.state.backend_service_ready = not cls.inject_consistency_fault
        response.state.reconciliation_required = cls.inject_consistency_fault
        if cls.inject_consistency_fault:
            response.state.last_backend_error_code = response.result.TIMEOUT
            response.state.unreconciled_lease_expiry_count = 1
            response.state.operation_capacity_exhausted = True
        response.result.success = True
        response.result.error_code = response.result.NONE
        response.result.revision = 7
        return response

    @classmethod
    def _health_callback(cls, message):
        cls.health_messages.append(message)

    @classmethod
    def _diagnostics_callback(cls, message):
        for status in message.status:
            owner, separator, task = status.name.rpartition(': ')
            if (
                separator
                and owner.endswith('planning_scene_health_monitor')
                and task == 'PlanningScene coordinator'
            ):
                cls.scene_diagnostic = status

    def _scene_component(self, issue):
        for message in reversed(self.health_messages):
            for component in message.components:
                if (
                    component.component_id
                    == HealthComponentStatus.COMPONENT_PLANNING_SCENE
                    and issue in component.issue_codes
                ):
                    return component
        return None

    def _spin_until(self, predicate, timeout, failure_message):
        deadline = time.monotonic() + timeout
        while not predicate():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                self.fail(failure_message)
            rclpy.spin_once(self.node, timeout_sec=min(0.05, remaining))

    def test_operation_transitions_from_warn_to_stuck_error(self):
        in_progress_issue = (
            HealthComponentStatus.ISSUE_PLANNING_SCENE_OPERATION_IN_PROGRESS
        )
        stuck_issue = HealthComponentStatus.ISSUE_PLANNING_SCENE_OPERATION_STUCK

        self._spin_until(
            lambda: self._scene_component(in_progress_issue) is not None,
            10.0,
            'did not observe fresh PlanningScene operation state',
        )
        in_progress = self._scene_component(in_progress_issue)
        self.assertEqual(in_progress.level, HealthComponentStatus.LEVEL_WARN)
        self.assertGreater(self.request_count, 0)

        self._spin_until(
            lambda: self._scene_component(stuck_issue) is not None,
            5.0,
            'PlanningScene operation did not become stuck',
        )
        stuck = self._scene_component(stuck_issue)
        self.assertEqual(stuck.level, HealthComponentStatus.LEVEL_ERROR)

        self._spin_until(
            lambda: (
                self.scene_diagnostic is not None
                and self.scene_diagnostic.level == DiagnosticStatus.ERROR
            ),
            5.0,
            'standard diagnostics did not report the same scene error',
        )

        type(self).inject_consistency_fault = True
        reconciliation_issue = (
            HealthComponentStatus.ISSUE_PLANNING_SCENE_RECONCILIATION_REQUIRED
        )
        self._spin_until(
            lambda: self._scene_component(reconciliation_issue) is not None,
            5.0,
            'typed health did not expose coordinator consistency failure',
        )
        consistency = self._scene_component(reconciliation_issue)
        self.assertEqual(consistency.level, HealthComponentStatus.LEVEL_ERROR)
        self.assertIn(
            HealthComponentStatus.ISSUE_PLANNING_SCENE_BACKEND_UNAVAILABLE,
            consistency.issue_codes,
        )
        self.assertIn(
            HealthComponentStatus.ISSUE_PLANNING_SCENE_BACKEND_ERROR,
            consistency.issue_codes,
        )
        self.assertIn(
            HealthComponentStatus.ISSUE_PLANNING_SCENE_LEASE_EXPIRED,
            consistency.issue_codes,
        )
        self.assertIn(
            HealthComponentStatus.ISSUE_PLANNING_SCENE_OPERATION_CAPACITY_EXHAUSTED,
            consistency.issue_codes,
        )
