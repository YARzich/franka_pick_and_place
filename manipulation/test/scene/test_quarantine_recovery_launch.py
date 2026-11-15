#!/usr/bin/env python3
"""
Exercise lease-expiry quarantine and recovery over ROS services.

Responsibility: Require explicit recovery before later PlanningScene mutations are admitted.
"""

import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
from manipulation_msgs.msg import PlanningSceneOperationResult
from manipulation_msgs.srv import AcquirePlanningSceneLease
from manipulation_msgs.srv import GetPlanningSceneState
from manipulation_msgs.srv import ReconcilePlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.srv import GetPlanningScene
import pytest
import rclpy


_PREFIX = '/quarantine_recovery_coordinator'


@pytest.mark.launch_test
def generate_test_description():
    """Launch a coordinator against a deterministic empty-scene backend."""
    coordinator = launch_ros.actions.Node(
        package='manipulation',
        executable='planning_scene_coordinator',
        name='quarantine_recovery_coordinator',
        output='screen',
        parameters=[{
            'session_id': 'quarantine-test-session',
            'backend_planning_frame': 'robot_base',
            'verify_fixed_root_model_frame': False,
            'lease_expiry_check_period_sec': 0.05,
            'reconciliation_check_period_sec': 0.05,
            'backend_service_timeout_sec': 0.2,
            'readback_service_timeout_sec': 0.2,
        }],
    )
    return launch.LaunchDescription([
        coordinator,
        launch_testing.actions.ReadyToTest(),
    ]), {'coordinator': coordinator}


class TestQuarantineRecovery(unittest.TestCase):
    """Exercise expiry, readback and typed recovery over ROS services."""

    @classmethod
    def setUpClass(cls):
        """Create the fake backend and coordinator service clients."""
        rclpy.init()
        cls.node = rclpy.create_node('quarantine_recovery_test_ports')
        cls.readback_count = 0
        cls.readback_service = cls.node.create_service(
            GetPlanningScene,
            '/get_planning_scene',
            cls._readback_scene,
        )
        cls.apply_service = cls.node.create_service(
            ApplyPlanningScene,
            '/apply_planning_scene',
            cls._apply_scene,
        )
        cls.acquire = cls.node.create_client(
            AcquirePlanningSceneLease, f'{_PREFIX}/acquire_lease')
        cls.state = cls.node.create_client(
            GetPlanningSceneState, f'{_PREFIX}/get_state')
        cls.reconcile = cls.node.create_client(
            ReconcilePlanningScene, f'{_PREFIX}/reconcile')

        deadline = time.monotonic() + 10.0
        for client in (cls.acquire, cls.state, cls.reconcile):
            while not client.wait_for_service(timeout_sec=0.05):
                if time.monotonic() >= deadline:
                    raise RuntimeError(
                        'PlanningScene coordinator service unavailable')

    @classmethod
    def tearDownClass(cls):
        """Destroy graph entities before launch shutdown checks."""
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _readback_scene(cls, request, response):
        """Return a stable authoritative scene and count every readback."""
        del request
        cls.readback_count += 1
        return response

    @staticmethod
    def _apply_scene(request, response):
        """Keep the apply dependency ready; this test performs no mutation."""
        del request
        response.success = True
        return response

    def _call(self, client, request, timeout=3.0):
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                self.fail('service response timed out')
            rclpy.spin_once(self.node, timeout_sec=min(0.02, remaining))
        return future.result()

    def _get_state(self):
        response = self._call(
            self.state, GetPlanningSceneState.Request())
        self.assertTrue(
            response.result.success,
            f'{response.result.error_code}: {response.result.message}',
        )
        return response.state

    def _wait_for_state(self, predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        last = None
        while time.monotonic() < deadline:
            last = self._get_state()
            if predicate(last):
                return last
            time.sleep(0.02)
        self.fail(f'coordinator state predicate timed out; last={last}')

    def test_expiry_stays_quarantined_until_typed_acknowledgement(self):
        """Reject takeover after TTL/readback, then clear via recovery ack."""
        initial = self._wait_for_state(
            lambda state: (
                state.backend_service_ready
                and not state.reconciliation_required
            )
        )

        request = AcquirePlanningSceneLease.Request()
        request.owner_id = 'unsafe-transfer'
        request.resource_keys = ['world:cube', 'attached:cube']
        request.requested_ttl.nanosec = 200_000_000
        acquired = self._call(self.acquire, request)
        self.assertTrue(acquired.result.success)

        quarantined = self._wait_for_state(
            lambda state: state.quarantined_resource_count == 2)
        self.assertTrue(quarantined.reconciliation_required)
        self.assertEqual(
            list(quarantined.quarantined_resource_keys),
            ['attached:cube', 'world:cube'],
        )

        takeover = AcquirePlanningSceneLease.Request()
        takeover.owner_id = 'second-owner'
        takeover.resource_keys = ['world:cube']
        takeover.requested_ttl.sec = 10
        rejected = self._call(self.acquire, takeover)
        self.assertFalse(rejected.result.success)
        self.assertEqual(
            rejected.result.error_code,
            PlanningSceneOperationResult.RESOURCE_QUARANTINED,
        )

        readbacks_before = self.readback_count
        still_quarantined = self._wait_for_state(
            lambda state: (
                self.readback_count > readbacks_before
                and state.quarantined_resource_count == 2
                and state.reconciliation_required
            )
        )
        self.assertEqual(still_quarantined.session_id, initial.session_id)

        recovery = ReconcilePlanningScene.Request()
        recovery.expected_session_id = still_quarantined.session_id
        recovery.expected_revision = still_quarantined.revision
        recovery.accept_authoritative_state = True
        recovery.quarantine_resolution = (
            ReconcilePlanningScene.Request.PRESERVE_QUARANTINE)
        missing_ack = self._call(self.reconcile, recovery)
        self.assertFalse(missing_ack.result.success)
        self.assertEqual(
            missing_ack.result.error_code,
            PlanningSceneOperationResult.RESOURCE_QUARANTINED,
        )

        recovery.quarantine_resolution = (
            ReconcilePlanningScene.Request.ACCEPT_QUARANTINED_RESOURCES)
        recovery.expected_quarantined_resource_keys = list(
            still_quarantined.quarantined_resource_keys)
        accepted = self._call(self.reconcile, recovery)
        self.assertTrue(
            accepted.result.success,
            f'{accepted.result.error_code}: {accepted.result.message}',
        )
        self.assertNotEqual(accepted.new_session_id, initial.session_id)

        recovered = self._wait_for_state(
            lambda state: (
                state.session_id == accepted.new_session_id
                and state.quarantined_resource_count == 0
                and not state.reconciliation_required
            )
        )
        self.assertEqual(recovered.revision, still_quarantined.revision)
        self.assertTrue(self._call(self.acquire, takeover).result.success)


@launch_testing.post_shutdown_test()
class TestCoordinatorShutdown(unittest.TestCase):
    """Require clean coordinator shutdown after the quarantine test."""

    def test_exit_code(self, proc_info, coordinator):
        launch_testing.asserts.assertExitCodes(
            proc_info, process=coordinator)
