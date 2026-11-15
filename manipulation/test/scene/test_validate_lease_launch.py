"""
Exercise pre-side-effect PlanningScene lease validation over ROS services.

Responsibility: Verify ownership, scope, expiry, and shutdown responses before mutation starts.
"""

import time
import unittest

from builtin_interfaces.msg import Duration
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
from manipulation_msgs.msg import PlanningSceneOperationResult
from manipulation_msgs.srv import AcquirePlanningSceneLease
from manipulation_msgs.srv import ValidatePlanningSceneLease
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.srv import GetPlanningScene
import pytest
import rclpy


ACQUIRE_SERVICE = '/lease_validation_coordinator/acquire_lease'
VALIDATE_SERVICE = '/lease_validation_coordinator/validate_lease'


@pytest.mark.launch_test
def generate_test_description():
    """Start the coordinator without a MoveIt backend for read-only services."""
    coordinator = launch_ros.actions.Node(
        package='manipulation',
        executable='planning_scene_coordinator',
        name='lease_validation_coordinator',
        output='screen',
        parameters=[{
            'session_id': 'lease-validation-session',
            # This isolated fake backend preserves messages verbatim and does
            # not own the test world <- robot_base TF.
            'backend_planning_frame': 'robot_base',
            'verify_fixed_root_model_frame': False,
            'lease_expiry_check_period_sec': 0.05,
            # This checks lease semantics, not backend latency. Allow DDS
            # discovery and the Python readback callback on a loaded runner.
            'backend_service_timeout_sec': 1.0,
        }],
    )
    return launch.LaunchDescription([
        coordinator,
        launch_testing.actions.ReadyToTest(),
    ]), {'coordinator': coordinator}


class TestValidateLeaseService(unittest.TestCase):
    """Verify typed success, revision and ownership responses over ROS."""

    @classmethod
    def setUpClass(cls):
        """Create clients and wait for both coordinator services."""
        rclpy.init()
        cls.node = rclpy.create_node('lease_validation_client')
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
            AcquirePlanningSceneLease,
            ACQUIRE_SERVICE,
        )
        cls.validate = cls.node.create_client(
            ValidatePlanningSceneLease,
            VALIDATE_SERVICE,
        )
        deadline = time.monotonic() + 10.0
        for client in (cls.acquire, cls.validate):
            while not client.wait_for_service(timeout_sec=0.05):
                if time.monotonic() >= deadline:
                    raise RuntimeError('PlanningScene coordinator service unavailable')

    @classmethod
    def tearDownClass(cls):
        """Destroy graph entities before launch shutdown assertions."""
        cls.node.destroy_node()
        rclpy.shutdown()

    @staticmethod
    def _readback_scene(request, response):
        """Return an empty authoritative scene for lease-only validation."""
        del request
        return response

    @staticmethod
    def _apply_scene(request, response):
        """Provide a ready backend; this read-only contract never calls it."""
        del request
        response.success = True
        return response

    def _call(self, client, request, timeout=2.0):
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                self.fail('coordinator service response timed out')
            rclpy.spin_once(self.node, timeout_sec=min(0.02, remaining))
        response = future.result()
        self.assertIsNotNone(response)
        return response

    def test_validation_reports_ttl_revision_and_resource_ownership(self):
        """An active lease proves only its owned canonical resource subset."""
        acquire_request = AcquirePlanningSceneLease.Request()
        acquire_request.owner_id = 'workflow-test'
        acquire_request.resource_keys = ['world:cube', 'attached:cube']
        acquire_request.requested_ttl = Duration(sec=5)
        acquired = self._call(self.acquire, acquire_request)
        self.assertTrue(acquired.result.success, str(acquired.result))

        valid_request = ValidatePlanningSceneLease.Request()
        valid_request.lease_id = acquired.lease_id
        valid_request.required_resource_keys = ['attached:cube', 'world:cube']
        valid_request.expected_revision = acquired.result.revision
        valid = self._call(self.validate, valid_request)
        self.assertTrue(valid.result.success, str(valid.result))
        self.assertEqual(valid.session_id, 'lease-validation-session')
        self.assertGreater(valid.remaining_ttl.sec, 0)

        valid_request.required_resource_keys = ['world:other']
        not_owner = self._call(self.validate, valid_request)
        self.assertFalse(not_owner.result.success)
        self.assertEqual(
            not_owner.result.error_code,
            PlanningSceneOperationResult.NOT_OWNER,
        )

        valid_request.required_resource_keys = ['world:cube']
        valid_request.expected_revision += 1
        stale = self._call(self.validate, valid_request)
        self.assertEqual(
            stale.result.error_code,
            PlanningSceneOperationResult.REVISION_MISMATCH,
        )


@launch_testing.post_shutdown_test()
class TestValidateLeaseShutdown(unittest.TestCase):
    """Ensure the coordinator exits normally after the contract test."""

    def test_process_exits_cleanly(self, proc_info, coordinator):
        """Check the exact process rather than accepting unrelated exits."""
        launch_testing.asserts.assertExitCodes(proc_info, process=coordinator)
