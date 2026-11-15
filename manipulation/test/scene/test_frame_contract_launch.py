"""
Exercise PlanningScene model-frame and admission contracts at runtime.

Responsibility: Verify TF normalization, fixed frames, lease ownership, and mutation rejection.
"""

import copy
from pathlib import Path
import sys
import threading
import time
import unittest

from geometry_msgs.msg import TransformStamped
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
from manipulation_msgs.msg import PlanningSceneOperationResult
from manipulation_msgs.srv import AcquirePlanningSceneLease
from manipulation_msgs.srv import ApplyPlanningSceneDiff
from manipulation_msgs.srv import GetPlanningSceneState
from moveit_msgs.msg import PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.srv import GetPlanningScene
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

sys.path.insert(0, str(Path(__file__).resolve().parent))
import frame_contract_test_support as support  # noqa: E402, I100

APPLY_BACKEND = support.APPLY_BACKEND
READBACK_BACKEND = support.READBACK_BACKEND
COORDINATOR = support.COORDINATOR
apply_request = support.apply_request
robot_description = support.robot_description


@pytest.mark.launch_test
def generate_test_description():
    """Run with simulated time deliberately paused at zero."""
    coordinator = launch_ros.actions.Node(
        package='manipulation',
        executable='planning_scene_coordinator',
        name='frame_contract_coordinator',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'session_id': 'frame-contract-session',
            'backend_service': APPLY_BACKEND,
            'readback_service': READBACK_BACKEND,
            'planning_frame': 'robot_base',
            'backend_planning_frame': 'world',
            'backend_service_timeout_sec': 1.0,
            'readback_service_timeout_sec': 1.0,
            'lease_expiry_check_period_sec': 0.05,
            'reconciliation_check_period_sec': 0.05,
        }],
    )
    return launch.LaunchDescription([
        coordinator,
        launch_testing.actions.ReadyToTest(),
    ]), {'coordinator': coordinator}


class TestFrameContract(unittest.TestCase):
    """Own a controllable MoveIt substitute and exercise fail-closed recovery."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.lock = threading.Lock()
        cls.authoritative_scene = PlanningScene()
        cls.forced_readback_frame = None
        cls.apply_delay = 0.0
        cls.apply_count = 0
        cls.last_applied_scene = None

        cls.backend_node = rclpy.create_node('frame_contract_fake_moveit')
        cls.client_node = rclpy.create_node('frame_contract_client')
        cls.apply_service = cls.backend_node.create_service(
            ApplyPlanningScene, APPLY_BACKEND, cls._apply_scene)
        cls.readback_service = cls.backend_node.create_service(
            GetPlanningScene, READBACK_BACKEND, cls._readback_scene)
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        cls.description_publisher = cls.backend_node.create_publisher(
            String, '/robot_description', qos)
        cls.static_tf = StaticTransformBroadcaster(cls.backend_node)

        cls.executor = MultiThreadedExecutor(num_threads=2)
        cls.executor.add_node(cls.backend_node)
        cls.executor_thread = threading.Thread(
            target=cls.executor.spin, daemon=True)
        cls.executor_thread.start()

        cls.acquire_client = cls.client_node.create_client(
            AcquirePlanningSceneLease, f'{COORDINATOR}/acquire_lease')
        cls.apply_client = cls.client_node.create_client(
            ApplyPlanningSceneDiff, f'{COORDINATOR}/apply_scene_diff')
        cls.state_client = cls.client_node.create_client(
            GetPlanningSceneState, f'{COORDINATOR}/get_state')
        for client in (
            cls.acquire_client,
            cls.apply_client,
            cls.state_client,
        ):
            if not client.wait_for_service(timeout_sec=10.0):
                raise RuntimeError('PlanningScene coordinator service unavailable')

    @classmethod
    def tearDownClass(cls):
        cls.client_node.destroy_node()
        cls.executor.shutdown(timeout_sec=2.0)
        cls.executor_thread.join(timeout=2.0)
        cls.backend_node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _apply_scene(cls, request, response):
        time.sleep(cls.apply_delay)
        with cls.lock:
            cls.apply_count += 1
            cls.authoritative_scene = copy.deepcopy(request.scene)
            cls.last_applied_scene = copy.deepcopy(request.scene)
        response.success = True
        return response

    @classmethod
    def _readback_scene(cls, request, response):
        del request
        with cls.lock:
            scene = copy.deepcopy(cls.authoritative_scene)
            forced_frame = cls.forced_readback_frame
        if forced_frame is not None:
            for object_message in scene.world.collision_objects:
                object_message.header.frame_id = forced_frame
        response.scene = scene
        return response

    def _call(self, client, request, timeout=3.0):
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                self.fail('service response timed out')
            rclpy.spin_once(
                self.client_node, timeout_sec=min(0.02, remaining))
        self.assertIsNotNone(future.result())
        return future.result()

    def _state(self):
        return self._call(
            self.state_client, GetPlanningSceneState.Request()).state

    def _wait_state(self, predicate, timeout=3.0):
        deadline = time.monotonic() + timeout
        last = None
        while time.monotonic() < deadline:
            last = self._state()
            if predicate(last):
                return last
            time.sleep(0.03)
        self.fail(f'coordinator state predicate timed out: {last}')

    def test_model_frame_tf_readback_and_concurrent_admission(self):
        acquire = AcquirePlanningSceneLease.Request()
        acquire.owner_id = 'frame-contract-test'
        acquire.resource_keys = ['world:cube']
        acquire.requested_ttl.sec = 30
        lease = self._call(self.acquire_client, acquire)
        self.assertTrue(lease.result.success)

        self.description_publisher.publish(robot_description('map'))
        self._wait_state(lambda state: not state.backend_service_ready)
        wrong_root = self._call(
            self.apply_client,
            apply_request(lease.lease_id, 'wrong-root', 0),
        )
        self.assertEqual(
            wrong_root.result.error_code,
            PlanningSceneOperationResult.RECONCILIATION_REQUIRED,
        )
        self.assertEqual(self.apply_count, 0)

        self.description_publisher.publish(robot_description('world'))
        self._wait_state(
            lambda state: state.backend_service_ready
            and not state.reconciliation_required)

        started = time.monotonic()
        missing_tf_request = apply_request(
            lease.lease_id, 'wait-for-tf', 0)
        missing_tf = self._call(self.apply_client, missing_tf_request)
        self.assertLess(time.monotonic() - started, 1.0)
        self.assertEqual(
            missing_tf.result.error_code,
            PlanningSceneOperationResult.DEPENDENCY_UNAVAILABLE,
        )
        state = self._state()
        self.assertEqual(state.revision, 0)
        self.assertFalse(state.operation_in_progress)
        self.assertFalse(state.reconciliation_required)
        self.assertEqual(self.apply_count, 0)

        transform = TransformStamped()
        transform.header.frame_id = 'world'
        transform.child_frame_id = 'robot_base'
        transform.transform.translation.z = 1.0
        transform.transform.rotation.w = 1.0
        self.static_tf.sendTransform(transform)

        deadline = time.monotonic() + 3.0
        applied = None
        while time.monotonic() < deadline:
            applied = self._call(self.apply_client, missing_tf_request)
            if applied.result.success:
                break
            self.assertEqual(
                applied.result.error_code,
                PlanningSceneOperationResult.DEPENDENCY_UNAVAILABLE,
            )
            time.sleep(0.03)
        self.assertIsNotNone(applied)
        self.assertTrue(applied.result.success, applied.result.message)
        self.assertEqual(applied.result.revision, 1)
        self.assertEqual(self.apply_count, 1)
        with self.lock:
            backend_object = self.last_applied_scene.world.collision_objects[0]
        self.assertEqual(backend_object.header.frame_id, 'world')
        self.assertAlmostEqual(backend_object.pose.position.z, 1.0)

        with self.lock:
            type(self).forced_readback_frame = 'map'
        self._wait_state(lambda state: state.reconciliation_required)
        with self.lock:
            type(self).forced_readback_frame = None
        self._wait_state(lambda state: not state.reconciliation_required)

        type(self).apply_delay = 0.3
        futures = [
            self.apply_client.call_async(
                apply_request(
                    lease.lease_id, f'concurrent-{index}', 1, index * 0.01))
            for index in range(6)
        ]
        state_future = self.state_client.call_async(
            GetPlanningSceneState.Request())
        state_deadline = time.monotonic() + 1.0
        while not state_future.done() and time.monotonic() < state_deadline:
            rclpy.spin_once(self.client_node, timeout_sec=0.02)
        self.assertTrue(state_future.done(), 'metadata service was starved by apply')

        deadline = time.monotonic() + 6.0
        while not all(future.done() for future in futures):
            if time.monotonic() >= deadline:
                self.fail('concurrent apply callbacks deadlocked')
            rclpy.spin_once(self.client_node, timeout_sec=0.02)
        results = [future.result().result for future in futures]
        self.assertEqual(sum(result.success for result in results), 1)
        self.assertEqual(self.apply_count, 2)


@launch_testing.post_shutdown_test()
class TestFrameContractShutdown(unittest.TestCase):
    """Require a clean coordinator shutdown after the stress contract."""

    def test_process_exits_cleanly(self, proc_info, coordinator):
        launch_testing.asserts.assertExitCodes(proc_info, process=coordinator)
