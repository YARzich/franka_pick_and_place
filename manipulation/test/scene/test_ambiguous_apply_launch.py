"""
Exercise ambiguous PlanningScene apply outcomes over ROS services.

Responsibility: Keep timed-out mutations quarantined until their late backend response resolves.
"""

import threading
import time
import unittest

from geometry_msgs.msg import Pose
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
from manipulation_msgs.msg import PlanningSceneOperationResult
from manipulation_msgs.srv import AcquirePlanningSceneLease
from manipulation_msgs.srv import ApplyPlanningSceneDiff
from manipulation_msgs.srv import GetPlanningSceneState
from moveit_msgs.msg import CollisionObject
from moveit_msgs.msg import PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.srv import GetPlanningScene
import pytest
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from shape_msgs.msg import SolidPrimitive


APPLY_BACKEND = '/ambiguous_test/apply_planning_scene'
READBACK_BACKEND = '/ambiguous_test/get_planning_scene'


@pytest.mark.launch_test
def generate_test_description():
    """Start a coordinator with a deliberately short backend deadline."""
    coordinator = launch_ros.actions.Node(
        package='manipulation',
        executable='planning_scene_coordinator',
        name='ambiguous_apply_coordinator',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'session_id': 'ambiguous-apply-session',
            'backend_service': APPLY_BACKEND,
            'readback_service': READBACK_BACKEND,
            'backend_service_timeout_sec': 0.1,
            'readback_service_timeout_sec': 1.0,
            # The fake backend preserves frames verbatim and has no TF graph.
            'planning_frame': 'robot_base',
            'backend_planning_frame': 'robot_base',
            'verify_fixed_root_model_frame': False,
            'lease_expiry_check_period_sec': 0.05,
            'reconciliation_check_period_sec': 0.05,
        }],
    )
    description = launch.LaunchDescription()
    description.add_action(coordinator)
    description.add_action(launch_testing.actions.ReadyToTest())
    return description, {'coordinator': coordinator}


class TestAmbiguousApply(unittest.TestCase):
    """Own deterministic fake MoveIt services and exercise late completion."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.scene_lock = threading.Lock()
        cls.backend_node = rclpy.create_node('ambiguous_apply_fake_moveit')
        cls.backend_group = ReentrantCallbackGroup()
        cls.node = rclpy.create_node('ambiguous_apply_contract_test')
        cls.authoritative_scene = PlanningScene()
        cls.backend_apply_count = 0
        cls.apply_backend = cls.backend_node.create_service(
            ApplyPlanningScene,
            APPLY_BACKEND,
            cls._apply_backend,
            callback_group=cls.backend_group,
        )
        cls.readback_backend = cls.backend_node.create_service(
            GetPlanningScene,
            READBACK_BACKEND,
            cls._readback_backend,
            callback_group=cls.backend_group,
        )
        cls.backend_executor = MultiThreadedExecutor(num_threads=2)
        cls.backend_executor.add_node(cls.backend_node)
        cls.backend_thread = threading.Thread(
            target=cls.backend_executor.spin,
            daemon=True,
        )
        cls.backend_thread.start()
        prefix = '/ambiguous_apply_coordinator'
        cls.acquire_client = cls.node.create_client(
            AcquirePlanningSceneLease,
            f'{prefix}/acquire_lease',
        )
        cls.apply_client = cls.node.create_client(
            ApplyPlanningSceneDiff,
            f'{prefix}/apply_scene_diff',
        )
        cls.state_client = cls.node.create_client(
            GetPlanningSceneState,
            f'{prefix}/get_state',
        )
        for client in (
            cls.acquire_client,
            cls.apply_client,
            cls.state_client,
        ):
            if not client.wait_for_service(timeout_sec=10.0):
                raise RuntimeError('PlanningScene coordinator service unavailable')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        cls.backend_executor.shutdown(timeout_sec=2.0)
        cls.backend_thread.join(timeout=2.0)
        cls.backend_node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _apply_backend(cls, request, response):
        """Return after the coordinator deadline, then expose the applied state."""
        with cls.scene_lock:
            cls.backend_apply_count += 1
        time.sleep(0.4)
        with cls.scene_lock:
            cls.authoritative_scene = request.scene
        response.success = True
        return response

    @classmethod
    def _readback_backend(cls, request, response):
        """Return the current coordinator-owned fake backend state."""
        del request
        with cls.scene_lock:
            response.scene = cls.authoritative_scene
        return response

    def _call(self, client, request, timeout=5.0):
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                self.fail('service response timed out')
            rclpy.spin_once(self.node, timeout_sec=min(0.02, remaining))
        return future.result()

    @staticmethod
    def _scene_diff():
        scene = PlanningScene()
        scene.is_diff = True
        object_message = CollisionObject()
        object_message.id = 'cube'
        object_message.header.frame_id = 'robot_base'
        object_message.operation = CollisionObject.ADD
        object_message.pose.orientation.w = 1.0
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.05, 0.05, 0.05]
        object_message.primitives.append(box)
        shape_pose = Pose()
        shape_pose.orientation.w = 1.0
        object_message.primitive_poses.append(shape_pose)
        scene.world.collision_objects.append(object_message)
        return scene

    def test_late_success_replaces_timeout_without_second_backend_apply(self):
        acquire = AcquirePlanningSceneLease.Request()
        acquire.owner_id = 'ambiguous-test'
        acquire.resource_keys = ['world:cube']
        acquire.requested_ttl.sec = 10
        acquired = self._call(self.acquire_client, acquire)
        self.assertTrue(acquired.result.success)

        apply = ApplyPlanningSceneDiff.Request()
        apply.lease_id = acquired.lease_id
        apply.operation_id = 'ambiguous-operation'
        apply.expected_revision = acquired.result.revision
        apply.resource_keys = ['world:cube']
        apply.scene_diff = self._scene_diff()
        timed_out = self._call(self.apply_client, apply)
        self.assertFalse(timed_out.result.success)
        self.assertEqual(
            timed_out.result.error_code,
            PlanningSceneOperationResult.TIMEOUT,
        )

        deadline = time.monotonic() + 5.0
        last_state = None
        while True:
            state = self._call(
                self.state_client,
                GetPlanningSceneState.Request(),
                timeout=1.0,
            )
            last_state = state.state
            if (
                state.state.revision == 1
                and not state.state.operation_in_progress
                and not state.state.reconciliation_required
            ):
                break
            if time.monotonic() >= deadline:
                with self.scene_lock:
                    apply_count = self.backend_apply_count
                self.fail(
                    'late backend response was not reconciled: '
                    f'revision={last_state.revision}, '
                    f'in_progress={last_state.operation_in_progress}, '
                    f'reconcile={last_state.reconciliation_required}, '
                    f'backend_ready={last_state.backend_service_ready}, '
                    f'backend_error={last_state.last_backend_error_code}, '
                    f'fake_apply_count={apply_count}'
                )

        replay = self._call(self.apply_client, apply)
        self.assertTrue(
            replay.result.success,
            f'code={replay.result.error_code}: {replay.result.message}',
        )
        self.assertEqual(replay.result.revision, 1)
        with self.scene_lock:
            self.assertEqual(self.backend_apply_count, 1)


@launch_testing.post_shutdown_test()
class TestCoordinatorShutdown(unittest.TestCase):
    """Ensure the coordinator exits cleanly after the contract test."""

    def test_process_exits_cleanly(self, proc_info, coordinator):
        launch_testing.asserts.assertExitCodes(proc_info, process=coordinator)
