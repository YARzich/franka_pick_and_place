"""
Provide reusable launch-test probes for Franka runtime scenarios.

Responsibility: Build ROS probes, await graph state, and assert bounded process shutdown.
"""

import math
import os
import threading
import time

import launch
import launch.actions
import launch.launch_description_sources
import launch_testing
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory, GripperCommand
from controller_manager_msgs.srv import ListControllers
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from rclpy.action import ActionClient
from rclpy.qos import HistoryPolicy, QoSPresetProfiles, QoSProfile, ReliabilityPolicy
from rcl_interfaces.srv import GetParameters
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, JointState, PointCloud2

PACKAGE_NAME = 'franka_motion'

DEFAULT_RUNTIME_TIMEOUT = 75.0
ISAAC_RUNTIME_TIMEOUT = 240.0
CAMERA_RUNTIME_TIMEOUT = 120.0
ISAAC_CAMERA_RUNTIME_TIMEOUT = 300.0

_STATE_SAMPLE_DURATION = 3.0
_MIN_JOINT_STATE_SAMPLES = 3
_INITIAL_POSITION_TOLERANCE = 0.2
_STABILITY_TOLERANCE = 0.08
_ARM_JOINTS = tuple(f'panda_joint{i}' for i in range(1, 8))
_HAND_JOINTS = ('panda_finger_joint1', 'panda_finger_joint2')
_EXPECTED_JOINTS = _ARM_JOINTS + _HAND_JOINTS
_ACTIVE_CONTROLLERS = (
    'joint_state_broadcaster',
    'panda_arm_controller',
    'panda_hand_controller',
)
_CLOCK_QOS = QoSProfile(
    depth=10,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.BEST_EFFORT,
)
_SENSOR_DATA_QOS = QoSPresetProfiles.SENSOR_DATA.value


def create_launch_description(launch_filename, launch_arguments):
    package_share = get_package_share_directory(PACKAGE_NAME)
    launch_path = os.path.join(package_share, 'launch', launch_filename)

    return launch.LaunchDescription(
        [
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.PythonLaunchDescriptionSource(
                    launch_path
                ),
                launch_arguments=launch_arguments.items(),
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    )


def wait_for_message(node, msg_type, topic, timeout_sec, qos_profile=10):
    event = threading.Event()
    received_msg = None

    def _cb(msg):
        nonlocal received_msg
        received_msg = msg
        event.set()

    sub = node.create_subscription(msg_type, topic, _cb, qos_profile)
    end_time = time.monotonic() + timeout_sec
    while time.monotonic() < end_time and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        if event.is_set():
            break

    node.destroy_subscription(sub)
    return received_msg if event.is_set() else None


def _wait_for_action(node, action_type, action_name, timeout_sec):
    client = ActionClient(node, action_type, action_name)
    ready = client.wait_for_server(timeout_sec=timeout_sec)
    client.destroy()
    return ready


def _wait_for_service(node, service_type, service_name, timeout_sec):
    client = node.create_client(service_type, service_name)
    try:
        return client.wait_for_service(timeout_sec=timeout_sec)
    finally:
        node.destroy_client(client)


def _wait_for_active_controllers(node, controller_names, timeout_sec):
    client = node.create_client(
        ListControllers,
        '/controller_manager/list_controllers',
    )
    try:
        end_time = time.monotonic() + timeout_sec
        while time.monotonic() < end_time and rclpy.ok():
            if not client.wait_for_service(timeout_sec=0.5):
                continue

            future = client.call_async(ListControllers.Request())
            request_end_time = min(time.monotonic() + 1.0, end_time)
            while time.monotonic() < request_end_time and not future.done():
                rclpy.spin_once(node, timeout_sec=0.1)

            if not future.done():
                continue

            result = future.result()
            if result is None:
                continue

            controller_states = {
                controller.name: controller.state
                for controller in result.controller
            }
            if all(
                controller_states.get(name) == 'active'
                for name in controller_names
            ):
                return True

        return False
    finally:
        node.destroy_client(client)


def _wait_for_collision_object(node, object_id, timeout_sec):
    client = node.create_client(GetPlanningScene, '/get_planning_scene')
    try:
        end_time = time.monotonic() + timeout_sec
        while time.monotonic() < end_time and rclpy.ok():
            if not client.wait_for_service(timeout_sec=0.5):
                continue

            request = GetPlanningScene.Request()
            request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            future = client.call_async(request)
            request_end_time = min(time.monotonic() + 1.0, end_time)
            while time.monotonic() < request_end_time and not future.done():
                rclpy.spin_once(node, timeout_sec=0.1)

            if not future.done() or future.result() is None:
                continue
            for collision_object in future.result().scene.world.collision_objects:
                if collision_object.id == object_id:
                    return collision_object
        return None
    finally:
        node.destroy_client(client)


def _positions_by_joint_name(msg):
    return dict(zip(msg.name, msg.position))


def _load_expected_positions():
    package_share = get_package_share_directory(PACKAGE_NAME)
    config_path = os.path.join(
        package_share,
        'config',
        'initial_positions.yaml',
    )
    with open(config_path, encoding='utf-8') as config_file:
        config = yaml.safe_load(config_file)

    expected_positions = dict(config['initial_positions'])
    expected_positions.update({joint: 0.0 for joint in _HAND_JOINTS})
    return expected_positions


def _collect_joint_state_samples(
    node,
    expected_joints,
    sample_duration_sec,
    timeout_sec,
):
    samples = []
    start_time = None

    def _cb(msg):
        nonlocal start_time
        positions = _positions_by_joint_name(msg)
        if not all(joint in positions for joint in expected_joints):
            return

        if start_time is None:
            start_time = time.monotonic()

        samples.append(positions)

    sub = node.create_subscription(JointState, '/joint_states', _cb, 10)
    end_time = time.monotonic() + timeout_sec
    while time.monotonic() < end_time and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        if (
            start_time is not None
            and time.monotonic() - start_time >= sample_duration_sec
        ):
            break

    node.destroy_subscription(sub)
    return samples


def _assert_default_joint_state(test_case, samples, expected_positions):
    test_case.assertGreaterEqual(
        len(samples),
        _MIN_JOINT_STATE_SAMPLES,
        'Not enough /joint_states samples with all Panda joints',
    )

    for sample in samples:
        for joint_name in expected_positions:
            joint_position = sample[joint_name]
            test_case.assertTrue(
                math.isfinite(joint_position) and abs(joint_position) < 10.0,
                f'{joint_name} has invalid position {joint_position}',
            )

    last_sample = samples[-1]
    for joint_name, expected_position in expected_positions.items():
        test_case.assertAlmostEqual(
            last_sample[joint_name],
            expected_position,
            delta=_INITIAL_POSITION_TOLERANCE,
            msg=(
                f'{joint_name} is not near its default position '
                f'{expected_position}'
            ),
        )

    for joint_name in expected_positions:
        joint_positions = [sample[joint_name] for sample in samples]
        drift = max(joint_positions) - min(joint_positions)
        test_case.assertLessEqual(
            drift,
            _STABILITY_TOLERANCE,
            f'{joint_name} drifted by {drift} while idle',
        )


def assert_moveit_robot_ready(test_case, node, timeout_sec):
    expected_positions = _load_expected_positions()

    test_case.assertIsNotNone(
        wait_for_message(node, Clock, '/clock', timeout_sec, _CLOCK_QOS),
        f'/clock did not publish within {timeout_sec:.1f}s',
    )
    test_case.assertTrue(
        _wait_for_service(
            node,
            GetParameters,
            '/move_group/get_parameters',
            timeout_sec,
        ),
        f'/move_group/get_parameters was not available within {timeout_sec:.1f}s',
    )
    test_case.assertTrue(
        _wait_for_active_controllers(node, _ACTIVE_CONTROLLERS, timeout_sec),
        f'Controllers {", ".join(_ACTIVE_CONTROLLERS)} were not active within {timeout_sec:.1f}s',
    )
    test_case.assertTrue(
        _wait_for_action(
            node,
            FollowJointTrajectory,
            '/panda_arm_controller/follow_joint_trajectory',
            timeout_sec,
        ),
        f'/panda_arm_controller/follow_joint_trajectory was not available within {timeout_sec:.1f}s',
    )
    test_case.assertTrue(
        _wait_for_action(
            node,
            GripperCommand,
            '/panda_gripper/gripper_cmd',
            timeout_sec,
        ),
        f'/panda_gripper/gripper_cmd was not available within {timeout_sec:.1f}s',
    )

    joint_state_samples = _collect_joint_state_samples(
        node,
        _EXPECTED_JOINTS,
        _STATE_SAMPLE_DURATION,
        timeout_sec,
    )
    _assert_default_joint_state(test_case, joint_state_samples, expected_positions)


def assert_table_collision_ready(
    test_case,
    node,
    timeout_sec,
    *,
    expected_dimensions,
    expected_position,
    expected_world_position,
):
    table = _wait_for_collision_object(node, 'table', timeout_sec)
    test_case.assertIsNotNone(table, 'MoveIt PlanningScene does not contain the table')
    test_case.assertIn(table.header.frame_id, ('panda_link0', 'world'))
    test_case.assertEqual(len(table.primitives), 1)
    test_case.assertEqual(len(table.primitive_poses), 1)
    for actual, expected in zip(table.primitives[0].dimensions, expected_dimensions):
        test_case.assertAlmostEqual(actual, expected, delta=1e-4)
    object_pose = table.pose
    primitive_pose = table.primitive_poses[0]
    test_case.assertAlmostEqual(object_pose.orientation.x, 0.0, delta=1e-6)
    test_case.assertAlmostEqual(object_pose.orientation.y, 0.0, delta=1e-6)
    test_case.assertAlmostEqual(object_pose.orientation.z, 0.0, delta=1e-6)
    test_case.assertAlmostEqual(object_pose.orientation.w, 1.0, delta=1e-6)
    test_case.assertAlmostEqual(primitive_pose.orientation.x, 0.0, delta=1e-6)
    test_case.assertAlmostEqual(primitive_pose.orientation.y, 0.0, delta=1e-6)
    test_case.assertAlmostEqual(primitive_pose.orientation.z, 0.0, delta=1e-6)
    test_case.assertAlmostEqual(primitive_pose.orientation.w, 1.0, delta=1e-6)
    position = (
        expected_position
        if table.header.frame_id == 'panda_link0'
        else expected_world_position
    )
    for actual, expected in zip(
        (
            object_pose.position.x + primitive_pose.position.x,
            object_pose.position.y + primitive_pose.position.y,
            object_pose.position.z + primitive_pose.position.z,
        ),
        position,
    ):
        test_case.assertAlmostEqual(actual, expected, delta=1e-4)


def assert_stereo_camera_ready(test_case, node, timeout_sec):
    expected_topics = (
        (Image, '/table_stereo/left/image_raw'),
        (Image, '/table_stereo/right/image_raw'),
        (CameraInfo, '/table_stereo/left/camera_info'),
        (CameraInfo, '/table_stereo/right/camera_info'),
        (Image, '/table_stereo/depth'),
        (PointCloud2, '/table_stereo/points'),
    )
    for msg_type, topic in expected_topics:
        test_case.assertIsNotNone(
            wait_for_message(node, msg_type, topic, timeout_sec, _SENSOR_DATA_QOS),
            f'{topic} did not publish a message',
        )
