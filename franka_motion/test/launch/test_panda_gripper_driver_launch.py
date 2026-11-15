"""
Exercise the legacy Panda gripper-driver compatibility executable.

Responsibility: Verify gripper action translation, joint-state feedback, and bounded shutdown.
"""

import math
import time
import unittest

from action_msgs.msg import GoalStatus
from control_msgs.action import GripperCommand
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


ACTION_NAME = '/test_panda_gripper_driver/gripper_cmd'
COMMAND_TOPIC = '/test_panda_gripper_driver/commands'
JOINT_STATES_TOPIC = '/test_panda_gripper_driver/joint_states'


@pytest.mark.launch_test
def generate_test_description():
    gripper_driver = launch_ros.actions.Node(
        package='franka_motion',
        executable='panda_gripper_driver',
        name='test_panda_gripper_driver',
        output='screen',
        parameters=[{
            'action_name': ACTION_NAME,
            'command_topic': COMMAND_TOPIC,
            'joint_states_topic': JOINT_STATES_TOPIC,
            'goal_tolerance': 0.001,
            'state_timeout_sec': 0.35,
            'stall_timeout_sec': 0.15,
            'stall_width_threshold': 0.0005,
            'goal_timeout_sec': 0.6,
            'action_monitor_rate': 100.0,
            'allow_stalling': True,
            'command_joint_count': 1,
            'command_joint_min_position': 0.001,
            'command_joint_max_position': 0.039,
            'finger_symmetry_tolerance': 0.003,
            'stall_finger_reverse_tolerance': 0.002,
        }],
    )

    return launch.LaunchDescription([
        gripper_driver,
        launch_testing.actions.ReadyToTest(),
    ]), {'gripper_driver': gripper_driver}


class TestPandaGripperDriver(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_panda_gripper_driver_client')
        cls.commands = []
        cls.feedback = []
        cls.command_subscription = cls.node.create_subscription(
            Float64MultiArray,
            COMMAND_TOPIC,
            lambda message: cls.commands.append(list(message.data)),
            10,
        )
        cls.joint_state_publisher = cls.node.create_publisher(
            JointState,
            JOINT_STATES_TOPIC,
            10,
        )
        cls.action_client = ActionClient(cls.node, GripperCommand, ACTION_NAME)

        if not cls.action_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('panda_gripper_driver action server did not start')

        deadline = time.monotonic() + 10.0
        while (
            cls.joint_state_publisher.get_subscription_count() == 0
            or cls.node.count_publishers(COMMAND_TOPIC) == 0
        ):
            if time.monotonic() >= deadline:
                raise RuntimeError('panda_gripper_driver topics did not connect')
            rclpy.spin_once(cls.node, timeout_sec=0.02)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _spin_until(self, predicate, timeout, failure_message):
        deadline = time.monotonic() + timeout
        while not predicate():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                self.fail(failure_message)
            rclpy.spin_once(self.node, timeout_sec=min(0.02, remaining))

    def _wait_future(self, future, timeout, failure_message):
        self._spin_until(future.done, timeout, failure_message)
        result = future.result()
        self.assertIsNotNone(result)
        return result

    def _feedback_callback(self, feedback_message):
        self.feedback.append(feedback_message.feedback)

    def _publish_joint_state(self, left, right, effort=1.0):
        joint_state = JointState()
        joint_state.name = ['panda_finger_joint1', 'panda_finger_joint2']
        joint_state.position = [left, right]
        joint_state.effort = [effort * 0.5, effort * 0.5]
        self.joint_state_publisher.publish(joint_state)

    def _publish_state(self, width, effort=1.0):
        self._publish_joint_state(width * 0.5, width * 0.5, effort)

    def _wait_feedback_width(self, width, timeout=1.0):
        self._spin_until(
            lambda: any(math.isclose(item.position, width, abs_tol=1e-9)
                        for item in self.feedback),
            timeout,
            f'did not receive feedback for gripper width {width}',
        )

    def _send_goal(self, target_width):
        self.commands.clear()
        self.feedback.clear()

        goal = GripperCommand.Goal()
        goal.command.position = target_width
        goal.command.max_effort = 0.0
        goal_handle = self._wait_future(
            self.action_client.send_goal_async(
                goal,
                feedback_callback=self._feedback_callback,
            ),
            2.0,
            'gripper goal was not acknowledged',
        )
        self.assertTrue(goal_handle.accepted)
        self._spin_until(
            lambda: bool(self.commands),
            1.0,
            'gripper command was not published',
        )
        return goal_handle, goal_handle.get_result_async()

    def _set_current_width(self, width):
        _, result_future = self._send_goal(width)
        self._publish_state(width)
        response = self._wait_future(
            result_future,
            1.0,
            f'could not establish initial gripper width {width}',
        )
        self.assertEqual(response.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertTrue(response.result.reached_goal)

    def test_01_open_succeeds_and_uses_single_joint_command(self):
        self._set_current_width(0.0)
        _, result_future = self._send_goal(0.08)
        self.assertEqual(self.commands[0], [0.039])

        self._publish_state(0.0)
        self._wait_feedback_width(0.0)
        self._publish_state(0.08, effort=2.0)

        response = self._wait_future(result_future, 1.0, 'open goal did not finish')
        self.assertEqual(response.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertTrue(response.result.reached_goal)
        self.assertFalse(response.result.stalled)
        self.assertAlmostEqual(response.result.position, 0.08)
        self.assertAlmostEqual(response.result.effort, 2.0)

    def test_02_close_stall_succeeds_only_after_closing_progress(self):
        self._set_current_width(0.08)
        _, result_future = self._send_goal(0.0)
        self.assertEqual(self.commands[0], [0.001])

        self._publish_state(0.08)
        self._wait_feedback_width(0.08)
        self.assertFalse(result_future.done())

        self._publish_state(0.04)
        self._wait_feedback_width(0.04)
        response = self._wait_future(result_future, 1.0, 'progress stall did not finish')

        self.assertEqual(response.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertFalse(response.result.reached_goal)
        self.assertTrue(response.result.stalled)
        self.assertAlmostEqual(response.result.position, 0.04)

    def test_03_close_without_progress_aborts(self):
        self._set_current_width(0.08)
        _, result_future = self._send_goal(0.0)
        self._publish_state(0.08)
        self._wait_feedback_width(0.08)

        response = self._wait_future(result_future, 1.0, 'no-progress goal did not finish')
        self.assertEqual(response.status, GoalStatus.STATUS_ABORTED)
        self.assertFalse(response.result.reached_goal)
        self.assertFalse(response.result.stalled)
        self.assertAlmostEqual(response.result.position, 0.08)

    def test_04_cancel_holds_the_last_joint_position(self):
        self._set_current_width(0.02)
        goal_handle, result_future = self._send_goal(0.08)
        self._publish_joint_state(0.014, 0.006)
        self._wait_feedback_width(0.02)
        command_count = len(self.commands)

        cancel_response = self._wait_future(
            goal_handle.cancel_goal_async(),
            1.0,
            'cancel request did not finish',
        )
        self.assertTrue(cancel_response.goals_canceling)
        response = self._wait_future(result_future, 1.0, 'canceled goal did not finish')
        self.assertEqual(response.status, GoalStatus.STATUS_CANCELED)
        self.assertFalse(response.result.reached_goal)
        self.assertFalse(response.result.stalled)

        self._spin_until(
            lambda: len(self.commands) > command_count,
            1.0,
            'cancel did not publish a hold command',
        )
        self.assertEqual(self.commands[-1], [0.014])

    def test_05_goal_timeout_aborts_while_states_remain_fresh(self):
        self._set_current_width(0.01)
        _, result_future = self._send_goal(0.07)
        widths = (0.01, 0.02)
        index = 0
        deadline = time.monotonic() + 1.2
        while not result_future.done() and time.monotonic() < deadline:
            self._publish_state(widths[index % len(widths)])
            index += 1
            rclpy.spin_once(self.node, timeout_sec=0.02)

        response = self._wait_future(result_future, 0.2, 'timed goal did not abort')
        self.assertEqual(response.status, GoalStatus.STATUS_ABORTED)
        self.assertFalse(response.result.reached_goal)
        self.assertFalse(response.result.stalled)

    def test_06_one_sided_contact_stall_succeeds(self):
        self._set_current_width(0.08)
        _, result_future = self._send_goal(0.0)
        self._publish_state(0.08)
        self._wait_feedback_width(0.08)

        self._publish_joint_state(0.04, 0.0)
        self._wait_feedback_width(0.04)
        response = self._wait_future(result_future, 1.0, 'one-sided stall did not finish')

        self.assertEqual(response.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertFalse(response.result.reached_goal)
        self.assertTrue(response.result.stalled)
        self.assertAlmostEqual(response.result.position, 0.04)

    def test_07_asymmetric_contact_succeeds_when_both_fingers_moved(self):
        self._set_current_width(0.08)
        _, result_future = self._send_goal(0.0)
        self._publish_state(0.08)
        self._wait_feedback_width(0.08)

        self._publish_joint_state(0.033, 0.017)
        self._wait_feedback_width(0.05)
        response = self._wait_future(result_future, 1.0, 'contact stall did not finish')

        self.assertEqual(response.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertFalse(response.result.reached_goal)
        self.assertTrue(response.result.stalled)
        self.assertAlmostEqual(response.result.position, 0.05)

    def test_08_reverse_finger_motion_aborts(self):
        self._set_current_width(0.06)
        _, result_future = self._send_goal(0.0)
        self._publish_state(0.06)
        self._wait_feedback_width(0.06)

        self._publish_joint_state(0.035, 0.005)
        self._wait_feedback_width(0.04)
        response = self._wait_future(result_future, 1.0, 'reverse-motion stall did not finish')

        self.assertEqual(response.status, GoalStatus.STATUS_ABORTED)
        self.assertFalse(response.result.reached_goal)
        self.assertFalse(response.result.stalled)
        self.assertAlmostEqual(response.result.position, 0.04)

    def test_09_positive_max_effort_is_rejected_without_controller_command(self):
        # Drain any subscription work from the preceding terminal goal before
        # establishing the no-command baseline for this rejected request.
        quiet_deadline = time.monotonic() + 0.1
        while time.monotonic() < quiet_deadline:
            rclpy.spin_once(self.node, timeout_sec=0.01)
        self.commands.clear()

        goal = GripperCommand.Goal()
        goal.command.position = 0.0
        goal.command.max_effort = 1.0
        goal_handle = self._wait_future(
            self.action_client.send_goal_async(goal),
            2.0,
            'positive-effort gripper goal was not acknowledged',
        )

        self.assertFalse(goal_handle.accepted)
        rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertEqual(self.commands, [])


@launch_testing.post_shutdown_test()
class TestPandaGripperDriverShutdown(unittest.TestCase):
    def test_process_exits_cleanly(self, proc_info, gripper_driver):
        launch_testing.asserts.assertExitCodes(proc_info, process=gripper_driver)
