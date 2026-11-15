"""Responsibility: Reject malformed stream stamps without refreshing health or killing the node."""

import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
from manipulation_msgs.msg import HealthComponentStatus
from manipulation_msgs.msg import SystemHealthSnapshot
import pytest
import rclpy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState


@pytest.mark.launch_test
def generate_test_description():
    """Run a monitor with short freshness bounds and an explicit clock requirement."""
    monitor = launch_ros.actions.Node(
        package='manipulation', executable='system_health_monitor',
        name='invalid_stamp_health', output='screen',
        parameters=[{
            'use_sim_time': False,
            'require_sim_clock': True,
            'startup_grace_sec': 0.0,
            'joint_states_topic': '/invalid_stamp_health/joints',
            'required_joint_names': ['test_joint'],
            'joint_states_warn_after_sec': 0.2,
            'joint_states_stale_after_sec': 0.4,
            'clock_warn_after_sec': 0.2,
            'clock_stale_after_sec': 0.4,
            'health_publish_period_sec': 0.1,
        }],
    )
    description = launch.LaunchDescription()
    description.add_action(monitor)
    description.add_action(launch_testing.actions.ReadyToTest())
    return description


class TestInvalidHealthStamps(unittest.TestCase):
    """Check invalid-first, invalid-after-valid, and recovery for both streams."""

    def test_invalid_stamps_do_not_count_as_observations(self):
        rclpy.init()
        node = rclpy.create_node('invalid_stamp_health_test')
        messages = []
        subscription = node.create_subscription(
            SystemHealthSnapshot, '/invalid_stamp_health/health', messages.append, 10)
        joint_publisher = node.create_publisher(
            JointState, '/invalid_stamp_health/joints', 10)
        clock_publisher = node.create_publisher(Clock, '/clock', 10)

        def spin_until(predicate, timeout=5.0):
            deadline = time.monotonic() + timeout
            while not predicate() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.02)
            self.assertTrue(predicate(), 'monitor stopped or did not publish the expected health')

        def publish(sec, nanosec):
            joint = JointState()
            joint.header.stamp.sec = sec
            joint.header.stamp.nanosec = nanosec
            joint.name = ['test_joint']
            joint.position = [0.0]
            joint_publisher.publish(joint)
            clock = Clock()
            clock.clock = joint.header.stamp
            clock_publisher.publish(clock)

        def streams():
            ids = (HealthComponentStatus.COMPONENT_JOINT_STATES,
                   HealthComponentStatus.COMPONENT_SIMULATION_CLOCK)
            return [c for c in messages[-1].components if c.component_id in ids]

        try:
            spin_until(lambda: bool(messages) and joint_publisher.get_subscription_count() > 0
                       and clock_publisher.get_subscription_count() > 0)
            for stamp in ((-1, 0), (1, 1_000_000_000)):
                before = messages[-1].sequence
                publish(*stamp)
                spin_until(lambda: messages[-1].sequence > before + 2)
                self.assertEqual(len(streams()), 2)
                self.assertTrue(all(not component.observed for component in streams()))

            now = node.get_clock().now().to_msg()
            publish(now.sec, now.nanosec)
            spin_until(lambda: all(component.observed for component in streams()))
            deadline = time.monotonic() + 0.8
            while time.monotonic() < deadline:
                publish(-1, 0)
                publish(1, 1_000_000_000)
                rclpy.spin_once(node, timeout_sec=0.02)
            spin_until(lambda: all(component.level == HealthComponentStatus.LEVEL_STALE
                                   for component in streams()))
            self.assertTrue(all(component.steady_observation_age.sec > 0
                                or component.steady_observation_age.nanosec > 400_000_000
                                for component in streams()))
            publish(now.sec + 1, now.nanosec)
            spin_until(lambda: all(component.level == HealthComponentStatus.LEVEL_OK
                                   for component in streams()))
        finally:
            node.destroy_subscription(subscription)
            node.destroy_node()
            rclpy.shutdown()
