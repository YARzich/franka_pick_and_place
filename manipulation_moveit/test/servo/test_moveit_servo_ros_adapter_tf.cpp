// Responsibility: Verifies exact-stamp target canonicalization, tip offsets, and fresh TF errors.

#include <chrono>
#include <cmath>
#include <future>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

#include "../action_test_support.hpp"
#include "manipulation_moveit/servo/moveit_servo_ros_adapter.hpp"

namespace manipulation::moveit
{
namespace
{

using namespace std::chrono_literals;
using test::ExecutorThread;
using RclcppFixture = test::RclcppFixture;
constexpr double kPi = 3.14159265358979323846;

MoveItServoRosAdapterConfig tf_config()
{
  MoveItServoRosAdapterConfig config;
  config.planning_frame = "servo_test_base";
  config.task_frame = "servo_test_task";
  config.servo_tip_frame = "servo_test_tip";
  config.joint_names = {"servo_joint_1", "servo_joint_2"};
  config.endpoints.pose_command_topic = "/servo_adapter_tf/pose";
  config.endpoints.command_type_service = "/servo_adapter_tf/mode";
  config.endpoints.pause_service = "/servo_adapter_tf/pause";
  config.endpoints.status_topic = "/servo_adapter_tf/status";
  config.endpoints.joint_state_topic = "/servo_adapter_tf/joints";
  config.poll_period = 1ms;
  config.current_tf_max_age = 500ms;
  return config;
}

geometry_msgs::msg::TransformStamped transform(
  const builtin_interfaces::msg::Time & stamp,
  const std::string & parent,
  const std::string & child,
  const double x,
  const double z)
{
  geometry_msgs::msg::TransformStamped value;
  value.header.stamp = stamp;
  value.header.frame_id = parent;
  value.child_frame_id = child;
  value.transform.translation.x = x;
  value.transform.translation.z = z;
  value.transform.rotation.w = 1.0;
  return value;
}

geometry_msgs::msg::PoseStamped task_goal(
  const builtin_interfaces::msg::Time & stamp)
{
  geometry_msgs::msg::PoseStamped goal;
  goal.header.stamp = stamp;
  goal.header.frame_id = "servo_test_sensor";
  goal.pose.position.x = 0.2;
  goal.pose.position.z = 1.0;
  goal.pose.orientation.z = std::sin(kPi / 4.0);
  goal.pose.orientation.w = std::cos(kPi / 4.0);
  return goal;
}

TEST_F(RclcppFixture, ExactStampCanonicalizationAppliesTaskToTipOffset)
{
  auto adapter_node = rclcpp::Node::make_shared("servo_adapter_tf_exact");
  auto broadcaster_node = rclcpp::Node::make_shared("servo_adapter_tf_exact_source");
  ExecutorThread executor{adapter_node, broadcaster_node};
  tf2_ros::TransformBroadcaster dynamic_broadcaster{broadcaster_node};
  tf2_ros::StaticTransformBroadcaster static_broadcaster{broadcaster_node};
  MoveItServoRosAdapter adapter{*adapter_node, tf_config()};

  const auto stamp = static_cast<builtin_interfaces::msg::Time>(adapter_node->now());
  static_broadcaster.sendTransform(transform(
      stamp, "servo_test_tip", "servo_test_task", 0.0, 0.1));
  auto future = std::async(std::launch::async, [&adapter, stamp]() {
        return adapter.prepare_target(
          task_goal(stamp), std::chrono::steady_clock::now() + 2s);
    });
  while (future.wait_for(0ms) != std::future_status::ready) {
    dynamic_broadcaster.sendTransform(transform(
        stamp, "servo_test_base", "servo_test_sensor", 1.0, 0.0));
    std::this_thread::sleep_for(5ms);
  }

  const auto result = future.get();
  ASSERT_TRUE(result.status.succeeded()) << result.status.detail();
  ASSERT_TRUE(result.target.has_value());
  EXPECT_EQ(result.target->task_goal.header.frame_id, "servo_test_base");
  EXPECT_EQ(result.target->task_goal.header.stamp, stamp);
  EXPECT_NEAR(result.target->task_goal.pose.position.x, 1.2, 1.0e-9);
  EXPECT_NEAR(result.target->task_goal.pose.position.z, 1.0, 1.0e-9);
  EXPECT_EQ(result.target->servo_tip_goal.header.frame_id, "servo_test_base");
  EXPECT_EQ(result.target->servo_tip_goal.header.stamp, stamp);
  EXPECT_NEAR(result.target->servo_tip_goal.pose.position.x, 1.2, 1.0e-9);
  EXPECT_NEAR(result.target->servo_tip_goal.pose.position.z, 0.9, 1.0e-9);
}

TEST_F(RclcppFixture, LatestFreshTaskTransformProducesDistanceAndShortestAngle)
{
  auto adapter_node = rclcpp::Node::make_shared("servo_adapter_tf_error");
  auto broadcaster_node = rclcpp::Node::make_shared("servo_adapter_tf_error_source");
  ExecutorThread executor{adapter_node, broadcaster_node};
  tf2_ros::TransformBroadcaster dynamic_broadcaster{broadcaster_node};
  tf2_ros::StaticTransformBroadcaster static_broadcaster{broadcaster_node};
  MoveItServoRosAdapter adapter{*adapter_node, tf_config()};

  const auto goal_stamp = static_cast<builtin_interfaces::msg::Time>(adapter_node->now());
  static_broadcaster.sendTransform(transform(
      goal_stamp, "servo_test_tip", "servo_test_task", 0.0, 0.1));
  auto prepare_future = std::async(std::launch::async, [&adapter, goal_stamp]() {
        return adapter.prepare_target(
          task_goal(goal_stamp), std::chrono::steady_clock::now() + 2s);
    });
  while (prepare_future.wait_for(0ms) != std::future_status::ready) {
    dynamic_broadcaster.sendTransform(transform(
        goal_stamp, "servo_test_base", "servo_test_sensor", 1.0, 0.0));
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_TRUE(prepare_future.get().status.succeeded());

  auto observation_future = std::async(std::launch::async, [&adapter]() {
        return adapter.latest_observation(std::chrono::steady_clock::now() + 2s);
    });
  while (observation_future.wait_for(0ms) != std::future_status::ready) {
    dynamic_broadcaster.sendTransform(transform(
        static_cast<builtin_interfaces::msg::Time>(broadcaster_node->now()),
        "servo_test_base", "servo_test_tip", 1.1, 0.8));
    std::this_thread::sleep_for(5ms);
  }

  const auto result = observation_future.get();
  ASSERT_TRUE(result.status.succeeded()) << result.status.detail();
  ASSERT_TRUE(result.observation.pose_sample.available);
  EXPECT_TRUE(result.observation.pose_sample.fresh);
  EXPECT_TRUE(result.observation.pose_sample.valid);
  EXPECT_NE(result.observation.pose_sample.sequence, 0U);
  EXPECT_NEAR(
    result.observation.pose_sample.position_error_m,
    std::sqrt(0.02), 1.0e-6);
  EXPECT_NEAR(result.observation.pose_sample.orientation_error_rad, kPi / 2.0, 1.0e-6);
}

}  // namespace
}  // namespace manipulation::moveit
