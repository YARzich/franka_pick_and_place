// Responsibility: Verifies Servo service sequencing, command publication, and feedback freshness.

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "moveit_msgs/msg/servo_status.hpp"
#include "moveit_msgs/srv/servo_command_type.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "../action_test_support.hpp"
#include "manipulation_moveit/servo/moveit_servo_ros_adapter.hpp"

namespace manipulation::moveit
{
namespace
{

using namespace std::chrono_literals;
using test::ExecutorThread;
using RclcppFixture = test::RclcppFixture;

MoveItServoRosAdapterConfig io_config(const std::string & prefix)
{
  MoveItServoRosAdapterConfig config;
  config.planning_frame = "servo_io_frame";
  config.task_frame = "servo_io_frame";
  config.servo_tip_frame = "servo_io_frame";
  config.joint_names = {"servo_joint_1", "servo_joint_2"};
  config.endpoints.pose_command_topic = prefix + "/pose";
  config.endpoints.command_type_service = prefix + "/mode";
  config.endpoints.pause_service = prefix + "/pause";
  config.endpoints.status_topic = prefix + "/status";
  config.endpoints.joint_state_topic = prefix + "/joints";
  config.poll_period = 1ms;
  config.current_tf_max_age = 500ms;
  config.joint_state_max_age = 40ms;
  config.status_max_age = 40ms;
  return config;
}

geometry_msgs::msg::PoseStamped identity_goal(rclcpp::Node & node)
{
  geometry_msgs::msg::PoseStamped goal;
  goal.header.stamp = static_cast<builtin_interfaces::msg::Time>(node.now());
  goal.header.frame_id = "servo_io_frame";
  goal.pose.orientation.w = 1.0;
  return goal;
}

sensor_msgs::msg::JointState joint_state(
  const builtin_interfaces::msg::Time & stamp,
  const bool complete = true)
{
  sensor_msgs::msg::JointState message;
  message.header.stamp = stamp;
  if (complete) {
    message.name = {"servo_joint_1", "servo_joint_2"};
    message.velocity = {-0.2, 0.1};
  } else {
    message.name = {"servo_joint_1"};
    message.velocity = {-0.2};
  }
  return message;
}

TEST_F(RclcppFixture, PauseModeAndResumeAlwaysUseStableServices)
{
  const auto config = io_config("/servo_adapter_services");
  auto adapter_node = rclcpp::Node::make_shared("servo_adapter_services_client");
  auto server_node = rclcpp::Node::make_shared("servo_adapter_services_server");
  std::mutex calls_mutex;
  std::vector<bool> pause_calls;
  std::atomic<std::int8_t> command_type{-1};

  auto mode_service = server_node->create_service<moveit_msgs::srv::ServoCommandType>(
    config.endpoints.command_type_service,
    [&command_type](
      const moveit_msgs::srv::ServoCommandType::Request::SharedPtr request,
      moveit_msgs::srv::ServoCommandType::Response::SharedPtr response)
    {
      command_type.store(request->command_type);
      response->success = true;
    });
  auto pause_service = server_node->create_service<std_srvs::srv::SetBool>(
    config.endpoints.pause_service,
    [&calls_mutex, &pause_calls](
      const std_srvs::srv::SetBool::Request::SharedPtr request,
      std_srvs::srv::SetBool::Response::SharedPtr response)
    {
      const std::lock_guard<std::mutex> lock(calls_mutex);
      pause_calls.push_back(request->data);
      response->success = true;
    });
  (void)mode_service;
  (void)pause_service;

  MoveItServoRosAdapter adapter{*adapter_node, config};
  ExecutorThread executor{adapter_node, server_node};
  const auto deadline = []() {return std::chrono::steady_clock::now() + 2s;};

  EXPECT_TRUE(adapter.pause(deadline()).succeeded());
  EXPECT_TRUE(adapter.pause(deadline()).succeeded());
  EXPECT_TRUE(adapter.configure_pose_mode(deadline()).succeeded());
  EXPECT_TRUE(adapter.unpause(deadline()).succeeded());
  const auto observation = adapter.latest_observation(deadline());
  EXPECT_EQ(
    observation.observation.backend_state,
    manipulation::servo::ServoBackendState::kRunning);

  EXPECT_EQ(
    command_type.load(), moveit_msgs::srv::ServoCommandType::Request::POSE);
  const std::lock_guard<std::mutex> lock(calls_mutex);
  ASSERT_EQ(pause_calls.size(), 3U);
  EXPECT_TRUE(pause_calls[0]);
  EXPECT_TRUE(pause_calls[1]);
  EXPECT_FALSE(pause_calls[2]);
}

TEST_F(RclcppFixture, MissingServiceHonorsSteadyDeadline)
{
  const auto config = io_config("/servo_adapter_missing_service");
  auto adapter_node = rclcpp::Node::make_shared("servo_adapter_missing_service_client");
  MoveItServoRosAdapter adapter{*adapter_node, config};

  const auto started_at = std::chrono::steady_clock::now();
  const auto result = adapter.pause(started_at + 50ms);
  const auto elapsed = std::chrono::steady_clock::now() - started_at;
  EXPECT_EQ(result.error(), MoveItServoRosError::kTimeout);
  EXPECT_LT(elapsed, 300ms);
}

TEST_F(RclcppFixture, ReadinessRequiresServicesCommandConsumerAndFreshFeedback)
{
  const auto config = io_config("/servo_adapter_readiness");
  auto adapter_node = rclcpp::Node::make_shared("servo_adapter_readiness_client");
  auto backend_node = rclcpp::Node::make_shared("servo_adapter_readiness_backend");

  auto mode_service = backend_node->create_service<moveit_msgs::srv::ServoCommandType>(
    config.endpoints.command_type_service,
    [](
      moveit_msgs::srv::ServoCommandType::Request::SharedPtr,
      moveit_msgs::srv::ServoCommandType::Response::SharedPtr response)
    {
      response->success = true;
    });
  auto pause_service = backend_node->create_service<std_srvs::srv::SetBool>(
    config.endpoints.pause_service,
    [](
      std_srvs::srv::SetBool::Request::SharedPtr,
      std_srvs::srv::SetBool::Response::SharedPtr response)
    {
      response->success = true;
    });
  auto pose_subscription = backend_node->create_subscription<geometry_msgs::msg::PoseStamped>(
    config.endpoints.pose_command_topic,
    rclcpp::SystemDefaultsQoS(),
    [](geometry_msgs::msg::PoseStamped::ConstSharedPtr) {});
  auto status_publisher = backend_node->create_publisher<moveit_msgs::msg::ServoStatus>(
    config.endpoints.status_topic, rclcpp::SystemDefaultsQoS());
  auto joint_publisher = backend_node->create_publisher<sensor_msgs::msg::JointState>(
    config.endpoints.joint_state_topic, rclcpp::SystemDefaultsQoS());
  (void)mode_service;
  (void)pause_service;
  (void)pose_subscription;

  MoveItServoRosAdapter adapter{*adapter_node, config};
  ExecutorThread executor{adapter_node, backend_node};
  EXPECT_FALSE(adapter.backend_ready());

  moveit_msgs::msg::ServoStatus status;
  status.code = moveit_msgs::msg::ServoStatus::NO_WARNING;
  status.message = "ready";
  const auto ready_deadline = std::chrono::steady_clock::now() + 2s;
  do {
    status_publisher->publish(status);
    joint_publisher->publish(joint_state(
        static_cast<builtin_interfaces::msg::Time>(backend_node->now())));
    std::this_thread::sleep_for(5ms);
  } while (!adapter.backend_ready() && std::chrono::steady_clock::now() < ready_deadline);

  EXPECT_TRUE(adapter.backend_ready());
  ASSERT_TRUE(adapter.pause(std::chrono::steady_clock::now() + 1s).succeeded());
  const auto paused_status_deadline = std::chrono::steady_clock::now() + 60ms;
  while (std::chrono::steady_clock::now() < paused_status_deadline) {
    joint_publisher->publish(joint_state(
        static_cast<builtin_interfaces::msg::Time>(backend_node->now())));
    std::this_thread::sleep_for(5ms);
  }
  EXPECT_TRUE(adapter.backend_ready());

  ASSERT_TRUE(adapter.unpause(std::chrono::steady_clock::now() + 1s).succeeded());
  EXPECT_FALSE(adapter.backend_ready());
}

TEST_F(RclcppFixture, RepeatedPublishUsesFreshNodeStampsAndCanonicalFrame)
{
  const auto config = io_config("/servo_adapter_publish");
  auto adapter_node = rclcpp::Node::make_shared("servo_adapter_publish_client");
  auto observer_node = rclcpp::Node::make_shared("servo_adapter_publish_observer");
  std::atomic<std::size_t> received{0U};
  auto subscription = observer_node->create_subscription<geometry_msgs::msg::PoseStamped>(
    config.endpoints.pose_command_topic,
    rclcpp::SystemDefaultsQoS(),
    [&received](geometry_msgs::msg::PoseStamped::ConstSharedPtr) {
      received.fetch_add(1U);
    });
  (void)subscription;

  MoveItServoRosAdapter adapter{*adapter_node, config};
  ExecutorThread executor{adapter_node, observer_node};
  ASSERT_TRUE(adapter.prepare_target(
      identity_goal(*adapter_node), std::chrono::steady_clock::now() + 1s).status.succeeded());
  const auto discovery_deadline = std::chrono::steady_clock::now() + 1s;
  while (observer_node->count_publishers(config.endpoints.pose_command_topic) == 0U &&
    std::chrono::steady_clock::now() < discovery_deadline)
  {
    std::this_thread::sleep_for(5ms);
  }

  const auto first = adapter.publish_target();
  std::this_thread::sleep_for(2ms);
  const auto second = adapter.publish_target();
  ASSERT_TRUE(first.status.succeeded()) << first.status.detail();
  ASSERT_TRUE(second.status.succeeded()) << second.status.detail();
  ASSERT_TRUE(first.published_target.has_value());
  ASSERT_TRUE(second.published_target.has_value());
  EXPECT_EQ(first.published_target->header.frame_id, config.planning_frame);
  EXPECT_EQ(second.published_target->header.frame_id, config.planning_frame);
  EXPECT_NE(first.published_target->header.stamp, second.published_target->header.stamp);

  const auto receive_deadline = std::chrono::steady_clock::now() + 1s;
  while (received.load() < 2U && std::chrono::steady_clock::now() < receive_deadline) {
    std::this_thread::sleep_for(5ms);
  }
  EXPECT_GE(received.load(), 2U);

  geometry_msgs::msg::PoseStamped invalid_goal;
  invalid_goal.header.frame_id = config.planning_frame;
  invalid_goal.pose.orientation.w = 1.0;
  EXPECT_FALSE(adapter.prepare_target(
      invalid_goal, std::chrono::steady_clock::now() + 200ms).status.succeeded());
  EXPECT_EQ(adapter.publish_target().status.error(), MoveItServoRosError::kInvalidData);
}

TEST_F(RclcppFixture, FeedbackReportsStatusVelocityFreshnessAndBackwardStamp)
{
  const auto config = io_config("/servo_adapter_feedback");
  auto adapter_node = rclcpp::Node::make_shared("servo_adapter_feedback_client");
  auto source_node = rclcpp::Node::make_shared("servo_adapter_feedback_source");
  auto status_publisher = source_node->create_publisher<moveit_msgs::msg::ServoStatus>(
    config.endpoints.status_topic, rclcpp::SystemDefaultsQoS());
  auto joint_publisher = source_node->create_publisher<sensor_msgs::msg::JointState>(
    config.endpoints.joint_state_topic, rclcpp::SystemDefaultsQoS());
  MoveItServoRosAdapter adapter{*adapter_node, config};
  ExecutorThread executor{adapter_node, source_node};
  ASSERT_TRUE(adapter.prepare_target(
      identity_goal(*adapter_node), std::chrono::steady_clock::now() + 1s).status.succeeded());

  moveit_msgs::msg::ServoStatus status;
  status.code = moveit_msgs::msg::ServoStatus::DECELERATE_FOR_COLLISION;
  status.message = "near collision";
  MoveItServoObservationResult observation;
  const auto receive_deadline = std::chrono::steady_clock::now() + 2s;
  do {
    status_publisher->publish(status);
    joint_publisher->publish(joint_state(
        static_cast<builtin_interfaces::msg::Time>(source_node->now())));
    std::this_thread::sleep_for(5ms);
    observation = adapter.latest_observation(std::chrono::steady_clock::now() + 200ms);
  } while ((!observation.observation.status.available ||
    !observation.observation.velocity_sample.available) &&
    std::chrono::steady_clock::now() < receive_deadline);

  ASSERT_TRUE(observation.observation.status.available);
  EXPECT_TRUE(observation.observation.status.fresh);
  EXPECT_TRUE(observation.observation.status.valid);
  EXPECT_EQ(
    observation.observation.safety_status,
    manipulation::servo::ServoSafetyStatus::kAdvisory);
  ASSERT_TRUE(observation.observation.velocity_sample.available);
  EXPECT_TRUE(observation.observation.velocity_sample.fresh);
  EXPECT_TRUE(observation.observation.velocity_sample.valid);
  EXPECT_NEAR(observation.observation.velocity_sample.max_abs_joint_velocity, 0.2, 1.0e-12);

  std::this_thread::sleep_for(60ms);
  observation = adapter.latest_observation(std::chrono::steady_clock::now() + 200ms);
  EXPECT_FALSE(observation.observation.status.fresh);
  EXPECT_FALSE(observation.observation.velocity_sample.fresh);
  EXPECT_EQ(
    observation.observation.safety_status,
    manipulation::servo::ServoSafetyStatus::kStopRequired);

  const auto sequence_before_invalid = observation.observation.velocity_sample.sequence;
  const auto invalid_stamp = static_cast<builtin_interfaces::msg::Time>(source_node->now());
  const auto invalid_deadline = std::chrono::steady_clock::now() + 1s;
  do {
    joint_publisher->publish(joint_state(invalid_stamp, false));
    std::this_thread::sleep_for(5ms);
    observation = adapter.latest_observation(std::chrono::steady_clock::now() + 200ms);
  } while (observation.observation.velocity_sample.sequence == sequence_before_invalid &&
    std::chrono::steady_clock::now() < invalid_deadline);
  EXPECT_TRUE(observation.observation.velocity_sample.available);
  EXPECT_TRUE(observation.observation.velocity_sample.fresh);
  EXPECT_FALSE(observation.observation.velocity_sample.valid);

  auto newer_stamp = invalid_stamp;
  newer_stamp.sec += 2;
  auto older_stamp = invalid_stamp;
  ++older_stamp.sec;
  const auto newer_sequence = observation.observation.velocity_sample.sequence;
  const auto newer_deadline = std::chrono::steady_clock::now() + 1s;
  do {
    joint_publisher->publish(joint_state(newer_stamp));
    std::this_thread::sleep_for(5ms);
    observation = adapter.latest_observation(std::chrono::steady_clock::now() + 200ms);
  } while (observation.observation.velocity_sample.sequence == newer_sequence &&
    std::chrono::steady_clock::now() < newer_deadline);
  const auto backwards_deadline = std::chrono::steady_clock::now() + 1s;
  do {
    joint_publisher->publish(joint_state(older_stamp));
    std::this_thread::sleep_for(5ms);
    observation = adapter.latest_observation(std::chrono::steady_clock::now() + 200ms);
  } while (!observation.observation.ros_time_moved_backwards &&
    std::chrono::steady_clock::now() < backwards_deadline);
  EXPECT_TRUE(observation.observation.ros_time_moved_backwards);
  EXPECT_FALSE(observation.observation.velocity_sample.fresh);
  EXPECT_EQ(
    observation.observation.safety_status,
    manipulation::servo::ServoSafetyStatus::kStopRequired);
}

TEST_F(RclcppFixture, JointFeedbackRejectsOldAndFutureRosStamps)
{
  auto config = io_config("/servo_adapter_joint_stamp_age");
  config.joint_state_max_age = 200ms;
  auto adapter_node = rclcpp::Node::make_shared("servo_adapter_joint_stamp_client");
  auto source_node = rclcpp::Node::make_shared("servo_adapter_joint_stamp_source");
  auto joint_publisher = source_node->create_publisher<sensor_msgs::msg::JointState>(
    config.endpoints.joint_state_topic, rclcpp::SystemDefaultsQoS());
  MoveItServoRosAdapter adapter{*adapter_node, config};
  ExecutorThread executor{adapter_node, source_node};
  ASSERT_TRUE(adapter.prepare_target(
      identity_goal(*adapter_node), std::chrono::steady_clock::now() + 1s).status.succeeded());

  auto old_stamp = static_cast<builtin_interfaces::msg::Time>(source_node->now());
  ASSERT_GT(old_stamp.sec, 1);
  --old_stamp.sec;
  MoveItServoObservationResult observation;
  const auto old_deadline = std::chrono::steady_clock::now() + 1s;
  do {
    joint_publisher->publish(joint_state(old_stamp));
    std::this_thread::sleep_for(5ms);
    observation = adapter.latest_observation(std::chrono::steady_clock::now() + 200ms);
  } while (!observation.observation.velocity_sample.available &&
    std::chrono::steady_clock::now() < old_deadline);
  ASSERT_TRUE(observation.observation.velocity_sample.available);
  EXPECT_TRUE(observation.observation.velocity_sample.fresh);
  EXPECT_FALSE(observation.observation.velocity_sample.valid);

  auto future_stamp = static_cast<builtin_interfaces::msg::Time>(source_node->now());
  ++future_stamp.sec;
  const auto sequence_before_future = observation.observation.velocity_sample.sequence;
  const auto future_deadline = std::chrono::steady_clock::now() + 1s;
  do {
    joint_publisher->publish(joint_state(future_stamp));
    std::this_thread::sleep_for(5ms);
    observation = adapter.latest_observation(std::chrono::steady_clock::now() + 200ms);
  } while (observation.observation.velocity_sample.sequence == sequence_before_future &&
    std::chrono::steady_clock::now() < future_deadline);
  EXPECT_GT(observation.observation.velocity_sample.sequence, sequence_before_future);
  EXPECT_TRUE(observation.observation.velocity_sample.fresh);
  EXPECT_FALSE(observation.observation.velocity_sample.valid);
}

}  // namespace
}  // namespace manipulation::moveit
