// Responsibility: Verifies ROS action arbitration and lease lifetime without a ServoNode.

#include <gtest/gtest.h>

#include <chrono>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "manipulation/recording/episode_event_sink.hpp"
#include "manipulation/skill/capability_set.hpp"
#include "manipulation/skill/preset_identity.hpp"
#include "manipulation/skill/resource_claim.hpp"
#include "manipulation/skill/resource_registry.hpp"
#include "manipulation/skill/skill_descriptor.hpp"
#include "manipulation/skill/standard_capabilities.hpp"
#include "manipulation/workflow/skill_runtime.hpp"
#include "manipulation_msgs/msg/motion_task_result.hpp"
#include "manipulation_moveit/servo/servo_to_pose_action_server.hpp"
#include "servo_to_pose_task_test_support.hpp"

namespace
{

using Action = manipulation_msgs::action::ServoToPose;
using ActionClient = rclcpp_action::Client<Action>;
using ClientGoalHandle = rclcpp_action::ClientGoalHandle<Action>;
using manipulation::moveit::ServoToPoseActionServer;
using manipulation::moveit::ServoToPoseActionServerConfig;
using manipulation::moveit::test::FakeServoToPosePort;
using manipulation::moveit::test::fast_task_config;
using manipulation::moveit::test::valid_target;
using manipulation::moveit::test::wait_until;
using manipulation::recording::EpisodeEvent;
using manipulation::recording::EpisodeEventSink;
using manipulation::recording::is_terminal;
using manipulation::skill::CapabilitySet;
using manipulation::skill::PresetIdentity;
using manipulation::skill::ResourceClaim;
using manipulation::skill::ResourceRegistry;
using manipulation::skill::SkillDescriptor;
using manipulation::task::ObjectDisposition;
using manipulation::workflow::SkillRuntime;
using namespace std::chrono_literals;

class RclcppFixture : public testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      int argc = 0;
      rclcpp::init(argc, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

class TerminalOutcomeSink final : public EpisodeEventSink
{
public:
  bool try_enqueue(const EpisodeEvent & event) noexcept override
  {
    if (!is_terminal(event.marker()) || !event.outcome()) {
      return true;
    }
    const std::lock_guard<std::mutex> lock(mutex_);
    terminal_disposition_ = event.outcome()->disposition();
    return true;
  }

  [[nodiscard]] std::optional<ObjectDisposition> terminal_disposition() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return terminal_disposition_;
  }

private:
  mutable std::mutex mutex_;
  std::optional<ObjectDisposition> terminal_disposition_;
};

std::shared_ptr<const SkillRuntime> make_runtime(
  const std::shared_ptr<ResourceRegistry> & registry,
  std::shared_ptr<EpisodeEventSink> event_sink = nullptr)
{
  const CapabilitySet capabilities{
    std::string{manipulation::skill::capability::kServoPose},
    std::string{manipulation::skill::capability::kExactStampTransform}};
  const auto descriptor = SkillDescriptor::create(
    "servo/to_pose",
    PresetIdentity::from_canonical_content(
      "test/servo_to_pose", 1U, "control_period_ms=1"),
    capabilities,
    {ResourceClaim{"arm/main"}});
  auto runtime = SkillRuntime::create(
    descriptor, capabilities, registry, std::move(event_sink));
  return std::make_shared<const SkillRuntime>(std::move(runtime));
}

ServoToPoseActionServerConfig action_config(
  std::string action_name,
  std::shared_ptr<const SkillRuntime> runtime)
{
  ServoToPoseActionServerConfig config;
  config.action_name = std::move(action_name);
  config.pause_service_name = config.action_name + "/set_task_paused";
  config.heartbeat_period = 1ms;
  config.default_timeout = 500ms;
  config.task = fast_task_config();
  config.skill_runtime = std::move(runtime);
  return config;
}

Action::Goal valid_goal()
{
  Action::Goal goal;
  goal.target_pose = valid_target();
  return goal;
}

struct RunningAction
{
  std::shared_ptr<rclcpp::Node> server_node;
  std::shared_ptr<rclcpp::Node> client_node;
  std::shared_ptr<ActionClient> client;
  rclcpp::executors::MultiThreadedExecutor executor;
  std::thread spin_thread;

  RunningAction()
  : server_node(std::make_shared<rclcpp::Node>("servo_action_test_server")),
    client_node(std::make_shared<rclcpp::Node>("servo_action_test_client")),
    executor(rclcpp::ExecutorOptions{}, 2U)
  {
    executor.add_node(server_node);
    executor.add_node(client_node);
    spin_thread = std::thread([this]() {executor.spin();});
  }

  ~RunningAction()
  {
    executor.cancel();
    if (spin_thread.joinable()) {
      spin_thread.join();
    }
    executor.remove_node(client_node);
    executor.remove_node(server_node);
  }
};

TEST_F(RclcppFixture, RejectsGoalUntilServoBackendIsReadyWithoutClaimingArm)
{
  const auto registry = std::make_shared<ResourceRegistry>();
  RunningAction running;
  FakeServoToPosePort port;
  port.backend_ready_flag = false;
  const std::string action_name{"/test_servo_to_pose/backend_startup"};
  ServoToPoseActionServer server(
    *running.server_node, port, action_config(action_name, make_runtime(registry)));
  running.client = rclcpp_action::create_client<Action>(running.client_node, action_name);
  ASSERT_TRUE(running.client->wait_for_action_server(1s));

  auto handle_future = running.client->async_send_goal(valid_goal());
  ASSERT_EQ(handle_future.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(handle_future.get(), nullptr);
  EXPECT_TRUE(registry->snapshot().active_ownerships.empty());
  EXPECT_EQ(port.pause_calls, 0U);
}

TEST_F(RclcppFixture, RejectsGoalTimeoutThatConsumesTheSafeHaltReserve)
{
  const auto registry = std::make_shared<ResourceRegistry>();
  RunningAction running;
  FakeServoToPosePort port;
  const std::string action_name{"/test_servo_to_pose/short_timeout"};
  ServoToPoseActionServer server(
    *running.server_node, port, action_config(action_name, make_runtime(registry)));
  running.client = rclcpp_action::create_client<Action>(running.client_node, action_name);
  ASSERT_TRUE(running.client->wait_for_action_server(1s));

  auto goal = valid_goal();
  goal.timeout.nanosec = 10'000'000U;
  auto handle_future = running.client->async_send_goal(goal);
  ASSERT_EQ(handle_future.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(handle_future.get(), nullptr);
  EXPECT_EQ(port.pause_calls, 0U);
  EXPECT_TRUE(registry->snapshot().active_ownerships.empty());
}

TEST_F(RclcppFixture, RejectsDefaultTimeoutThatConsumesTheSafeHaltReserve)
{
  const auto registry = std::make_shared<ResourceRegistry>();
  RunningAction running;
  FakeServoToPosePort port;
  auto config = action_config(
    "/test_servo_to_pose/invalid_default_timeout", make_runtime(registry));
  config.default_timeout =
    config.task.state_machine.halt_timeout + config.task.control_period;

  EXPECT_THROW(
    ServoToPoseActionServer(*running.server_node, port, std::move(config)),
    std::invalid_argument);
  EXPECT_EQ(port.pause_calls, 0U);
}

TEST_F(RclcppFixture, ResourceConflictIsAcceptedThenReturnsTypedAbort)
{
  const auto registry = std::make_shared<ResourceRegistry>();
  auto external = registry->try_acquire(
    "test/external_owner", {ResourceClaim{"arm/main"}});
  ASSERT_TRUE(external.acquired());

  RunningAction running;
  FakeServoToPosePort port;
  const std::string action_name{"/test_servo_to_pose/resource_conflict"};
  ServoToPoseActionServer server(
    *running.server_node, port, action_config(action_name, make_runtime(registry)));
  running.client = rclcpp_action::create_client<Action>(running.client_node, action_name);
  ASSERT_TRUE(running.client->wait_for_action_server(1s));

  auto handle_future = running.client->async_send_goal(valid_goal());
  ASSERT_EQ(handle_future.wait_for(1s), std::future_status::ready);
  const auto handle = handle_future.get();
  ASSERT_NE(handle, nullptr);
  auto result_future = running.client->async_get_result(handle);
  ASSERT_EQ(result_future.wait_for(1s), std::future_status::ready);
  const auto result = result_future.get();

  ASSERT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
  ASSERT_NE(result.result, nullptr);
  EXPECT_EQ(
    result.result->result.error_code,
    manipulation_msgs::msg::MotionTaskResult::RESOURCE_UNAVAILABLE);
  EXPECT_EQ(port.pause_calls, 0U);
  EXPECT_EQ(port.unpause_calls, 0U);
  ASSERT_EQ(registry->snapshot().active_ownerships.size(), 1U);
}

TEST_F(RclcppFixture, CancelKeepsLeaseUntilLowVelocityStopAndTerminalResult)
{
  const auto registry = std::make_shared<ResourceRegistry>();
  RunningAction running;
  FakeServoToPosePort port(FakeServoToPosePort::Mode::kNeverReach);
  port.allow_low_velocity = false;
  const std::string action_name{"/test_servo_to_pose/cancel_lease"};
  ServoToPoseActionServer server(
    *running.server_node, port, action_config(action_name, make_runtime(registry)));
  running.client = rclcpp_action::create_client<Action>(running.client_node, action_name);
  ASSERT_TRUE(running.client->wait_for_action_server(1s));

  auto handle_future = running.client->async_send_goal(valid_goal());
  ASSERT_EQ(handle_future.wait_for(1s), std::future_status::ready);
  const auto handle = handle_future.get();
  ASSERT_NE(handle, nullptr);
  ASSERT_TRUE(wait_until([&]() {return port.publish_calls.load() >= 2U;}));
  ASSERT_EQ(registry->snapshot().active_ownerships.size(), 1U);

  auto cancel_future = running.client->async_cancel_goal(handle);
  ASSERT_EQ(cancel_future.wait_for(1s), std::future_status::ready);
  ASSERT_EQ(cancel_future.get()->goals_canceling.size(), 1U);
  auto result_future = running.client->async_get_result(handle);
  EXPECT_EQ(result_future.wait_for(5ms), std::future_status::timeout);
  EXPECT_EQ(registry->snapshot().active_ownerships.size(), 1U);

  port.allow_low_velocity = true;
  ASSERT_EQ(result_future.wait_for(1s), std::future_status::ready);
  const auto result = result_future.get();
  ASSERT_EQ(result.code, rclcpp_action::ResultCode::CANCELED);
  ASSERT_NE(result.result, nullptr);
  EXPECT_TRUE(result.result->result.stop_confirmed);
  EXPECT_TRUE(wait_until([&]() {return registry->snapshot().active_ownerships.empty();}));
}

TEST_F(RclcppFixture, PortExceptionAfterTrackingUsesSafeHaltBeforeReleasingLease)
{
  const auto registry = std::make_shared<ResourceRegistry>();
  RunningAction running;
  FakeServoToPosePort port(FakeServoToPosePort::Mode::kThrowOnceAfterPublish);
  port.allow_low_velocity = false;
  const std::string action_name{"/test_servo_to_pose/port_exception"};
  auto config = action_config(action_name, make_runtime(registry));
  config.task.state_machine.halt_timeout = 100ms;
  ServoToPoseActionServer server(*running.server_node, port, std::move(config));
  running.client = rclcpp_action::create_client<Action>(running.client_node, action_name);
  ASSERT_TRUE(running.client->wait_for_action_server(1s));

  auto handle_future = running.client->async_send_goal(valid_goal());
  ASSERT_EQ(handle_future.wait_for(1s), std::future_status::ready);
  const auto handle = handle_future.get();
  ASSERT_NE(handle, nullptr);
  auto result_future = running.client->async_get_result(handle);
  ASSERT_TRUE(wait_until([&]() {return port.throw_count.load() == 1U;}));
  EXPECT_EQ(result_future.wait_for(5ms), std::future_status::timeout);
  EXPECT_EQ(registry->snapshot().active_ownerships.size(), 1U);
  const auto publications_at_failure = port.publish_calls.load();

  port.allow_low_velocity = true;
  ASSERT_EQ(result_future.wait_for(1s), std::future_status::ready);
  const auto result = result_future.get();
  ASSERT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
  ASSERT_NE(result.result, nullptr);
  EXPECT_EQ(
    result.result->result.error_code,
    manipulation_msgs::msg::MotionTaskResult::INTERNAL_ERROR);
  EXPECT_TRUE(result.result->result.stop_confirmed);
  EXPECT_EQ(port.publish_calls.load(), publications_at_failure);
  EXPECT_GE(port.pause_calls.load(), 2U);
  EXPECT_TRUE(wait_until([&]() {return registry->snapshot().active_ownerships.empty();}));
}

TEST_F(RclcppFixture, CancelAfterWorkerCompletionIsRejectedAndSuccessIsPreserved)
{
  const auto registry = std::make_shared<ResourceRegistry>();
  RunningAction running;
  FakeServoToPosePort port;
  const std::string action_name{"/test_servo_to_pose/cancel_after_completion"};
  auto config = action_config(action_name, make_runtime(registry));
  config.heartbeat_period = 250ms;
  ServoToPoseActionServer server(*running.server_node, port, std::move(config));
  running.client = rclcpp_action::create_client<Action>(running.client_node, action_name);
  ASSERT_TRUE(running.client->wait_for_action_server(1s));

  auto handle_future = running.client->async_send_goal(valid_goal());
  ASSERT_EQ(handle_future.wait_for(1s), std::future_status::ready);
  const auto handle = handle_future.get();
  ASSERT_NE(handle, nullptr);
  ASSERT_TRUE(wait_until([&]() {return server.snapshot().result_pending;}, 200ms));

  auto cancel_future = running.client->async_cancel_goal(handle);
  ASSERT_EQ(cancel_future.wait_for(1s), std::future_status::ready);
  EXPECT_TRUE(cancel_future.get()->goals_canceling.empty());

  auto result_future = running.client->async_get_result(handle);
  ASSERT_EQ(result_future.wait_for(1s), std::future_status::ready);
  const auto result = result_future.get();
  ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_NE(result.result, nullptr);
  EXPECT_TRUE(result.result->result.success);
  EXPECT_EQ(
    result.result->result.error_code,
    manipulation_msgs::msg::MotionTaskResult::NONE);
}

TEST_F(RclcppFixture, HaltTimeoutQuarantinesArmLeaseAndRecordsUnknownDisposition)
{
  const auto registry = std::make_shared<ResourceRegistry>();
  const auto event_sink = std::make_shared<TerminalOutcomeSink>();
  RunningAction running;
  FakeServoToPosePort port(FakeServoToPosePort::Mode::kNeverReach);
  port.allow_low_velocity = false;
  const std::string action_name{"/test_servo_to_pose/halt_timeout"};
  auto config = action_config(action_name, make_runtime(registry, event_sink));
  config.task.state_machine.halt_timeout = 10ms;
  ServoToPoseActionServer server(*running.server_node, port, std::move(config));
  running.client = rclcpp_action::create_client<Action>(running.client_node, action_name);
  ASSERT_TRUE(running.client->wait_for_action_server(1s));

  auto handle_future = running.client->async_send_goal(valid_goal());
  ASSERT_EQ(handle_future.wait_for(1s), std::future_status::ready);
  const auto handle = handle_future.get();
  ASSERT_NE(handle, nullptr);
  ASSERT_TRUE(wait_until([&]() {return port.publish_calls.load() >= 2U;}));
  auto cancel_future = running.client->async_cancel_goal(handle);
  ASSERT_EQ(cancel_future.wait_for(1s), std::future_status::ready);
  ASSERT_EQ(cancel_future.get()->goals_canceling.size(), 1U);
  ASSERT_EQ(registry->snapshot().active_ownerships.size(), 1U);

  auto result_future = running.client->async_get_result(handle);
  ASSERT_EQ(result_future.wait_for(1s), std::future_status::ready);
  const auto result = result_future.get();
  ASSERT_EQ(result.code, rclcpp_action::ResultCode::CANCELED);
  ASSERT_NE(result.result, nullptr);
  EXPECT_EQ(
    result.result->result.error_code,
    manipulation_msgs::msg::MotionTaskResult::RECOVERY_FAILED);
  EXPECT_FALSE(result.result->result.stop_confirmed);

  ASSERT_TRUE(wait_until([&]() {
      const auto state = server.snapshot();
      return !state.active && state.resource_quarantined;
  }));
  const auto state = server.snapshot();
  EXPECT_TRUE(state.resource_held);
  EXPECT_TRUE(state.resource_quarantined);
  ASSERT_EQ(registry->snapshot().active_ownerships.size(), 1U);

  const auto publications_at_quarantine = port.publish_calls.load();
  auto second_handle_future = running.client->async_send_goal(valid_goal());
  ASSERT_EQ(second_handle_future.wait_for(1s), std::future_status::ready);
  const auto second_handle = second_handle_future.get();
  ASSERT_NE(second_handle, nullptr);
  auto second_result_future = running.client->async_get_result(second_handle);
  ASSERT_EQ(second_result_future.wait_for(1s), std::future_status::ready);
  const auto second_result = second_result_future.get();
  ASSERT_EQ(second_result.code, rclcpp_action::ResultCode::ABORTED);
  ASSERT_NE(second_result.result, nullptr);
  EXPECT_EQ(
    second_result.result->result.error_code,
    manipulation_msgs::msg::MotionTaskResult::RESOURCE_UNAVAILABLE);
  EXPECT_EQ(port.publish_calls.load(), publications_at_quarantine);
  EXPECT_TRUE(server.snapshot().resource_quarantined);

  auto transfer_claim = registry->try_acquire(
    "test/transfer_owner", {ResourceClaim{"arm/main"}});
  EXPECT_FALSE(transfer_claim.acquired());
  EXPECT_EQ(
    transfer_claim.error,
    manipulation::skill::ResourceAcquireError::kConflict);
  EXPECT_EQ(transfer_claim.conflicting_resource_id, "arm/main");

  ASSERT_TRUE(wait_until([&]() {return event_sink->terminal_disposition().has_value();}));
  EXPECT_EQ(event_sink->terminal_disposition(), ObjectDisposition::kUnknown);
}

}  // namespace
