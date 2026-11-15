// Responsibility: Proves quiescent dispatch, terminal convergence, budgets, and exact cancel.

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "control_msgs/action/gripper_command.hpp"
#include "moveit_msgs/action/execute_trajectory.hpp"
#include "moveit_msgs/msg/move_it_error_codes.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include "action_test_support.hpp"
#include "manipulation_moveit/ros_v2_physical_ports.hpp"

namespace manipulation::moveit
{
namespace
{

using namespace std::chrono_literals;
using ExecuteAction = moveit_msgs::action::ExecuteTrajectory;
using ExecuteHandle = rclcpp_action::ServerGoalHandle<ExecuteAction>;
using GripperAction = control_msgs::action::GripperCommand;
using GripperHandle = rclcpp_action::ServerGoalHandle<GripperAction>;
using test::ExecutorThread;
using RclcppFixture = test::RclcppFixture;
using test::Signal;

class PhysicalTestContext final : public manipulation::workflow::TransferExecutionContext
{
public:
  manipulation::workflow::TransferExecutionControlResult checkpoint() override
  {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [this]() {return !paused_ || stopped_.load();});
    return control_result();
  }

  [[nodiscard]] bool stop_requested() const noexcept override
  {
    return stopped_.load();
  }

  manipulation::workflow::TransferExecutionControlResult wait_until(TimePoint) override
  {
    return checkpoint();
  }

  manipulation::workflow::TransferExecutionControlResult heartbeat() override
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    ++heartbeat_count_;
    cv_.notify_all();
    return control_result();
  }

  manipulation::workflow::TaskOperationGateResult run_operation(
    const std::function<void()> & operation) override
  {
    std::unique_lock<std::mutex> lock(mutex_);
    ++gate_count_;
    gate_entered_ = true;
    cv_.notify_all();
    if (block_dispatch_) {
      cv_.wait(lock, [this]() {return !block_dispatch_ || stopped_.load();});
    }
    if (stopped_.load()) {
      return manipulation::workflow::TaskOperationGateResult::kStopped;
    }
    if (pause_next_dispatch_) {
      pause_next_dispatch_ = false;
      paused_ = true;
      cv_.notify_all();
      return manipulation::workflow::TaskOperationGateResult::kPaused;
    }
    if (paused_) {
      return manipulation::workflow::TaskOperationGateResult::kPaused;
    }
    operation();
    return manipulation::workflow::TaskOperationGateResult::kExecuted;
  }

  void report_feedback(manipulation::task::TaskStateSnapshot) override {}

  void block_dispatch()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    block_dispatch_ = true;
  }

  void pause_next_dispatch()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    pause_next_dispatch_ = true;
  }

  void request_pause()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    paused_ = true;
    cv_.notify_all();
  }

  void resume()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    paused_ = false;
    block_dispatch_ = false;
    cv_.notify_all();
  }

  void cancel()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    stopped_.store(true);
    block_dispatch_ = false;
    cv_.notify_all();
  }

  [[nodiscard]] bool wait_for_gate(
    const std::chrono::steady_clock::duration timeout)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return cv_.wait_for(lock, timeout, [this]() {return gate_entered_;});
  }

  [[nodiscard]] bool wait_for_heartbeats(
    const std::size_t count,
    const std::chrono::steady_clock::duration timeout)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return cv_.wait_for(
      lock, timeout, [this, count]() {return heartbeat_count_ >= count;});
  }

  [[nodiscard]] std::size_t heartbeat_count() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return heartbeat_count_;
  }

private:
  [[nodiscard]] manipulation::workflow::TransferExecutionControlResult
  control_result() const noexcept
  {
    return stopped_.load() ?
           manipulation::workflow::TransferExecutionControlResult::kStopped :
           manipulation::workflow::TransferExecutionControlResult::kContinue;
  }

  std::atomic_bool stopped_{false};
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::size_t heartbeat_count_{0U};
  std::size_t gate_count_{0U};
  bool gate_entered_{false};
  bool block_dispatch_{false};
  bool pause_next_dispatch_{false};
  bool paused_{false};
};

class JointStateStream final
{
public:
  JointStateStream(rclcpp::Node & node, std::string topic)
  : clock_(node.get_clock()),
    publisher_(node.create_publisher<sensor_msgs::msg::JointState>(
        std::move(topic), rclcpp::SensorDataQoS()))
  {
    timer_ = node.create_wall_timer(2ms, [this]() {publish_if_enabled();});
  }

  void set_sample(
    const double position,
    const std::optional<double> velocity = 0.0)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    names_ = {"joint_1"};
    positions_ = {position};
    velocities_ = velocity ?
      std::optional<std::vector<double>>{std::vector<double>{*velocity}} :
    std::nullopt;
    position_step_per_publish_ = 0.0;
  }

  void set_sample(
    std::vector<std::string> names,
    std::vector<double> positions,
    std::optional<std::vector<double>> velocities)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    names_ = std::move(names);
    positions_ = std::move(positions);
    velocities_ = std::move(velocities);
    position_step_per_publish_ = 0.0;
  }

  void set_drifting_sample(
    const double position,
    const double velocity,
    const double position_step_per_publish)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    names_ = {"joint_1"};
    positions_ = {position};
    velocities_ = std::vector<double>{velocity};
    position_step_per_publish_ = position_step_per_publish;
  }

  void pause()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    enabled_ = false;
  }

  void publish_once()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    publisher_->publish(make_message_locked());
  }

  void publish_once_at(const std::int64_t stamp_nanoseconds)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    auto message = make_message_locked();
    message.header.stamp = rclcpp::Time(stamp_nanoseconds, clock_->get_clock_type());
    publisher_->publish(message);
  }

private:
  sensor_msgs::msg::JointState make_message_locked()
  {
    sensor_msgs::msg::JointState message;
    const auto now = clock_->now().nanoseconds();
    last_stamp_nanoseconds_ = std::max(last_stamp_nanoseconds_ + 1, now);
    message.header.stamp =
      rclcpp::Time(last_stamp_nanoseconds_, clock_->get_clock_type());
    message.name = names_;
    message.position = positions_;
    if (positions_.size() == 1U) {
      positions_.front() += position_step_per_publish_;
    }
    if (velocities_) {
      message.velocity = *velocities_;
    }
    return message;
  }

  void publish_if_enabled()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (!enabled_) {
      return;
    }
    publisher_->publish(make_message_locked());
  }

  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex mutex_;
  std::vector<std::string> names_{"joint_1"};
  std::vector<double> positions_{0.0};
  std::optional<std::vector<double>> velocities_{
    std::vector<double>{0.0}};
  double position_step_per_publish_{0.0};
  std::int64_t last_stamp_nanoseconds_{0};
  bool enabled_{true};
};

TrajectoryExecutionRequest trajectory_request(const std::string & label)
{
  TrajectoryExecutionRequest request;
  request.label = label;
  request.trajectory.joint_trajectory.joint_names = {"joint_1"};
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = {0.0};
  request.trajectory.joint_trajectory.points.push_back(std::move(point));
  return request;
}

std::shared_ptr<ExecuteAction::Result> execute_result(const std::int32_t code)
{
  auto result = std::make_shared<ExecuteAction::Result>();
  result->error_code.val = code;
  return result;
}

bool wait_for_convergence(
  const RosTrajectoryExecutionPort & port,
  const std::chrono::steady_clock::duration timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (port.cached_readiness().detail.find("convergence") != std::string::npos) {
      return true;
    }
    std::this_thread::sleep_for(1ms);
  }
  return false;
}

TEST_F(RclcppFixture, CancelAtTrajectoryDispatchGateSendsNoGoal)
{
  const std::string action_name = "/test_v2_physical/cancel_at_dispatch";
  const std::string joint_state_topic =
    "/test_v2_physical/cancel_at_dispatch/joint_states";
  auto server_node = rclcpp::Node::make_shared("v2_cancel_dispatch_server");
  auto client_node = rclcpp::Node::make_shared("v2_cancel_dispatch_client");
  std::atomic_size_t received_goals{0U};
  auto server = rclcpp_action::create_server<ExecuteAction>(
    server_node, action_name,
    [&received_goals](
      const rclcpp_action::GoalUUID &, std::shared_ptr<const ExecuteAction::Goal>) {
      received_goals.fetch_add(1U);
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [](const std::shared_ptr<ExecuteHandle>) {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [](const std::shared_ptr<ExecuteHandle> handle) {
      handle->succeed(execute_result(moveit_msgs::msg::MoveItErrorCodes::SUCCESS));
    });
  JointStateStream joint_states{*server_node, joint_state_topic};
  joint_states.set_sample(0.0, std::nullopt);
  ExecutorThread executor{server_node, client_node};
  RosTrajectoryExecutionPort port(*client_node, {action_name, joint_state_topic});
  PhysicalTestContext context;
  context.block_dispatch();
  port.begin_goal();

  auto future = std::async(std::launch::async, [&port, &context]() {
        return port.execute(trajectory_request("cancel_at_dispatch"), 5s, context);
    });
  ASSERT_TRUE(context.wait_for_gate(1s));
  context.cancel();
  ASSERT_EQ(future.wait_for(1s), std::future_status::ready);
  const auto result = future.get();

  EXPECT_EQ(result.outcome(), PhysicalCommandOutcome::kCanceled);
  EXPECT_EQ(result.safety(), PhysicalCommandSafety::kNotAccepted);
  EXPECT_EQ(received_goals.load(), 0U);
  (void)server;
}

TEST_F(RclcppFixture, PauseAtTrajectoryDispatchGateDefersGoalUntilResume)
{
  const std::string action_name = "/test_v2_physical/pause_at_dispatch";
  const std::string joint_state_topic =
    "/test_v2_physical/pause_at_dispatch/joint_states";
  auto server_node = rclcpp::Node::make_shared("v2_pause_dispatch_server");
  auto client_node = rclcpp::Node::make_shared("v2_pause_dispatch_client");
  std::atomic_size_t received_goals{0U};
  auto server = rclcpp_action::create_server<ExecuteAction>(
    server_node, action_name,
    [&received_goals](
      const rclcpp_action::GoalUUID &, std::shared_ptr<const ExecuteAction::Goal>) {
      received_goals.fetch_add(1U);
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [](const std::shared_ptr<ExecuteHandle>) {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [](const std::shared_ptr<ExecuteHandle> handle) {
      handle->succeed(execute_result(moveit_msgs::msg::MoveItErrorCodes::SUCCESS));
    });
  JointStateStream joint_states{*server_node, joint_state_topic};
  ExecutorThread executor{server_node, client_node};
  RosTrajectoryExecutionPort port(*client_node, {action_name, joint_state_topic});
  PhysicalTestContext context;
  context.pause_next_dispatch();
  port.begin_goal();

  auto future = std::async(std::launch::async, [&port, &context]() {
        return port.execute(trajectory_request("pause_at_dispatch"), 5s, context);
    });
  ASSERT_TRUE(context.wait_for_gate(1s));
  EXPECT_EQ(received_goals.load(), 0U);
  EXPECT_EQ(future.wait_for(30ms), std::future_status::timeout);

  context.resume();
  ASSERT_EQ(future.wait_for(1s), std::future_status::ready);
  EXPECT_TRUE(future.get().succeeded());
  EXPECT_EQ(received_goals.load(), 1U);
  (void)server;
}

TEST_F(RclcppFixture, RequestStopInterruptsPreDispatchQuiescenceWithoutSendingGoal)
{
  const std::string action_name = "/test_v2_physical/cancel_pre_quiescence";
  const std::string joint_state_topic =
    "/test_v2_physical/cancel_pre_quiescence/joint_states";
  auto server_node = rclcpp::Node::make_shared("v2_cancel_pre_quiescence_server");
  auto client_node = rclcpp::Node::make_shared("v2_cancel_pre_quiescence_client");
  std::atomic_size_t received_goals{0U};
  auto server = rclcpp_action::create_server<ExecuteAction>(
    server_node, action_name,
    [&received_goals](
      const rclcpp_action::GoalUUID &, std::shared_ptr<const ExecuteAction::Goal>) {
      received_goals.fetch_add(1U);
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [](const std::shared_ptr<ExecuteHandle>) {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [](const std::shared_ptr<ExecuteHandle> handle) {
      handle->succeed(execute_result(moveit_msgs::msg::MoveItErrorCodes::SUCCESS));
    });
  JointStateStream joint_states{*server_node, joint_state_topic};
  joint_states.set_sample(0.0, 0.2);
  ExecutorThread executor{server_node, client_node};
  RosTrajectoryExecutionPort port(*client_node, {action_name, joint_state_topic});
  PhysicalTestContext context;
  port.begin_goal();

  auto future = std::async(std::launch::async, [&port, &context]() {
        return port.execute(trajectory_request("cancel_pre_quiescence"), 5s, context);
    });
  if (!context.wait_for_heartbeats(3U, 1s)) {
    port.request_stop();
    (void)future.get();
    FAIL() << "pre-dispatch quiescence wait was not entered";
  }
  ASSERT_EQ(future.wait_for(20ms), std::future_status::timeout);
  const auto waiting_readiness = port.cached_readiness();
  EXPECT_FALSE(waiting_readiness.ready);
  EXPECT_NE(waiting_readiness.detail.find("convergence"), std::string::npos);

  auto stop_future = std::async(std::launch::async, [&port]() {port.request_stop();});
  ASSERT_EQ(stop_future.wait_for(1s), std::future_status::ready);
  stop_future.get();
  ASSERT_EQ(future.wait_for(1s), std::future_status::ready);
  const auto result = future.get();

  EXPECT_EQ(result.outcome(), PhysicalCommandOutcome::kCanceled);
  EXPECT_EQ(result.safety(), PhysicalCommandSafety::kNotAccepted);
  EXPECT_EQ(received_goals.load(), 0U);
  (void)server;
}

TEST_F(RclcppFixture, AcceptedTrajectoryBeyondLeaseTtlKeepsHeartbeatAndIgnoresPause)
{
  const std::string action_name = "/test_v2_physical/long_accepted_heartbeat";
  const std::string joint_state_topic =
    "/test_v2_physical/long_accepted_heartbeat/joint_states";
  auto server_node = rclcpp::Node::make_shared("v2_long_accepted_server");
  auto client_node = rclcpp::Node::make_shared("v2_long_accepted_client");
  std::promise<std::shared_ptr<ExecuteHandle>> accepted;
  auto accepted_future = accepted.get_future();
  auto server = rclcpp_action::create_server<ExecuteAction>(
    server_node, action_name,
    [](const rclcpp_action::GoalUUID &, std::shared_ptr<const ExecuteAction::Goal>) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [](const std::shared_ptr<ExecuteHandle>) {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [&accepted](const std::shared_ptr<ExecuteHandle> handle) {
      accepted.set_value(handle);
    });
  JointStateStream joint_states{*server_node, joint_state_topic};
  ExecutorThread executor{server_node, client_node};
  RosTrajectoryExecutionPort port(*client_node, {action_name, joint_state_topic});
  PhysicalTestContext context;
  port.begin_goal();

  auto future = std::async(std::launch::async, [&port, &context]() {
        return port.execute(
          trajectory_request("long_accepted"), std::chrono::seconds{50}, context);
    });
  ASSERT_EQ(accepted_future.wait_for(1s), std::future_status::ready);
  const auto handle = accepted_future.get();
  context.request_pause();
  ASSERT_TRUE(context.wait_for_heartbeats(5U, 1s));
  EXPECT_EQ(future.wait_for(20ms), std::future_status::timeout);

  handle->succeed(execute_result(moveit_msgs::msg::MoveItErrorCodes::SUCCESS));
  ASSERT_EQ(future.wait_for(1s), std::future_status::ready);
  EXPECT_TRUE(future.get().succeeded());
  EXPECT_GE(context.heartbeat_count(), 5U);
  (void)server;
}

TEST_F(RclcppFixture, TrajectoryAcceptanceDelayDoesNotConsumePhysicalBudget)
{
  const std::string action_name = "/test_v2_physical/delayed_accept";
  const std::string joint_state_topic =
    "/test_v2_physical/delayed_accept/joint_states";
  auto server_node = rclcpp::Node::make_shared("v2_delayed_accept_server");
  auto client_node = rclcpp::Node::make_shared("v2_delayed_accept_client");
  Signal goal_entered;
  Signal allow_accept;
  auto server = rclcpp_action::create_server<ExecuteAction>(
    server_node, action_name,
    [&goal_entered, &allow_accept](
      const rclcpp_action::GoalUUID &, std::shared_ptr<const ExecuteAction::Goal>) {
      goal_entered.notify();
      allow_accept.wait();
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [](const std::shared_ptr<ExecuteHandle>) {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [](const std::shared_ptr<ExecuteHandle> handle) {
      handle->succeed(execute_result(moveit_msgs::msg::MoveItErrorCodes::SUCCESS));
    });
  JointStateStream joint_states{*server_node, joint_state_topic};
  ExecutorThread executor{server_node, client_node};
  RosTrajectoryExecutionPort port(*client_node, {action_name, joint_state_topic});
  PhysicalTestContext context;
  port.begin_goal();

  auto future = std::async(std::launch::async, [&port, &context]() {
        return port.execute(trajectory_request("delayed"), 40ms, context);
    });
  if (!goal_entered.wait_for(1s)) {
    allow_accept.notify();
    FAIL() << "ExecuteTrajectory goal callback was not entered";
  }
  EXPECT_EQ(future.wait_for(80ms), std::future_status::timeout);
  allow_accept.notify();
  const auto result = future.get();

  EXPECT_TRUE(result.succeeded()) << result.detail();
  EXPECT_EQ(result.safety(), PhysicalCommandSafety::kTerminalSafe);
  (void)server;
}

TEST_F(RclcppFixture, TrajectoryTimeoutWithoutTerminalEvidenceQuarantinesGoal)
{
  const std::string action_name = "/test_v2_physical/timeout_unknown";
  const std::string joint_state_topic =
    "/test_v2_physical/timeout_unknown/joint_states";
  auto server_node = rclcpp::Node::make_shared("v2_timeout_unknown_server");
  auto client_node = rclcpp::Node::make_shared("v2_timeout_unknown_client");
  std::promise<std::shared_ptr<ExecuteHandle>> accepted;
  auto accepted_future = accepted.get_future();
  Signal cancel_seen;
  auto server = rclcpp_action::create_server<ExecuteAction>(
    server_node, action_name,
    [](const rclcpp_action::GoalUUID &, std::shared_ptr<const ExecuteAction::Goal>) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [&cancel_seen](const std::shared_ptr<ExecuteHandle>) {
      cancel_seen.notify();
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [&accepted](const std::shared_ptr<ExecuteHandle> handle) {
      accepted.set_value(handle);
    });
  JointStateStream joint_states{*server_node, joint_state_topic};
  ExecutorThread executor{server_node, client_node};
  RosTrajectoryExecutionPort port(*client_node, {action_name, joint_state_topic});
  PhysicalTestContext context;
  port.begin_goal();

  const auto result = port.execute(trajectory_request("timeout_unknown"), 30ms, context);

  EXPECT_EQ(result.outcome(), PhysicalCommandOutcome::kTimedOut);
  EXPECT_EQ(result.safety(), PhysicalCommandSafety::kActiveStateUnknown);
  EXPECT_FALSE(port.cached_readiness().ready);
  EXPECT_TRUE(cancel_seen.wait_for(1s));
  ASSERT_EQ(accepted_future.wait_for(1s), std::future_status::ready);
  const auto handle = accepted_future.get();
  handle->canceled(execute_result(moveit_msgs::msg::MoveItErrorCodes::PREEMPTED));
  (void)server;
}

TEST_F(RclcppFixture, TrajectoryTimeoutWithTerminalCancelEvidenceIsSafe)
{
  const std::string action_name = "/test_v2_physical/timeout_safe";
  const std::string joint_state_topic =
    "/test_v2_physical/timeout_safe/joint_states";
  auto server_node = rclcpp::Node::make_shared("v2_timeout_safe_server");
  auto client_node = rclcpp::Node::make_shared("v2_timeout_safe_client");
  auto cancel_timer = std::make_shared<rclcpp::TimerBase::SharedPtr>();
  const std::weak_ptr<rclcpp::TimerBase::SharedPtr> weak_timer = cancel_timer;
  auto retained_handle = std::make_shared<std::shared_ptr<ExecuteHandle>>();
  auto server = rclcpp_action::create_server<ExecuteAction>(
    server_node, action_name,
    [](const rclcpp_action::GoalUUID &, std::shared_ptr<const ExecuteAction::Goal>) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [server_node, cancel_timer, weak_timer](const std::shared_ptr<ExecuteHandle> handle) {
      *cancel_timer = server_node->create_wall_timer(
        1ms, [handle, weak_timer]() {
          if (const auto timer = weak_timer.lock()) {
            (*timer)->cancel();
          }
          handle->canceled(execute_result(moveit_msgs::msg::MoveItErrorCodes::PREEMPTED));
        });
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [retained_handle](const std::shared_ptr<ExecuteHandle> handle) {
      *retained_handle = handle;
    });
  JointStateStream joint_states{*server_node, joint_state_topic};
  ExecutorThread executor{server_node, client_node};
  RosTrajectoryExecutionPort port(*client_node, {action_name, joint_state_topic});
  PhysicalTestContext context;
  port.begin_goal();

  const auto result = port.execute(trajectory_request("timeout_safe"), 30ms, context);

  EXPECT_EQ(result.outcome(), PhysicalCommandOutcome::kTimedOut);
  EXPECT_EQ(result.safety(), PhysicalCommandSafety::kTerminalSafe);
  (void)server;
}

TEST_F(RclcppFixture, SuccessfulTrajectoryWaitsForTwoFreshQuiescentTerminalSamples)
{
  const std::string action_name = "/test_v2_physical/terminal_convergence";
  const std::string joint_state_topic =
    "/test_v2_physical/terminal_convergence/joint_states";
  auto server_node = rclcpp::Node::make_shared("v2_terminal_convergence_server");
  auto client_node = rclcpp::Node::make_shared("v2_terminal_convergence_client");
  JointStateStream joint_states{*server_node, joint_state_topic};
  Signal result_sent;
  auto server = rclcpp_action::create_server<ExecuteAction>(
    server_node, action_name,
    [](const rclcpp_action::GoalUUID &, std::shared_ptr<const ExecuteAction::Goal>) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [](const std::shared_ptr<ExecuteHandle>) {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [&joint_states, &result_sent](const std::shared_ptr<ExecuteHandle> handle) {
      joint_states.set_sample(0.0, 0.2);
      handle->succeed(execute_result(moveit_msgs::msg::MoveItErrorCodes::SUCCESS));
      result_sent.notify();
    });
  ExecutorThread executor{server_node, client_node};
  RosTrajectoryExecutionPort port(*client_node, {action_name, joint_state_topic});
  PhysicalTestContext context;
  port.begin_goal();

  auto future = std::async(std::launch::async, [&port, &context]() {
        return port.execute(trajectory_request("terminal_convergence"), 5s, context);
    });
  if (!result_sent.wait_for(1s)) {
    port.request_stop();
    (void)future.get();
    FAIL() << "ExecuteTrajectory result was not sent";
  }
  ASSERT_EQ(future.wait_for(40ms), std::future_status::timeout);
  const auto waiting_readiness = port.cached_readiness();
  EXPECT_FALSE(waiting_readiness.ready);
  EXPECT_NE(waiting_readiness.detail.find("convergence"), std::string::npos);

  joint_states.pause();
  ASSERT_EQ(future.wait_for(20ms), std::future_status::timeout);
  joint_states.set_sample(0.0, 0.0);
  joint_states.publish_once();
  EXPECT_EQ(future.wait_for(30ms), std::future_status::timeout);

  joint_states.publish_once();
  if (future.wait_for(1s) != std::future_status::ready) {
    joint_states.publish_once();
    port.request_stop();
    (void)future.get();
    FAIL() << "two fresh quiescent samples did not release terminal convergence";
  }
  const auto result = future.get();
  EXPECT_TRUE(result.succeeded()) << result.detail();
  EXPECT_EQ(result.safety(), PhysicalCommandSafety::kTerminalSafe);
  (void)server;
}

TEST_F(RclcppFixture, InvalidTerminalTimestampsQuarantinePhysicalReadiness)
{
  for (const std::string invalid_kind : {"zero", "future", "clock_rollback"}) {
    SCOPED_TRACE(invalid_kind);
    const std::string action_name = "/test_v2_physical/invalid_stamp_" + invalid_kind;
    const std::string joint_state_topic = action_name + "/joint_states";
    auto server_node = rclcpp::Node::make_shared("v2_stamp_server_" + invalid_kind);
    auto client_node = rclcpp::Node::make_shared("v2_stamp_client_" + invalid_kind);
    JointStateStream joint_states{*server_node, joint_state_topic};
    Signal result_sent;
    auto server = rclcpp_action::create_server<ExecuteAction>(
      server_node, action_name,
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const ExecuteAction::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<ExecuteHandle>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [&joint_states, &result_sent](const std::shared_ptr<ExecuteHandle> handle) {
        joint_states.pause();
        handle->succeed(execute_result(moveit_msgs::msg::MoveItErrorCodes::SUCCESS));
        result_sent.notify();
      });
    ExecutorThread executor{server_node, client_node};
    RosTrajectoryExecutionPort port(*client_node, {action_name, joint_state_topic});
    PhysicalTestContext context;
    port.begin_goal();

    auto future = std::async(std::launch::async, [&port, &context]() {
          return port.execute(trajectory_request("invalid_terminal_stamp"), 300ms, context);
        });
    if (!result_sent.wait_for(1s)) {
      port.request_stop();
      (void)future.get();
      FAIL() << "ExecuteTrajectory result was not sent";
    }
    EXPECT_EQ(future.wait_for(20ms), std::future_status::timeout);
    const auto now_ns = client_node->now().nanoseconds();
    if (invalid_kind == "clock_rollback") {
      const auto clock = client_node->get_clock();
      const std::lock_guard<std::mutex> clock_lock(clock->get_clock_mutex());
      ASSERT_EQ(rcl_enable_ros_time_override(clock->get_clock_handle()), RCL_RET_OK);
      ASSERT_EQ(
        rcl_set_ros_time_override(clock->get_clock_handle(), now_ns - 1000000000LL),
        RCL_RET_OK);
    } else {
      const auto stamp = invalid_kind == "zero" ? 0LL : now_ns + 30000000000LL;
      joint_states.publish_once_at(stamp);
      joint_states.publish_once_at(stamp + (invalid_kind == "zero" ? 0LL : 1LL));
    }

    ASSERT_EQ(future.wait_for(1s), std::future_status::ready);
    const auto result = future.get();
    EXPECT_EQ(result.outcome(), PhysicalCommandOutcome::kInternalError) << result.detail();
    EXPECT_EQ(result.safety(), PhysicalCommandSafety::kActiveStateUnknown);
    EXPECT_FALSE(port.cached_readiness().ready);
    (void)server;
  }
}

TEST_F(RclcppFixture, ReplayedAndStaleTerminalStampsDoNotProveQuiescence)
{
  const std::string action_name = "/test_v2_physical/replayed_terminal_stamp";
  const std::string joint_state_topic = action_name + "/joint_states";
  auto server_node = rclcpp::Node::make_shared("v2_replayed_stamp_server");
  auto client_node = rclcpp::Node::make_shared("v2_replayed_stamp_client");
  JointStateStream joint_states{*server_node, joint_state_topic};
  Signal result_sent;
  auto server = rclcpp_action::create_server<ExecuteAction>(
    server_node, action_name,
    [](const rclcpp_action::GoalUUID &, std::shared_ptr<const ExecuteAction::Goal>) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [](const std::shared_ptr<ExecuteHandle>) {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [&joint_states, &result_sent](const std::shared_ptr<ExecuteHandle> handle) {
      joint_states.pause();
      handle->succeed(execute_result(moveit_msgs::msg::MoveItErrorCodes::SUCCESS));
      result_sent.notify();
    });
  ExecutorThread executor{server_node, client_node};
  RosTrajectoryExecutionPort port(*client_node, {action_name, joint_state_topic});
  PhysicalTestContext context;
  port.begin_goal();
  auto future = std::async(std::launch::async, [&port, &context]() {
        return port.execute(trajectory_request("replayed_terminal_stamp"), 2s, context);
      });
  if (!result_sent.wait_for(1s)) {
    port.request_stop();
    (void)future.get();
    FAIL() << "ExecuteTrajectory result was not sent";
  }
  EXPECT_EQ(future.wait_for(20ms), std::future_status::timeout);
  const auto stamp = server_node->now().nanoseconds();
  joint_states.publish_once_at(stamp);
  EXPECT_EQ(future.wait_for(20ms), std::future_status::timeout);
  joint_states.publish_once_at(stamp);
  EXPECT_EQ(future.wait_for(20ms), std::future_status::timeout);
  joint_states.publish_once_at(stamp - 2);
  joint_states.publish_once_at(stamp - 1);
  joint_states.publish_once_at(stamp);
  EXPECT_EQ(future.wait_for(20ms), std::future_status::timeout);
  EXPECT_EQ(future.wait_for(270ms), std::future_status::timeout);
  joint_states.publish_once_at(stamp + 1);
  joint_states.publish_once_at(stamp + 2);
  EXPECT_EQ(future.wait_for(20ms), std::future_status::timeout);
  joint_states.publish_once();
  joint_states.publish_once();
  ASSERT_EQ(future.wait_for(1s), std::future_status::ready);
  EXPECT_TRUE(future.get().succeeded());
  (void)server;
}

TEST_F(RclcppFixture, StableTerminalPositionsOverrideNoisyReportedVelocity)
{
  const std::string action_name = "/test_v2_physical/noisy_velocity_stable_position";
  const std::string joint_state_topic =
    "/test_v2_physical/noisy_velocity_stable_position/joint_states";
  auto server_node = rclcpp::Node::make_shared("v2_noisy_velocity_stable_server");
  auto client_node = rclcpp::Node::make_shared("v2_noisy_velocity_stable_client");
  JointStateStream joint_states{*server_node, joint_state_topic};
  Signal result_sent;
  auto server = rclcpp_action::create_server<ExecuteAction>(
    server_node, action_name,
    [](const rclcpp_action::GoalUUID &, std::shared_ptr<const ExecuteAction::Goal>) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [](const std::shared_ptr<ExecuteHandle>) {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [&joint_states, &result_sent](const std::shared_ptr<ExecuteHandle> handle) {
      joint_states.set_sample(0.0, 0.2);
      handle->succeed(execute_result(moveit_msgs::msg::MoveItErrorCodes::SUCCESS));
      result_sent.notify();
    });
  ExecutorThread executor{server_node, client_node};
  RosTrajectoryExecutionPort port(*client_node, {action_name, joint_state_topic});
  PhysicalTestContext context;
  port.begin_goal();

  auto future = std::async(std::launch::async, [&port, &context]() {
        return port.execute(trajectory_request("noisy_velocity_stable"), 2s, context);
    });
  if (!result_sent.wait_for(1s)) {
    port.request_stop();
    (void)future.get();
    FAIL() << "ExecuteTrajectory result was not sent";
  }

  EXPECT_EQ(future.wait_for(70ms), std::future_status::timeout);
  ASSERT_EQ(future.wait_for(1s), std::future_status::ready);
  const auto result = future.get();
  EXPECT_TRUE(result.succeeded()) << result.detail();
  EXPECT_EQ(result.safety(), PhysicalCommandSafety::kTerminalSafe);
  (void)server;
}

TEST_F(RclcppFixture, NoisyVelocityPositionEvidenceRejectsInsufficientTimeSpan)
{
  const std::string action_name = "/test_v2_physical/terminal_convergence_timeout";
  const std::string joint_state_topic =
    "/test_v2_physical/terminal_convergence_timeout/joint_states";
  auto server_node = rclcpp::Node::make_shared("v2_terminal_timeout_server");
  auto client_node = rclcpp::Node::make_shared("v2_terminal_timeout_client");
  JointStateStream joint_states{*server_node, joint_state_topic};
  auto server = rclcpp_action::create_server<ExecuteAction>(
    server_node, action_name,
    [](const rclcpp_action::GoalUUID &, std::shared_ptr<const ExecuteAction::Goal>) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [](const std::shared_ptr<ExecuteHandle>) {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [&joint_states](const std::shared_ptr<ExecuteHandle> handle) {
      joint_states.set_sample(0.0, 0.2);
      handle->succeed(execute_result(moveit_msgs::msg::MoveItErrorCodes::SUCCESS));
    });
  ExecutorThread executor{server_node, client_node};
  RosTrajectoryExecutionPort port(*client_node, {action_name, joint_state_topic});
  PhysicalTestContext context;
  port.begin_goal();

  const auto started = std::chrono::steady_clock::now();
  const auto result = port.execute(
    trajectory_request("terminal_convergence_timeout"), 60ms, context);
  const auto elapsed = std::chrono::steady_clock::now() - started;

  EXPECT_EQ(result.outcome(), PhysicalCommandOutcome::kTimedOut);
  EXPECT_EQ(result.safety(), PhysicalCommandSafety::kActiveStateUnknown);
  EXPECT_GE(elapsed, 40ms);
  EXPECT_LT(elapsed, 1s);
  EXPECT_NE(result.detail().find("last_observation=at_target"), std::string::npos);
  EXPECT_NE(result.detail().find("max_abs_velocity=0.200000"), std::string::npos);
  EXPECT_NE(result.detail().find("position_window_span_ms="), std::string::npos);
  EXPECT_NE(result.detail().find("position_excursion=0.000000"), std::string::npos);
  EXPECT_NE(result.detail().find("for 100 ms with at least 5 samples"), std::string::npos);
  const auto readiness = port.cached_readiness();
  EXPECT_FALSE(readiness.ready);
  EXPECT_NE(readiness.detail.find("unknown"), std::string::npos);
  (void)server;
}

TEST_F(RclcppFixture, TerminalTimeoutDiagnosticReportsOrderedJointEvidence)
{
  const std::string action_name = "/test_v2_physical/ordered_joint_diagnostic";
  const std::string joint_state_topic =
    "/test_v2_physical/ordered_joint_diagnostic/joint_states";
  auto server_node = rclcpp::Node::make_shared("v2_ordered_joint_diagnostic_server");
  auto client_node = rclcpp::Node::make_shared("v2_ordered_joint_diagnostic_client");
  JointStateStream joint_states{*server_node, joint_state_topic};
  joint_states.set_sample(
    {"joint_c", "joint_a", "joint_b"},
    {0.004, -0.008, 0.008},
    std::vector<double>{0.75, -0.75, 0.25});
  auto server = rclcpp_action::create_server<ExecuteAction>(
    server_node, action_name,
    [](const rclcpp_action::GoalUUID &, std::shared_ptr<const ExecuteAction::Goal>) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [](const std::shared_ptr<ExecuteHandle>) {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [](const std::shared_ptr<ExecuteHandle> handle) {
      handle->succeed(execute_result(moveit_msgs::msg::MoveItErrorCodes::SUCCESS));
    });
  ExecutorThread executor{server_node, client_node};
  RosTrajectoryExecutionPort port(*client_node, {action_name, joint_state_topic});
  PhysicalTestContext context;
  port.begin_goal();

  TrajectoryExecutionRequest request;
  request.label = "ordered_joint_diagnostic";
  request.trajectory.joint_trajectory.joint_names =
  {"joint_b", "joint_a", "joint_c"};
  trajectory_msgs::msg::JointTrajectoryPoint terminal;
  terminal.positions = {0.0, 0.0, 0.0};
  terminal.velocities = {0.1, -0.6, 0.6};
  request.trajectory.joint_trajectory.points.push_back(std::move(terminal));

  const auto result = port.execute(request, 60ms, context);

  EXPECT_EQ(result.outcome(), PhysicalCommandOutcome::kTimedOut);
  EXPECT_EQ(result.safety(), PhysicalCommandSafety::kActiveStateUnknown);
  EXPECT_NE(
    result.detail().find(
      "max_target_error=0.008000 max_target_error_joint=joint_b "
      "max_target_actual=0.008000 max_target=0.000000"),
    std::string::npos);
  EXPECT_NE(
    result.detail().find(
      "max_abs_velocity=0.750000 max_abs_velocity_joint=joint_a "
      "max_abs_velocity_signed=-0.750000"),
    std::string::npos);
  EXPECT_NE(
    result.detail().find(
      "max_terminal_waypoint_abs_velocity=0.600000 "
      "max_terminal_waypoint_velocity_joint=joint_a "
      "max_terminal_waypoint_velocity_signed=-0.600000"),
    std::string::npos);
  (void)server;
}

TEST_F(RclcppFixture, MalformedTerminalWaypointVelocitiesAreRejectedBeforeDispatch)
{
  auto client_node = rclcpp::Node::make_shared("v2_malformed_terminal_velocity_client");
  RosTrajectoryExecutionPort port(
    *client_node,
    {"/test_v2_physical/malformed_terminal_velocity",
      "/test_v2_physical/malformed_terminal_velocity/joint_states"});
  PhysicalTestContext context;
  port.begin_goal();
  auto request = trajectory_request("malformed_terminal_velocity");
  request.trajectory.joint_trajectory.points.back().velocities = {0.0, 0.0};

  const auto result = port.execute(request, 1s, context);

  EXPECT_EQ(result.outcome(), PhysicalCommandOutcome::kRejected);
  EXPECT_EQ(result.safety(), PhysicalCommandSafety::kNotAccepted);
  EXPECT_NE(
    result.detail().find(
      "terminal waypoint velocities must cover every named joint"),
    std::string::npos);
}

TEST_F(RclcppFixture, NoisyVelocityPositionWindowRequiresFiveFreshSamples)
{
  const std::string action_name = "/test_v2_physical/position_sample_count";
  const std::string joint_state_topic =
    "/test_v2_physical/position_sample_count/joint_states";
  auto server_node = rclcpp::Node::make_shared("v2_position_sample_count_server");
  auto client_node = rclcpp::Node::make_shared("v2_position_sample_count_client");
  JointStateStream joint_states{*server_node, joint_state_topic};
  Signal result_sent;
  auto server = rclcpp_action::create_server<ExecuteAction>(
    server_node, action_name,
    [](const rclcpp_action::GoalUUID &, std::shared_ptr<const ExecuteAction::Goal>) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [](const std::shared_ptr<ExecuteHandle>) {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [&joint_states, &result_sent](const std::shared_ptr<ExecuteHandle> handle) {
      joint_states.pause();
      joint_states.set_sample(0.0, 0.2);
      handle->succeed(execute_result(moveit_msgs::msg::MoveItErrorCodes::SUCCESS));
      result_sent.notify();
    });
  ExecutorThread executor{server_node, client_node};
  RosTrajectoryExecutionPort port(*client_node, {action_name, joint_state_topic});
  PhysicalTestContext context;
  port.begin_goal();

  auto future = std::async(std::launch::async, [&port, &context]() {
        return port.execute(trajectory_request("position_sample_count"), 2s, context);
    });
  if (!result_sent.wait_for(1s) || !wait_for_convergence(port, 1s)) {
    port.request_stop();
    (void)future.get();
    FAIL() << "terminal convergence wait was not entered";
  }

  joint_states.publish_once();
  std::this_thread::sleep_for(40ms);
  joint_states.publish_once();
  std::this_thread::sleep_for(40ms);
  joint_states.publish_once();
  std::this_thread::sleep_for(40ms);
  joint_states.publish_once();
  EXPECT_EQ(future.wait_for(30ms), std::future_status::timeout);

  joint_states.publish_once();
  ASSERT_EQ(future.wait_for(1s), std::future_status::ready);
  const auto result = future.get();
  EXPECT_TRUE(result.succeeded()) << result.detail();
  EXPECT_EQ(result.safety(), PhysicalCommandSafety::kTerminalSafe);
  (void)server;
}

TEST_F(RclcppFixture, NoisyVelocityPositionWindowRejectsCumulativeDrift)
{
  const std::string action_name = "/test_v2_physical/noisy_velocity_position_drift";
  const std::string joint_state_topic =
    "/test_v2_physical/noisy_velocity_position_drift/joint_states";
  auto server_node = rclcpp::Node::make_shared("v2_position_drift_server");
  auto client_node = rclcpp::Node::make_shared("v2_position_drift_client");
  JointStateStream joint_states{*server_node, joint_state_topic};
  Signal result_sent;
  auto server = rclcpp_action::create_server<ExecuteAction>(
    server_node, action_name,
    [](const rclcpp_action::GoalUUID &, std::shared_ptr<const ExecuteAction::Goal>) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [](const std::shared_ptr<ExecuteHandle>) {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [&joint_states, &result_sent](const std::shared_ptr<ExecuteHandle> handle) {
      joint_states.set_drifting_sample(0.0, 0.2, 0.00005);
      handle->succeed(execute_result(moveit_msgs::msg::MoveItErrorCodes::SUCCESS));
      result_sent.notify();
    });
  ExecutorThread executor{server_node, client_node};
  RosTrajectoryExecutionPort port(*client_node, {action_name, joint_state_topic});
  PhysicalTestContext context;
  port.begin_goal();

  auto future = std::async(std::launch::async, [&port, &context]() {
        return port.execute(trajectory_request("noisy_velocity_drift"), 240ms, context);
    });
  if (!result_sent.wait_for(1s)) {
    port.request_stop();
    (void)future.get();
    FAIL() << "ExecuteTrajectory result was not sent";
  }

  ASSERT_EQ(future.wait_for(1s), std::future_status::ready);
  const auto result = future.get();
  EXPECT_EQ(result.outcome(), PhysicalCommandOutcome::kTimedOut);
  EXPECT_EQ(result.safety(), PhysicalCommandSafety::kActiveStateUnknown);
  EXPECT_NE(result.detail().find("last_observation=at_target"), std::string::npos);
  EXPECT_NE(result.detail().find("position_window_resets="), std::string::npos);
  EXPECT_EQ(result.detail().find("position_window_resets=0 "), std::string::npos);
  (void)server;
}

TEST_F(RclcppFixture, RequestStopInterruptsTerminalConvergenceWait)
{
  const std::string action_name = "/test_v2_physical/cancel_terminal_convergence";
  const std::string joint_state_topic =
    "/test_v2_physical/cancel_terminal_convergence/joint_states";
  auto server_node = rclcpp::Node::make_shared("v2_cancel_terminal_convergence_server");
  auto client_node = rclcpp::Node::make_shared("v2_cancel_terminal_convergence_client");
  JointStateStream joint_states{*server_node, joint_state_topic};
  Signal result_sent;
  auto server = rclcpp_action::create_server<ExecuteAction>(
    server_node, action_name,
    [](const rclcpp_action::GoalUUID &, std::shared_ptr<const ExecuteAction::Goal>) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [](const std::shared_ptr<ExecuteHandle>) {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [&joint_states, &result_sent](const std::shared_ptr<ExecuteHandle> handle) {
      joint_states.set_sample(0.0, 0.2);
      handle->succeed(execute_result(moveit_msgs::msg::MoveItErrorCodes::SUCCESS));
      result_sent.notify();
    });
  ExecutorThread executor{server_node, client_node};
  RosTrajectoryExecutionPort port(*client_node, {action_name, joint_state_topic});
  PhysicalTestContext context;
  port.begin_goal();

  auto future = std::async(std::launch::async, [&port, &context]() {
        return port.execute(
          trajectory_request("cancel_terminal_convergence"), 5s, context);
    });
  if (!result_sent.wait_for(1s)) {
    port.request_stop();
    (void)future.get();
    FAIL() << "ExecuteTrajectory result was not sent";
  }
  ASSERT_EQ(future.wait_for(40ms), std::future_status::timeout);
  const auto waiting_readiness = port.cached_readiness();
  EXPECT_FALSE(waiting_readiness.ready);
  EXPECT_NE(waiting_readiness.detail.find("convergence"), std::string::npos);

  auto stop_future = std::async(std::launch::async, [&port]() {port.request_stop();});
  ASSERT_EQ(stop_future.wait_for(1s), std::future_status::ready);
  stop_future.get();
  ASSERT_EQ(future.wait_for(1s), std::future_status::ready);
  const auto result = future.get();
  EXPECT_EQ(result.outcome(), PhysicalCommandOutcome::kCanceled);
  EXPECT_EQ(result.safety(), PhysicalCommandSafety::kActiveStateUnknown);
  (void)server;
}

TEST_F(RclcppFixture, GripperMapsReachedGoalAndAllowedCloseStall)
{
  const std::string action_name = "/test_v2_physical/gripper_evidence";
  auto server_node = rclcpp::Node::make_shared("v2_gripper_evidence_server");
  auto client_node = rclcpp::Node::make_shared("v2_gripper_evidence_client");
  auto server = rclcpp_action::create_server<GripperAction>(
    server_node, action_name,
    [](const rclcpp_action::GoalUUID &, std::shared_ptr<const GripperAction::Goal>) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [](const std::shared_ptr<GripperHandle>) {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [](const std::shared_ptr<GripperHandle> handle) {
      auto result = std::make_shared<GripperAction::Result>();
      result->position = handle->get_goal()->command.position;
      result->effort = 5.0;
      if (result->position < 0.02) {
        result->stalled = true;
      } else {
        result->reached_goal = true;
      }
      handle->succeed(result);
    });
  ExecutorThread executor{server_node, client_node};
  RosStrategyGripperPort port(*client_node, {action_name});
  PhysicalTestContext context;
  port.begin_goal();

  const auto opened = port.command(
    {GripperCommandKind::kOpen, 0.08, 10.0, false, "open"}, 100ms, context);
  ASSERT_TRUE(opened.command().succeeded()) << opened.command().detail();
  EXPECT_EQ(opened.postcondition(), GripperPostcondition::kOpenConfirmed);

  const auto grasped = port.command(
    {GripperCommandKind::kClose, 0.01, 20.0, true, "close"}, 100ms, context);
  ASSERT_TRUE(grasped.command().succeeded()) << grasped.command().detail();
  EXPECT_EQ(grasped.postcondition(), GripperPostcondition::kClosedConfirmed);

  const auto disallowed = port.command(
    {GripperCommandKind::kClose, 0.01, 20.0, false, "close_no_stall"},
    100ms, context);
  EXPECT_EQ(disallowed.command().outcome(), PhysicalCommandOutcome::kFailed);
  EXPECT_EQ(disallowed.command().safety(), PhysicalCommandSafety::kTerminalSafe);
  EXPECT_EQ(disallowed.postcondition(), GripperPostcondition::kUnknown);
  (void)server;
}

TEST_F(
  RclcppFixture,
  GripperAcceptanceDelayDoesNotConsumeBudgetAndTimedOutGoalIsTerminalSafe)
{
  const std::string action_name = "/test_v2_physical/gripper_delayed_accept_timeout";
  auto server_node = rclcpp::Node::make_shared("v2_gripper_delayed_accept_server");
  auto client_node = rclcpp::Node::make_shared("v2_gripper_delayed_accept_client");
  Signal goal_entered;
  Signal allow_accept;
  auto cancel_timer = std::make_shared<rclcpp::TimerBase::SharedPtr>();
  const std::weak_ptr<rclcpp::TimerBase::SharedPtr> weak_timer = cancel_timer;
  auto retained_handle = std::make_shared<std::shared_ptr<GripperHandle>>();
  auto server = rclcpp_action::create_server<GripperAction>(
    server_node, action_name,
    [&goal_entered, &allow_accept](
      const rclcpp_action::GoalUUID &, std::shared_ptr<const GripperAction::Goal>) {
      goal_entered.notify();
      allow_accept.wait();
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [server_node, cancel_timer, weak_timer](const std::shared_ptr<GripperHandle> handle) {
      *cancel_timer = server_node->create_wall_timer(
        1ms, [handle, weak_timer]() {
          if (const auto timer = weak_timer.lock()) {
            (*timer)->cancel();
          }
          handle->canceled(std::make_shared<GripperAction::Result>());
        });
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [retained_handle](const std::shared_ptr<GripperHandle> handle) {
      *retained_handle = handle;
    });
  ExecutorThread executor{server_node, client_node};
  RosStrategyGripperPort port(*client_node, {action_name});
  PhysicalTestContext context;
  port.begin_goal();

  auto future = std::async(std::launch::async, [&port, &context]() {
        return port.command(
          {GripperCommandKind::kClose, 0.01, 20.0, true, "delayed_timeout"},
          80ms, context);
    });
  if (!goal_entered.wait_for(1s)) {
    allow_accept.notify();
    port.request_stop();
    (void)future.get();
    FAIL() << "GripperCommand goal callback was not entered";
  }

  EXPECT_EQ(future.wait_for(120ms), std::future_status::timeout);
  allow_accept.notify();
  EXPECT_EQ(future.wait_for(30ms), std::future_status::timeout);
  ASSERT_EQ(future.wait_for(1s), std::future_status::ready);
  const auto result = future.get();

  EXPECT_EQ(result.command().outcome(), PhysicalCommandOutcome::kTimedOut);
  EXPECT_EQ(result.command().safety(), PhysicalCommandSafety::kTerminalSafe);
  EXPECT_EQ(result.postcondition(), GripperPostcondition::kUnknown);
  EXPECT_TRUE(port.cached_readiness().ready);
  (void)server;
}

TEST_F(RclcppFixture, GripperCancelIsExactAndNextGenerationCanRun)
{
  const std::string action_name = "/test_v2_physical/gripper_generation";
  auto server_node = rclcpp::Node::make_shared("v2_gripper_generation_server");
  auto client_node = rclcpp::Node::make_shared("v2_gripper_generation_client");
  std::promise<std::shared_ptr<GripperHandle>> first_accepted;
  auto first_accepted_future = first_accepted.get_future();
  std::promise<double> canceled_position;
  auto canceled_position_future = canceled_position.get_future();
  auto cancel_timer = std::make_shared<rclcpp::TimerBase::SharedPtr>();
  const std::weak_ptr<rclcpp::TimerBase::SharedPtr> weak_timer = cancel_timer;
  auto server = rclcpp_action::create_server<GripperAction>(
    server_node, action_name,
    [](const rclcpp_action::GoalUUID &, std::shared_ptr<const GripperAction::Goal>) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [server_node, &canceled_position, cancel_timer, weak_timer](
      const std::shared_ptr<GripperHandle> handle) {
      canceled_position.set_value(handle->get_goal()->command.position);
      *cancel_timer = server_node->create_wall_timer(
        1ms, [handle, weak_timer]() {
          if (const auto timer = weak_timer.lock()) {
            (*timer)->cancel();
          }
          handle->canceled(std::make_shared<GripperAction::Result>());
        });
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [&first_accepted](const std::shared_ptr<GripperHandle> handle) {
      if (handle->get_goal()->command.position < 0.02) {
        first_accepted.set_value(handle);
        return;
      }
      auto result = std::make_shared<GripperAction::Result>();
      result->position = handle->get_goal()->command.position;
      result->effort = 5.0;
      result->reached_goal = true;
      handle->succeed(result);
    });
  ExecutorThread executor{server_node, client_node};
  RosStrategyGripperPort port(*client_node, {action_name});
  PhysicalTestContext context;
  port.begin_goal();

  auto first_result = std::async(std::launch::async, [&port, &context]() {
        return port.command(
          {GripperCommandKind::kClose, 0.01, 20.0, true, "first"}, 5s, context);
    });
  ASSERT_EQ(first_accepted_future.wait_for(1s), std::future_status::ready);
  const auto first_handle = first_accepted_future.get();
  ASSERT_TRUE(first_handle);
  port.request_stop();
  const auto canceled = first_result.get();
  EXPECT_EQ(canceled.command().outcome(), PhysicalCommandOutcome::kCanceled);
  EXPECT_EQ(canceled.command().safety(), PhysicalCommandSafety::kTerminalSafe);
  ASSERT_EQ(canceled_position_future.wait_for(1s), std::future_status::ready);
  EXPECT_DOUBLE_EQ(canceled_position_future.get(), 0.01);

  port.begin_goal();
  const auto next = port.command(
    {GripperCommandKind::kOpen, 0.08, 10.0, false, "next"}, 100ms, context);
  EXPECT_TRUE(next.command().succeeded()) << next.command().detail();
  EXPECT_EQ(next.postcondition(), GripperPostcondition::kOpenConfirmed);
  (void)server;
}

}  // namespace
}  // namespace manipulation::moveit
