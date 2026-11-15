// Responsibility: Verifies unlimited typed-health waits, probes, cancellation, and generations.

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "manipulation_msgs/msg/system_health_snapshot.hpp"

#include "action_test_support.hpp"
#include "manipulation_moveit/ros_pick_lift_place_health_port.hpp"

namespace manipulation::moveit
{
namespace
{

using namespace std::chrono_literals;
using HealthMessage = manipulation_msgs::msg::SystemHealthSnapshot;
using WaitingReason = manipulation::workflow::TransferWaitingReason;
using test::ExecutorThread;
using RclcppFixture = test::RclcppFixture;
using test::Signal;

constexpr auto kFormerHealthReadyDeadlineEvidence = 5100ms;

class HealthTestContext final : public manipulation::workflow::TransferExecutionContext
{
public:
  manipulation::workflow::TransferExecutionControlResult checkpoint() override
  {
    return stopped_.load() ?
           manipulation::workflow::TransferExecutionControlResult::kStopped :
           manipulation::workflow::TransferExecutionControlResult::kContinue;
  }

  [[nodiscard]] bool stop_requested() const noexcept override
  {
    return stopped_.load();
  }

  manipulation::workflow::TransferExecutionControlResult wait_until(TimePoint) override
  {
    return checkpoint();
  }

  void report_feedback(manipulation::task::TaskStateSnapshot) override {}

  void report_waiting(
    const WaitingReason reason,
    manipulation::task::TaskStateSnapshot) override
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    reasons_.push_back(reason);
    waiting_reported_.notify();
  }

  void cancel() noexcept
  {
    stopped_.store(true);
  }

  [[nodiscard]] bool wait_for_report(const std::chrono::steady_clock::duration timeout)
  {
    return waiting_reported_.wait_for(timeout);
  }

  [[nodiscard]] std::vector<WaitingReason> reasons() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return reasons_;
  }

private:
  std::atomic_bool stopped_{false};
  mutable std::mutex mutex_;
  std::vector<WaitingReason> reasons_;
  Signal waiting_reported_;
};

class CancelHealthWaitOnExit final
{
public:
  explicit CancelHealthWaitOnExit(HealthTestContext & context)
  : context_(context)
  {
  }

  ~CancelHealthWaitOnExit()
  {
    context_.cancel();
  }

private:
  HealthTestContext & context_;
};

template<typename Predicate>
bool wait_until(Predicate predicate, const std::chrono::steady_clock::duration timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(1ms);
  }
  return predicate();
}

HealthMessage ready_health(const std::uint64_t sequence = 1U)
{
  HealthMessage message;
  message.sequence = sequence;
  message.motion_ready = true;
  message.overall_level = 0U;
  return message;
}

RosPickLiftPlaceReadinessProbes ready_probes()
{
  const auto ready = []() {return PickLiftPlaceReadiness{true, "ready"};};
  return {ready, ready, ready, ready};
}

TEST_F(RclcppFixture, HealthWaitOutlivesTransientProbeDelayThenBecomesReady)
{
  const std::string topic = "/test_v2_health/probe_delay";
  auto publisher_node = rclcpp::Node::make_shared("v2_health_delay_publisher");
  auto client_node = rclcpp::Node::make_shared("v2_health_delay_client");
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  auto publisher = publisher_node->create_publisher<HealthMessage>(topic, qos);
  std::atomic_bool planning_ready{false};
  Signal planning_polled;
  auto probes = ready_probes();
  probes.planning = [&planning_ready, &planning_polled]() {
      planning_polled.notify();
      return PickLiftPlaceReadiness{
      planning_ready.load(), planning_ready.load() ? "ready" : "still starting"};
    };
  RosPickLiftPlaceHealthPort port(*client_node, {topic}, std::move(probes));
  ExecutorThread executor{publisher_node, client_node};
  ASSERT_TRUE(wait_until([&publisher]() {return publisher->get_subscription_count() > 0U;}, 1s));
  publisher->publish(ready_health());
  HealthTestContext context;
  port.begin_goal();

  auto result = std::async(std::launch::async, [&port, &context]() {
        return port.wait_until_ready(context);
    });
  CancelHealthWaitOnExit cancel_on_exit{context};
  ASSERT_TRUE(planning_polled.wait_for(1s));
  EXPECT_EQ(
    result.wait_for(kFormerHealthReadyDeadlineEvidence),
    std::future_status::timeout);
  planning_ready.store(true);
  publisher->publish(ready_health(2U));
  ASSERT_EQ(result.wait_for(1s), std::future_status::ready);
  const auto ready = result.get();

  EXPECT_TRUE(ready.is_ready()) << ready.detail();
  const auto reasons = context.reasons();
  EXPECT_NE(
    std::find(reasons.begin(), reasons.end(), WaitingReason::kHealthReadiness),
    reasons.end());
}

TEST_F(RclcppFixture, HealthLongWaitCancelsAndNextGenerationCanBecomeReady)
{
  const std::string topic = "/test_v2_health/cancel_generation";
  auto publisher_node = rclcpp::Node::make_shared("v2_health_cancel_publisher");
  auto client_node = rclcpp::Node::make_shared("v2_health_cancel_client");
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  auto publisher = publisher_node->create_publisher<HealthMessage>(topic, qos);
  RosPickLiftPlaceHealthPort port(*client_node, {topic}, ready_probes());
  ExecutorThread executor{publisher_node, client_node};
  ASSERT_TRUE(wait_until([&publisher]() {return publisher->get_subscription_count() > 0U;}, 1s));
  HealthTestContext canceled_context;
  port.begin_goal();

  auto canceled_result = std::async(std::launch::async, [&port, &canceled_context]() {
        return port.wait_until_ready(canceled_context);
    });
  ASSERT_TRUE(canceled_context.wait_for_report(1s));
  EXPECT_EQ(canceled_result.wait_for(120ms), std::future_status::timeout);
  canceled_context.cancel();
  port.request_stop();
  ASSERT_EQ(canceled_result.wait_for(1s), std::future_status::ready);
  const auto canceled = canceled_result.get();
  EXPECT_EQ(canceled.outcome(), PickLiftPlaceHealthOutcome::kCanceled);

  HealthTestContext next_context;
  port.begin_goal();
  publisher->publish(ready_health(2U));
  const auto next = port.wait_until_ready(next_context);
  EXPECT_TRUE(next.is_ready()) << next.detail();
}

TEST_F(RclcppFixture, TransientLocalSnapshotIsAvailableToLateHealthSubscriber)
{
  const std::string topic = "/test_v2_health/transient_local";
  auto publisher_node = rclcpp::Node::make_shared("v2_health_transient_publisher");
  auto client_node = rclcpp::Node::make_shared("v2_health_transient_client");
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  auto publisher = publisher_node->create_publisher<HealthMessage>(topic, qos);
  publisher->publish(ready_health());
  RosPickLiftPlaceHealthPort port(*client_node, {topic}, ready_probes());
  ExecutorThread executor{publisher_node, client_node};
  ASSERT_TRUE(wait_until([&port]() {return port.cached_readiness().ready;}, 1s));

  HealthTestContext context;
  port.begin_goal();
  const auto result = port.wait_until_ready(context);
  EXPECT_TRUE(result.is_ready()) << result.detail();
}

}  // namespace
}  // namespace manipulation::moveit
