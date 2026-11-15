// Responsibility: Shares an isolated ROS fixture and neutral transport probes for core tests.

#ifndef WORKFLOW__TRANSFER_SEQUENCE_ACTION_CORE_TEST_SUPPORT_HPP_
#define WORKFLOW__TRANSFER_SEQUENCE_ACTION_CORE_TEST_SUPPORT_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

#include "detail/transfer_sequence_action_lifecycle_core.hpp"
#include "detail/transfer_sequence_goal_transport.hpp"
#include "manipulation/task/task_status.hpp"
#include "manipulation/workflow/transfer_sequence_goal_reservation.hpp"
#include "rclcpp/rclcpp.hpp"

namespace manipulation::workflow::detail::test
{

using ObjectDisposition = manipulation::task::ObjectDisposition;
using TaskOutcome = manipulation::task::TaskOutcome;

inline SequenceGoalId goal_id(const std::uint8_t seed)
{
  SequenceGoalId result{};
  for (std::size_t index = 0U; index < result.size(); ++index) {
    result[index] = static_cast<std::uint8_t>(seed + index);
  }
  return result;
}

inline TransferSequenceGoalDescriptor descriptor(const std::size_t step_count = 1U)
{
  TransferSequenceGoalDescriptor result;
  result.step_count = step_count;
  return result;
}

inline TransferSequenceAdmission accepted_admission()
{
  return {true, ObjectDisposition::kObjectAtSource, {}};
}

inline TransferSequenceExecution successful_execution(const std::size_t step_count = 1U)
{
  const auto outcome = TaskOutcome::make_success(
    ObjectDisposition::kObjectAtTarget, "test sequence completed");
  return {
    TransferSequenceResult{
      outcome, step_count, std::nullopt,
      std::vector<TaskOutcome>(step_count, outcome)},
    std::vector<std::chrono::steady_clock::duration>(
      step_count, std::chrono::milliseconds{1})};
}

struct TransportProbe
{
  SequenceGoalTransportState state{SequenceGoalTransportState::kExecuting};
  std::size_t state_failure_count{0U};
  std::size_t feedback_count{0U};
  std::size_t delivery_attempt_count{0U};
  std::size_t delivery_success_count{0U};
  bool throw_state_once{false};
  bool throw_feedback_once{false};
  bool throw_delivery_once{false};
  SequenceTerminalKind delivered_kind{SequenceTerminalKind::kAborted};
  std::optional<TransferSequenceExecution> delivered_execution;
};

inline SequenceGoalTransportPort transport_port(
  const SequenceGoalId & id,
  const std::shared_ptr<TransportProbe> & probe)
{
  return SequenceGoalTransportPort(
    id,
    [probe]() {
      if (std::exchange(probe->throw_state_once, false)) {
        ++probe->state_failure_count;
        throw 11;
      }
      return probe->state;
    },
    [probe](const TransferSequenceFeedbackSnapshot &) {
      if (std::exchange(probe->throw_feedback_once, false)) {
        throw 17;
      }
      ++probe->feedback_count;
    },
    [probe](
      const SequenceTerminalKind kind,
      const TransferSequenceExecution & execution,
      std::chrono::steady_clock::duration) {
      ++probe->delivery_attempt_count;
      if (std::exchange(probe->throw_delivery_once, false)) {
        throw 23;
      }
      ++probe->delivery_success_count;
      probe->delivered_kind = kind;
      probe->delivered_execution = execution;
    });
}

class TransferSequenceActionCoreTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    context_ = std::make_shared<rclcpp::Context>();
    context_->init(0, nullptr);
    rclcpp::NodeOptions options;
    options.context(context_);
    node_ = std::make_shared<rclcpp::Node>("sequence_core_test", options);
    rclcpp::ExecutorOptions executor_options;
    executor_options.context = context_;
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(
      executor_options);
    executor_->add_node(node_);
  }

  void TearDown() override
  {
    if (node_in_executor_) {
      executor_->remove_node(node_);
    }
    executor_.reset();
    node_.reset();
    context_->shutdown("TransferSequenceActionLifecycleCore test complete");
    context_.reset();
  }

  std::unique_ptr<TransferSequenceActionLifecycleCore> make_core(
    const std::string & suffix,
    TransferSequenceLifecycleConfig config = {},
    TransferSequenceRequestStopCallback request_stop = {},
    std::shared_ptr<TransferSequenceGoalReservation> goal_reservation = {})
  {
    config.heartbeat_period = std::chrono::milliseconds{1};
    if (!goal_reservation) {
      goal_reservation = std::make_shared<TransferSequenceGoalReservation>();
    }
    auto callback_group = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    return std::make_unique<TransferSequenceActionLifecycleCore>(
      *node_, config, "TestSequence", "/test/sequence_core/" + suffix + "/pause",
      std::move(request_stop), std::move(callback_group), std::move(goal_reservation));
  }

  bool spin_until(
    const std::function<bool()> & predicate,
    const std::chrono::steady_clock::duration timeout = std::chrono::seconds{1})
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!predicate() && std::chrono::steady_clock::now() < deadline) {
      executor_->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds{1});
    }
    return predicate();
  }

  void remove_node_from_executor()
  {
    executor_->remove_node(node_);
    node_in_executor_ = false;
  }

  std::shared_ptr<rclcpp::Context> context_;
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  bool node_in_executor_{true};
};

}  // namespace manipulation::workflow::detail::test

#endif  // WORKFLOW__TRANSFER_SEQUENCE_ACTION_CORE_TEST_SUPPORT_HPP_
