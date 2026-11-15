// Responsibility: Verifies production strategy selection and test-only composition injection.

#include <chrono>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

#include "manipulation/workflow/strategy_registry.hpp"
#include "transfer_domain_test_support.hpp"

namespace manipulation::workflow
{
namespace
{

class RecordingExecutionContext final : public TransferExecutionContext
{
public:
  TransferExecutionControlResult checkpoint() override
  {
    ++checkpoint_count;
    return stopped ? TransferExecutionControlResult::kStopped :
           TransferExecutionControlResult::kContinue;
  }

  bool stop_requested() const noexcept override
  {
    return stopped;
  }

  TransferExecutionControlResult wait_until(const TimePoint deadline) override
  {
    last_wait_deadline = deadline;
    ++wait_count;
    return stopped ? TransferExecutionControlResult::kStopped :
           TransferExecutionControlResult::kContinue;
  }

  void report_feedback(manipulation::task::TaskStateSnapshot feedback) override
  {
    feedback_phases.emplace_back(feedback.phase);
  }

  bool stopped{false};
  int checkpoint_count{0};
  int wait_count{0};
  TimePoint last_wait_deadline{};
  std::vector<manipulation::task::TaskPhase> feedback_phases;
};

class RecordingStrategy final : public TransferExecutionStrategy
{
public:
  explicit RecordingStrategy(std::string strategy_id)
  : strategy_id_(std::move(strategy_id))
  {
  }

  std::string_view id() const noexcept override
  {
    return strategy_id_;
  }

  manipulation::task::TaskOutcome execute(
    const ResolvedTransferTask & task,
    TransferExecutionContext & context) override
  {
    manipulation::task::TaskStateSnapshot planning;
    planning.phase = manipulation::task::TaskPhase::kPlanning;
    context.report_feedback(std::move(planning));
    if (context.checkpoint() == TransferExecutionControlResult::kStopped) {
      return manipulation::task::TaskOutcome::make_failure(
        manipulation::task::TaskErrorCode::kCanceled,
        manipulation::task::ObjectDisposition::kUnknown);
    }
    manipulation::task::TaskStateSnapshot placing;
    placing.phase = manipulation::task::TaskPhase::kPlacing;
    context.report_feedback(std::move(placing));
    last_object_id = task.instance().object_id();
    return manipulation::task::TaskOutcome::make_success(
      manipulation::task::ObjectDisposition::kObjectAtTarget);
  }

  void request_stop() noexcept override
  {
    stop_forwarded = true;
  }

  std::string last_object_id;
  bool stop_forwarded{false};

private:
  std::string strategy_id_;
};

TEST(StrategyRegistry, ProductionRegistryAcceptsOnlyPickLiftPlace)
{
  auto production = StrategyRegistry::production(
    std::make_unique<RecordingStrategy>(std::string{kPickLiftPlaceStrategyId}));

  EXPECT_EQ(production.size(), 1U);
  EXPECT_EQ(production.require(kPickLiftPlaceStrategyId).id(), kPickLiftPlaceStrategyId);
  EXPECT_THROW(
    static_cast<void>(production.require("unknown")), std::invalid_argument);
  EXPECT_THROW(
    StrategyRegistry::production(std::make_unique<RecordingStrategy>("other")),
    std::invalid_argument);
}

TEST(StrategyRegistry, TestStrategyChangesCompositionWithoutRosOwnership)
{
  std::vector<std::unique_ptr<TransferExecutionStrategy>> strategies;
  strategies.emplace_back(std::make_unique<RecordingStrategy>("test_two_stage"));
  auto registry = StrategyRegistry::for_testing(std::move(strategies));
  RecordingExecutionContext context;

  auto & strategy = registry.require("test_two_stage");
  const auto outcome = strategy.execute(test_support::resolved_task(), context);

  EXPECT_TRUE(outcome.succeeded());
  EXPECT_EQ(outcome.disposition(), manipulation::task::ObjectDisposition::kObjectAtTarget);
  EXPECT_EQ(context.checkpoint_count, 1);
  EXPECT_EQ(
    context.feedback_phases,
    (std::vector<manipulation::task::TaskPhase>{
        manipulation::task::TaskPhase::kPlanning,
        manipulation::task::TaskPhase::kPlacing,
      }));
  EXPECT_EQ(static_cast<RecordingStrategy &>(strategy).last_object_id, "cube_1");

  const auto before = TransferExecutionContext::Clock::now();
  EXPECT_EQ(
    context.wait_for(std::chrono::milliseconds{5}),
    TransferExecutionControlResult::kContinue);
  EXPECT_EQ(context.wait_count, 1);
  EXPECT_GE(context.last_wait_deadline, before);
}

}  // namespace
}  // namespace manipulation::workflow
