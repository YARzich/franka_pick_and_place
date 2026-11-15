// Responsibility: Verifies pure V2 resolution, lease lifecycle, interruption, and composition.

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

#include "manipulation/task/execution_control.hpp"
#include "manipulation/workflow/detail/transfer_sequence_session.hpp"
#include "manipulation/workflow/detail/transfer_sequence_task_context_factory.hpp"
#include "manipulation/workflow/resolved_transfer_workflow.hpp"
#include "manipulation/workflow/resolved_transfer_workflow_action_adapter.hpp"
#include "transfer_domain_test_support.hpp"

namespace manipulation::workflow
{
namespace
{

using namespace std::chrono_literals;
using ErrorCode = manipulation::task::TaskErrorCode;
using Disposition = manipulation::task::ObjectDisposition;
using Phase = manipulation::task::TaskPhase;
using ResourceReleaseDisposition = manipulation::task::ResourceReleaseDisposition;
using TaskOutcome = manipulation::task::TaskOutcome;
using Step = manipulation_msgs::msg::TransferObjectStepV2;
using Clock = TransferExecutionContext::Clock;

class FakeTimeContext final : public TransferExecutionContext
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

  TaskStopCause stop_cause() const noexcept override
  {
    return stopped ? stopped_cause : TaskStopCause::kNone;
  }

  TransferExecutionControlResult heartbeat() override
  {
    ++heartbeat_count;
    return stopped ? TransferExecutionControlResult::kStopped :
           TransferExecutionControlResult::kContinue;
  }

  TaskOperationGateResult run_operation(
    const std::function<void()> & operation) override
  {
    ++operation_gate_count;
    if (stopped) {
      return TaskOperationGateResult::kStopped;
    }
    if (paused_operation_attempts > 0 &&
      operation_gate_count >= pause_on_or_after_operation_call)
    {
      --paused_operation_attempts;
      now += paused_operation_advance;
      return TaskOperationGateResult::kPaused;
    }
    if (checkpoint() == TransferExecutionControlResult::kStopped) {
      return TaskOperationGateResult::kStopped;
    }
    operation();
    return TaskOperationGateResult::kExecuted;
  }

  TaskOperationGateResult run_maintenance_operation(
    const std::function<void()> & operation) override
  {
    ++maintenance_gate_count;
    if (stopped) {
      return TaskOperationGateResult::kStopped;
    }
    operation();
    return TaskOperationGateResult::kExecuted;
  }

  TransferExecutionControlResult wait_until(const TimePoint deadline) override
  {
    ++wait_count;
    if (stop_on_wait && wait_count >= *stop_on_wait) {
      stopped = true;
      return TransferExecutionControlResult::kStopped;
    }
    now = std::max(now, deadline);
    return TransferExecutionControlResult::kContinue;
  }

  void report_feedback(manipulation::task::TaskStateSnapshot feedback) override
  {
    task_feedback.emplace_back(std::move(feedback));
  }

  TimePoint now{Clock::now()};
  bool stopped{false};
  TaskStopCause stopped_cause{TaskStopCause::kClientCanceled};
  std::optional<int> stop_on_wait;
  int paused_operation_attempts{0};
  int pause_on_or_after_operation_call{1};
  Clock::duration paused_operation_advance{Clock::duration::zero()};
  int checkpoint_count{0};
  int wait_count{0};
  int heartbeat_count{0};
  int operation_gate_count{0};
  int maintenance_gate_count{0};
  std::vector<manipulation::task::TaskStateSnapshot> task_feedback;
};

PlanningSceneLease make_lease(const std::uint64_t revision)
{
  moveit_msgs::msg::PlanningScene scene;
  scene.name = "scene-" + std::to_string(revision);
  scene.is_diff = false;
  return PlanningSceneLease::create(
    PlanningSceneSnapshot::create("coordinator-1", revision, std::move(scene)),
    PlanningSceneAccessProof::create("goal-lease", "coordinator-1", revision));
}

class FakeSceneWorkflow final : public TransferSceneWorkflowPort
{
public:
  PlanningSceneLeaseResult acquire(
    const std::vector<ObjectInstance> & instances,
    TransferExecutionContext & context) override
  {
    ++acquire_count;
    acquired_ids.clear();
    for (const auto & instance : instances) {
      acquired_ids.emplace_back(instance.object_id());
    }
    if (acquire_wait > Clock::duration::zero() &&
      context.wait_for(acquire_wait) == TransferExecutionControlResult::kStopped)
    {
      return PlanningSceneLeaseResult::failure(
        ErrorCode::kCanceled, "scene acquisition stopped");
    }
    return PlanningSceneLeaseResult::success(make_lease(revision));
  }

  PlanningSceneAccessProofResult renew(
    const PlanningSceneAccessProof & proof,
    TransferExecutionContext & context) override
  {
    const auto dispatched = context.run_maintenance_operation(
      [this, &proof]() {
        ++renew_count;
        renewed_revisions.emplace_back(proof.revision());
      });
    if (dispatched != TaskOperationGateResult::kExecuted) {
      return PlanningSceneAccessProofResult::failure(
        ErrorCode::kCanceled, "lease renewal dispatch stopped");
    }
    if (renew_failure) {
      return PlanningSceneAccessProofResult::failure(
        *renew_failure, "scripted lease renewal failure");
    }
    if (on_renew) {
      on_renew();
    }
    return PlanningSceneAccessProofResult::success(
      PlanningSceneAccessProof::create(
        proof.lease_id(), proof.coordinator_session_id(), proof.revision()));
  }

  PlanningSceneLeaseResult stage_observation(
    const ObjectInstance & instance,
    const ObjectProfile & profile,
    const ObjectObservation & observation,
    const PlanningSceneAccessProof & proof,
    TransferExecutionContext & context) override
  {
    ++stage_count;
    staged_ids.emplace_back(instance.object_id());
    staged_profiles.emplace_back(profile.profile_id());
    staged_correlations.emplace_back(observation.correlation().correlation_id());
    staged_orientations.emplace_back(observation.pose().pose.orientation);
    EXPECT_EQ(proof.lease_id(), "goal-lease");
    if (stage_wait > Clock::duration::zero() &&
      context.wait_for(stage_wait) == TransferExecutionControlResult::kStopped)
    {
      return PlanningSceneLeaseResult::failure(
        ErrorCode::kCanceled, "scene staging stopped");
    }
    if (stage_failure) {
      return PlanningSceneLeaseResult::failure(
        *stage_failure, "scripted scene staging failure");
    }
    ++revision;
    return PlanningSceneLeaseResult::success(make_lease(revision));
  }

  PlanningSceneLeaseResult revalidate(
    const PlanningSceneAccessProof & proof,
    TransferExecutionContext & context) override
  {
    ++revalidate_count;
    if (report_revalidate_wait) {
      manipulation::task::TaskStateSnapshot waiting;
      waiting.phase = Phase::kWaitingForResources;
      waiting.detail = "backend scene readback wait";
      context.report_waiting(TransferWaitingReason::kRosEndpoint, std::move(waiting));
    }
    if (on_revalidate) {
      on_revalidate();
    }
    EXPECT_EQ(proof.lease_id(), "goal-lease");
    ++revision;
    return PlanningSceneLeaseResult::success(make_lease(revision));
  }

  void release(const PlanningSceneAccessProof & proof) noexcept override
  {
    ++release_count;
    released_revision = proof.revision();
  }

  void request_stop() noexcept override
  {
    stop_forwarded = true;
  }

  std::uint64_t revision{1U};
  Clock::duration acquire_wait{Clock::duration::zero()};
  Clock::duration stage_wait{Clock::duration::zero()};
  std::optional<ErrorCode> renew_failure;
  std::optional<ErrorCode> stage_failure;
  int acquire_count{0};
  int renew_count{0};
  int stage_count{0};
  int revalidate_count{0};
  bool report_revalidate_wait{false};
  int release_count{0};
  std::uint64_t released_revision{0U};
  bool stop_forwarded{false};
  std::function<void()> on_renew;
  std::function<void()> on_revalidate;
  std::vector<std::uint64_t> renewed_revisions;
  std::vector<std::string> acquired_ids;
  std::vector<std::string> staged_ids;
  std::vector<std::string> staged_profiles;
  std::vector<std::string> staged_correlations;
  std::vector<geometry_msgs::msg::Quaternion> staged_orientations;
};

class ScriptedObservationProvider : public ObservationProvider
{
public:
  ScriptedObservationProvider(
    std::string source_name,
    FakeTimeContext & clock,
    std::vector<double> positions,
    const Clock::duration wait = Clock::duration::zero(),
    std::vector<geometry_msgs::msg::Quaternion> orientations = {})
  : source_name_(std::move(source_name)), clock_(clock), positions_(std::move(positions)),
    wait_(wait), orientations_(std::move(orientations))
  {
  }

  ObservationProviderResult observe(
    const ObjectInstance & instance,
    const ObjectProfile &,
    TransferExecutionContext & context) override
  {
    ++call_count;
    if (wait_ > Clock::duration::zero() &&
      context.wait_until(clock_.now + wait_) == TransferExecutionControlResult::kStopped)
    {
      return ObservationProviderResult::failure(ErrorCode::kCanceled, "observation stopped");
    }
    const auto index = static_cast<std::size_t>(call_count - 1);
    auto pose = test_support::stamped_pose("world", positions_.at(index));
    if (!orientations_.empty()) {
      pose.pose.orientation = orientations_.at(index);
    }
    pose.header.stamp.sec += call_count;
    return ObservationProviderResult::success(ObjectObservation::create(
      instance.object_id(), std::move(pose),
      source_name_ + "-" + std::to_string(call_count), 0.95));
  }

  void request_stop() noexcept override
  {
    stop_forwarded = true;
  }

  int call_count{0};
  bool stop_forwarded{false};

private:
  std::string source_name_;
  FakeTimeContext & clock_;
  std::vector<double> positions_;
  Clock::duration wait_;
  std::vector<geometry_msgs::msg::Quaternion> orientations_;
};

class ConfiguredObservationProvider final : public ScriptedObservationProvider
{
public:
  ConfiguredObservationProvider(FakeTimeContext & clock, std::vector<double> positions)
  : ScriptedObservationProvider("configured", clock, std::move(positions))
  {
  }
};

class DetectedObservationProvider final : public ScriptedObservationProvider
{
public:
  DetectedObservationProvider(FakeTimeContext & clock, std::vector<double> positions)
  : ScriptedObservationProvider("detected", clock, std::move(positions))
  {
  }
};

class CheckpointSlicedObservationProvider final : public ObservationProvider
{
public:
  explicit CheckpointSlicedObservationProvider(FakeTimeContext & clock)
  : clock_(clock)
  {
  }

  ObservationProviderResult observe(
    const ObjectInstance & instance,
    const ObjectProfile &,
    TransferExecutionContext & context) override
  {
    for (int slice = 0; slice < 2; ++slice) {
      clock_.now += 5s;
      if (context.checkpoint() == TransferExecutionControlResult::kStopped) {
        return ObservationProviderResult::failure(
          ErrorCode::kCanceled, "checkpoint-sliced observation stopped");
      }
    }
    return ObservationProviderResult::success(ObjectObservation::create(
      instance.object_id(), test_support::stamped_pose(), "checkpoint-observation"));
  }

  void request_stop() noexcept override
  {
  }

private:
  FakeTimeContext & clock_;
};

class FailingObservationProvider final : public ObservationProvider
{
public:
  ObservationProviderResult observe(
    const ObjectInstance &,
    const ObjectProfile &,
    TransferExecutionContext &) override
  {
    return ObservationProviderResult::failure(
      ErrorCode::kPerceptionTimeout, "camera stream ended");
  }

  void request_stop() noexcept override
  {
  }
};

class FakeGraspProvider final : public GraspProvider
{
public:
  GraspProviderResult generate(
    const ObjectObservation & observation,
    const ObjectProfile &,
    const PlanningSceneSnapshot & snapshot,
    TransferExecutionContext &) override
  {
    ++call_count;
    correlations.emplace_back(observation.correlation().correlation_id());
    scene_revisions.emplace_back(snapshot.revision());
    std::vector<GraspCandidate> candidates;
    candidates.emplace_back(test_support::grasp_candidate(observation, 0.2, 0.02));
    candidates.emplace_back(test_support::grasp_candidate(observation, 0.9, 0.05));
    return GraspProviderResult::success(std::move(candidates));
  }

  void request_stop() noexcept override
  {
    stop_forwarded = true;
  }

  int call_count{0};
  bool stop_forwarded{false};
  std::vector<std::string> correlations;
  std::vector<std::uint64_t> scene_revisions;
};

class RecordingStrategy final : public TransferExecutionStrategy
{
public:
  RecordingStrategy(
    std::string strategy_id,
    std::vector<Phase> phases,
    std::optional<TransferWaitingReason> first_wait_reason = std::nullopt)
  : strategy_id_(std::move(strategy_id)), phases_(std::move(phases)),
    first_wait_reason_(first_wait_reason)
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
    ++call_count;
    target_positions.emplace_back(task.target_object_pose().pose.position.x);
    target_orientations.emplace_back(task.target_object_pose().pose.orientation);
    observation_orientations.emplace_back(task.observation().pose().pose.orientation);
    grasp_scores.emplace_back(task.grasp().score());
    correlation_ids.emplace_back(task.grasp().correlation().correlation_id());
    scene_revisions.emplace_back(task.scene_snapshot().revision());
    proof_revisions.emplace_back(task.scene_access_proof().revision());
    for (std::size_t index = 0U; index < phases_.size(); ++index) {
      manipulation::task::TaskStateSnapshot state;
      state.phase = phases_[index];
      state.progress = 0.5F;
      if (first_wait_reason_ && index == 0U) {
        state.detail = "strategy-owned typed wait";
        context.report_waiting(*first_wait_reason_, std::move(state));
      } else if (first_wait_reason_ && index == 1U) {
        context.report_waiting(TransferWaitingReason::kNone, std::move(state));
      } else {
        context.report_feedback(std::move(state));
      }
    }
    if (terminal_feedback) {
      context.report_feedback(*terminal_feedback);
    }
    ++physical_side_effect_count;
    return manipulation::task::TaskOutcome::make_success(
      manipulation::task::ObjectDisposition::kObjectAtTarget);
  }

  void request_stop() noexcept override
  {
    stop_forwarded = true;
  }

  int call_count{0};
  int physical_side_effect_count{0};
  bool stop_forwarded{false};
  std::optional<manipulation::task::TaskStateSnapshot> terminal_feedback;
  std::vector<double> target_positions;
  std::vector<geometry_msgs::msg::Quaternion> target_orientations;
  std::vector<geometry_msgs::msg::Quaternion> observation_orientations;
  std::vector<double> grasp_scores;
  std::vector<std::string> correlation_ids;
  std::vector<std::uint64_t> scene_revisions;
  std::vector<std::uint64_t> proof_revisions;

private:
  std::string strategy_id_;
  std::vector<Phase> phases_;
  std::optional<TransferWaitingReason> first_wait_reason_;
};

class GatedDispatchStrategy final : public TransferExecutionStrategy
{
public:
  std::string_view id() const noexcept override
  {
    return "gated_dispatch";
  }

  manipulation::task::TaskOutcome execute(
    const ResolvedTransferTask &,
    TransferExecutionContext & context) override
  {
    manipulation::task::TaskStateSnapshot waiting;
    waiting.phase = Phase::kPlanning;
    waiting.detail = "waiting to dispatch planned motion";
    context.report_waiting(TransferWaitingReason::kPlanning, std::move(waiting));
    const auto result = context.run_operation([this]() {++dispatch_count;});
    if (result != TaskOperationGateResult::kExecuted) {
      return manipulation::task::TaskOutcome::make_failure(
        ErrorCode::kCanceled, manipulation::task::ObjectDisposition::kUnknown, 0U,
        "gated dispatch stopped");
    }
    manipulation::task::TaskStateSnapshot executing;
    executing.phase = Phase::kTransferring;
    executing.detail = "planned motion accepted";
    context.report_waiting(TransferWaitingReason::kNone, std::move(executing));
    return manipulation::task::TaskOutcome::make_success(
      manipulation::task::ObjectDisposition::kObjectAtTarget);
  }

  void request_stop() noexcept override
  {
  }

  int dispatch_count{0};
};

class TerminalOutcomeStrategy final : public TransferExecutionStrategy
{
public:
  using Callback = std::function<void(TransferExecutionContext &)>;

  explicit TerminalOutcomeStrategy(
    TaskOutcome outcome,
    Callback callback = {})
  : outcome_(std::move(outcome)), callback_(std::move(callback))
  {
  }

  std::string_view id() const noexcept override
  {
    return "terminal_outcome";
  }

  TaskOutcome execute(
    const ResolvedTransferTask &,
    TransferExecutionContext & context) override
  {
    ++call_count;
    if (callback_) {
      callback_(context);
    }
    return outcome_;
  }

  void request_stop() noexcept override
  {
    stop_forwarded = true;
  }

  int call_count{0};
  bool stop_forwarded{false};

private:
  TaskOutcome outcome_;
  Callback callback_;
};

ObjectCatalog catalog()
{
  return ObjectCatalog::create(
    {test_support::profile()},
      {
        test_support::instance(),
        ObjectInstance::create("cube_2", "cube_profile"),
    });
}

ObjectCatalog catalog_with_profile(ObjectProfile profile)
{
  return ObjectCatalog::create(
    {std::move(profile)}, {test_support::instance()});
}

geometry_msgs::msg::Quaternion quaternion(
  const double x, const double y, const double z, const double w)
{
  geometry_msgs::msg::Quaternion value;
  value.x = x;
  value.y = y;
  value.z = z;
  value.w = w;
  return value;
}

Step absolute_step(const std::string & object_id = "cube_1", const double x = 4.0)
{
  Step step;
  step.object_id = object_id;
  step.target_mode = Step::TARGET_ABSOLUTE_POSE;
  step.target_object_pose = test_support::stamped_pose("world", x);
  step.recovery_policy = Step::RECOVERY_SAFE_STOP;
  return step;
}

ResolvedTransferWorkflow::Goal absolute_goal(const std::string & object_id = "cube_1")
{
  ResolvedTransferWorkflow::Goal goal;
  goal.steps.emplace_back(absolute_step(object_id));
  return goal;
}

ResolvedTransferWorkflow::Goal translated_return_goal()
{
  ResolvedTransferWorkflow::Goal goal;
  auto translated = absolute_step();
  translated.target_mode = Step::TARGET_TRANSLATION_FROM_CURRENT_SOURCE;
  translated.target_object_pose = geometry_msgs::msg::PoseStamped{};
  translated.target_translation.header.frame_id = "world";
  translated.target_translation.header.stamp = test_support::test_stamp();
  translated.target_translation.vector.x = 2.0;
  goal.steps.emplace_back(std::move(translated));
  auto initial = absolute_step();
  initial.target_mode = Step::TARGET_INITIAL_SOURCE;
  initial.target_object_pose = geometry_msgs::msg::PoseStamped{};
  goal.steps.emplace_back(std::move(initial));
  return goal;
}

ResolvedTransferWorkflowOptions options(FakeTimeContext & context)
{
  ResolvedTransferWorkflowOptions value;
  value.validation.planning_frame = "world";
  value.validation.supported_recovery_policies = {
    Step::RECOVERY_NONE,
    Step::RECOVERY_SAFE_STOP,
    Step::RECOVERY_RETREAT,
    Step::RECOVERY_RETREAT_AND_HOME,
  };
  value.scene_renew_period = 30s;
  value.now = [&context]() {return context.now;};
  return value;
}

struct StrategyFixture
{
  explicit StrategyFixture(
    std::vector<Phase> phases = {Phase::kPlanning, Phase::kPlacing},
    std::optional<TransferWaitingReason> first_wait_reason = std::nullopt)
  {
    auto owned = std::make_unique<RecordingStrategy>(
      "test_strategy", std::move(phases), first_wait_reason);
    strategy = owned.get();
    std::vector<std::unique_ptr<TransferExecutionStrategy>> entries;
    entries.emplace_back(std::move(owned));
    registry = std::make_unique<StrategyRegistry>(
      StrategyRegistry::for_testing(std::move(entries)));
  }

  RecordingStrategy * strategy{nullptr};
  std::unique_ptr<StrategyRegistry> registry;
};

struct TerminalStrategyFixture
{
  explicit TerminalStrategyFixture(
    TaskOutcome outcome,
    TerminalOutcomeStrategy::Callback callback = {})
  {
    auto owned = std::make_unique<TerminalOutcomeStrategy>(
      std::move(outcome), std::move(callback));
    strategy = owned.get();
    std::vector<std::unique_ptr<TransferExecutionStrategy>> entries;
    entries.emplace_back(std::move(owned));
    registry = std::make_unique<StrategyRegistry>(
      StrategyRegistry::for_testing(std::move(entries)));
  }

  TerminalOutcomeStrategy * strategy{nullptr};
  std::unique_ptr<StrategyRegistry> registry;
};

template<typename ObservationProviderT>
void expect_provider_composes(ObservationProviderT & provider, FakeTimeContext & context)
{
  const auto object_catalog = catalog();
  FakeGraspProvider grasps;
  FakeSceneWorkflow scene;
  StrategyFixture strategies;
  ResolvedTransferWorkflow workflow(
    options(context), object_catalog, provider, grasps, scene, *strategies.registry,
    "test_strategy");

  const auto result = workflow.execute(absolute_goal(), context);

  EXPECT_TRUE(result.result.outcome.succeeded());
  EXPECT_EQ(provider.call_count, 1);
  EXPECT_EQ(grasps.call_count, 1);
  EXPECT_EQ(strategies.strategy->call_count, 1);
  EXPECT_EQ(scene.release_count, 1);
}

TEST(ResolvedTransferWorkflow, AdmissionValidatesWireIntentAndCatalogBeforeAcquire)
{
  FakeTimeContext context;
  ConfiguredObservationProvider observations(context, {1.0});
  FakeGraspProvider grasps;
  FakeSceneWorkflow scene;
  StrategyFixture strategies;
  const auto object_catalog = catalog();
  ResolvedTransferWorkflow workflow(
    options(context), object_catalog, observations, grasps, scene, *strategies.registry,
    "test_strategy");

  auto unknown = absolute_goal("unknown_object");
  const auto admission = workflow.admit(unknown);
  const auto execution = workflow.execute(unknown, context);

  EXPECT_FALSE(admission.accepted);
  EXPECT_NE(admission.reason.find("catalog"), std::string::npos);
  EXPECT_EQ(execution.result.outcome.error_code(), ErrorCode::kInvalidGoal);
  EXPECT_EQ(scene.acquire_count, 0);
  EXPECT_EQ(observations.call_count, 0);
}

TEST(ResolvedTransferWorkflow, ConfiguredAndDetectedObservationProvidersAreInterchangeable)
{
  FakeTimeContext configured_context;
  ConfiguredObservationProvider configured(configured_context, {1.0});
  expect_provider_composes(configured, configured_context);

  FakeTimeContext detected_context;
  DetectedObservationProvider detected(detected_context, {1.0});
  expect_provider_composes(detected, detected_context);
}

TEST(ResolvedTransferWorkflow, FinalReadbackPreservesCurrentStepProgressAndCounters)
{
  FakeTimeContext context;
  ConfiguredObservationProvider observations(context, {1.0});
  FakeGraspProvider grasps;
  FakeSceneWorkflow scene;
  scene.report_revalidate_wait = true;
  StrategyFixture strategies;
  manipulation::task::TaskStateSnapshot terminal;
  terminal.phase = Phase::kCompleted;
  terminal.progress = 1.0F;
  terminal.step_index = 12U;
  terminal.step_count = 13U;
  terminal.retry_count = 2U;
  terminal.recovery_count = 1U;
  strategies.strategy->terminal_feedback = terminal;
  const auto object_catalog = catalog();
  ResolvedTransferWorkflow workflow(
    options(context), object_catalog, observations, grasps, scene, *strategies.registry,
    "test_strategy");
  std::vector<TransferSequenceFeedbackSnapshot> feedback;
  const auto result = workflow.execute(
    absolute_goal(), context,
    [&feedback](auto update) {feedback.emplace_back(std::move(update));});
  ASSERT_TRUE(result.result.outcome.succeeded());
  const auto readback = std::find_if(feedback.begin(), feedback.end(), [](const auto & update) {
        return update.current_state.detail == "confirming scene revision after execution";
    });
  ASSERT_NE(readback, feedback.end());
  for (auto update = readback; update != feedback.end(); ++update) {
    EXPECT_FLOAT_EQ(update->current_state.progress, 1.0F);
    EXPECT_EQ(update->current_state.step_index, 12U);
    EXPECT_EQ(update->current_state.step_count, 13U);
    EXPECT_EQ(update->current_state.retry_count, 2U);
    EXPECT_EQ(update->current_state.recovery_count, 1U);
  }
}

TEST(ResolvedTransferWorkflow, ResolvesTargetsRankingCorrelationAndSceneRevisions)
{
  FakeTimeContext context;
  ConfiguredObservationProvider observations(context, {1.0, 5.0, 2.0});
  FakeGraspProvider grasps;
  FakeSceneWorkflow scene;
  StrategyFixture strategies;
  const auto object_catalog = catalog();
  ResolvedTransferWorkflow workflow(
    options(context), object_catalog, observations, grasps, scene, *strategies.registry,
    "test_strategy");

  ResolvedTransferWorkflow::Goal goal;
  auto translated = absolute_step();
  translated.target_mode = Step::TARGET_TRANSLATION_FROM_CURRENT_SOURCE;
  translated.target_object_pose = geometry_msgs::msg::PoseStamped{};
  translated.target_translation.header.frame_id = "world";
  translated.target_translation.header.stamp = test_support::test_stamp();
  translated.target_translation.vector.x = 2.0;
  goal.steps.emplace_back(std::move(translated));
  auto initial = absolute_step();
  initial.target_mode = Step::TARGET_INITIAL_SOURCE;
  initial.target_object_pose = geometry_msgs::msg::PoseStamped{};
  goal.steps.emplace_back(std::move(initial));
  goal.steps.emplace_back(absolute_step("cube_2", 9.0));
  std::vector<TransferSequenceFeedbackSnapshot> feedback;

  const auto result = workflow.execute(
    goal, context,
    [&feedback](TransferSequenceFeedbackSnapshot value) {
      feedback.emplace_back(std::move(value));
    });

  ASSERT_TRUE(result.result.outcome.succeeded());
  EXPECT_EQ(result.result.completed_step_count, 3U);
  EXPECT_EQ(scene.acquired_ids, (std::vector<std::string>{"cube_1", "cube_2"}));
  EXPECT_EQ(strategies.strategy->target_positions, (std::vector<double>{3.0, 1.0, 9.0}));
  EXPECT_EQ(strategies.strategy->grasp_scores, (std::vector<double>{0.9, 0.9, 0.9}));
  EXPECT_EQ(strategies.strategy->correlation_ids, grasps.correlations);
  EXPECT_EQ(grasps.scene_revisions, (std::vector<std::uint64_t>{2U, 5U, 8U}));
  EXPECT_EQ(strategies.strategy->scene_revisions,
    (std::vector<std::uint64_t>{3U, 6U, 9U}));
  EXPECT_EQ(strategies.strategy->proof_revisions,
    strategies.strategy->scene_revisions);
  EXPECT_EQ(scene.stage_count, 3);
  EXPECT_EQ(scene.staged_ids, (std::vector<std::string>{"cube_1", "cube_1", "cube_2"}));
  EXPECT_EQ(scene.staged_correlations, grasps.correlations);
  EXPECT_EQ(scene.revalidate_count, 6);
  EXPECT_EQ(scene.release_count, 1);
  EXPECT_EQ(scene.released_revision, 10U);
  EXPECT_TRUE(std::any_of(
    feedback.cbegin(), feedback.cend(), [](const auto & value) {
        return value.active_object_id == "cube_1" &&
               value.waiting_reason == TransferWaitingReason::kObservation;
    }));
  EXPECT_TRUE(std::any_of(
    feedback.cbegin(), feedback.cend(), [](const auto & value) {
        return value.active_object_id == "cube_2" &&
               value.waiting_reason == TransferWaitingReason::kGraspGeneration;
    }));
}

TEST(ResolvedTransferWorkflow, InitialSourceUsesNearestProfileDeclaredEquivalentOrientation)
{
  constexpr double kHalfSqrtTwo = 0.7071067811865476;
  const auto initial_orientation = quaternion(kHalfSqrtTwo, 0.0, 0.0, kHalfSqrtTwo);
  const auto local_symmetry = quaternion(0.0, 0.0, kHalfSqrtTwo, kHalfSqrtTwo);
  const auto current_orientation = quaternion(0.5, -0.5, 0.5, 0.5);
  const auto identity = quaternion(0.0, 0.0, 0.0, 1.0);
  const auto base = test_support::profile();
  auto object_catalog = catalog_with_profile(ObjectProfile::create(
      "cube_profile", base.geometry(), std::nullopt, {identity, local_symmetry}));
  FakeTimeContext context;
  ScriptedObservationProvider observations(
    "symmetric", context, {1.0, 5.0}, Clock::duration::zero(),
    {initial_orientation, current_orientation});
  FakeGraspProvider grasps;
  FakeSceneWorkflow scene;
  StrategyFixture strategies;
  ResolvedTransferWorkflow workflow(
    options(context), object_catalog, observations, grasps, scene, *strategies.registry,
    "test_strategy");

  const auto result = workflow.execute(translated_return_goal(), context);

  ASSERT_TRUE(result.result.outcome.succeeded());
  ASSERT_EQ(strategies.strategy->target_orientations.size(), 2U);
  const auto & return_target = strategies.strategy->target_orientations.back();
  EXPECT_DOUBLE_EQ(strategies.strategy->target_positions.back(), 1.0);
  EXPECT_NEAR(return_target.x, current_orientation.x, 1.0e-12);
  EXPECT_NEAR(return_target.y, current_orientation.y, 1.0e-12);
  EXPECT_NEAR(return_target.z, current_orientation.z, 1.0e-12);
  EXPECT_NEAR(return_target.w, current_orientation.w, 1.0e-12);
  ASSERT_EQ(strategies.strategy->observation_orientations.size(), 2U);
  EXPECT_DOUBLE_EQ(
    strategies.strategy->observation_orientations.back().y, current_orientation.y);
  ASSERT_EQ(scene.staged_orientations.size(), 2U);
  EXPECT_DOUBLE_EQ(scene.staged_orientations.back().y, current_orientation.y);
}

TEST(ResolvedTransferWorkflow, InitialSourcePreservesOrientationWithoutDeclaredSymmetry)
{
  constexpr double kHalfSqrtTwo = 0.7071067811865476;
  const auto initial_orientation = quaternion(kHalfSqrtTwo, 0.0, 0.0, kHalfSqrtTwo);
  const auto current_orientation = quaternion(0.5, -0.5, 0.5, 0.5);
  FakeTimeContext context;
  ScriptedObservationProvider observations(
    "asymmetric", context, {1.0, 5.0}, Clock::duration::zero(),
    {initial_orientation, current_orientation});
  FakeGraspProvider grasps;
  FakeSceneWorkflow scene;
  StrategyFixture strategies;
  auto object_catalog = catalog_with_profile(test_support::profile());
  ResolvedTransferWorkflow workflow(
    options(context), object_catalog, observations, grasps, scene, *strategies.registry,
    "test_strategy");

  const auto result = workflow.execute(translated_return_goal(), context);

  ASSERT_TRUE(result.result.outcome.succeeded());
  ASSERT_EQ(strategies.strategy->target_orientations.size(), 2U);
  const auto & return_target = strategies.strategy->target_orientations.back();
  EXPECT_DOUBLE_EQ(return_target.x, initial_orientation.x);
  EXPECT_DOUBLE_EQ(return_target.y, initial_orientation.y);
  EXPECT_DOUBLE_EQ(return_target.z, initial_orientation.z);
  EXPECT_DOUBLE_EQ(return_target.w, initial_orientation.w);
  EXPECT_NE(return_target.y, current_orientation.y);
}

TEST(ResolvedTransferWorkflow, WaitBeyondFormerGoalLimitRenewsLeaseAndContinues)
{
  FakeTimeContext context;
  ScriptedObservationProvider observations("slow", context, {1.0}, 130s);
  FakeGraspProvider grasps;
  FakeSceneWorkflow scene;
  StrategyFixture strategies;
  const auto object_catalog = catalog();
  std::vector<WaitLifecycleRecord> wait_records;
  auto configured_options = options(context);
  configured_options.wait_logger = std::make_shared<WaitLifecycleLogger>(
    [&wait_records](WaitLifecycleRecord record) {
      wait_records.emplace_back(std::move(record));
    },
    [&context]() {return context.now;});
  ResolvedTransferWorkflow workflow(
    std::move(configured_options), object_catalog, observations, grasps, scene,
    *strategies.registry,
    "test_strategy");

  const auto result = workflow.execute(
    absolute_goal(), context, {}, "slow-goal-17");

  ASSERT_TRUE(result.result.outcome.succeeded());
  ASSERT_EQ(result.step_elapsed.size(), 1U);
  EXPECT_GE(result.step_elapsed.front(), 130s);
  EXPECT_EQ(scene.renew_count, 4);
  EXPECT_EQ(strategies.strategy->physical_side_effect_count, 1);
  EXPECT_TRUE(std::any_of(
    wait_records.cbegin(), wait_records.cend(), [](const auto & record) {
        return record.event == WaitLifecycleEvent::kWaiting &&
               record.scope.goal_id == "slow-goal-17" &&
               record.scope.object_id == "cube_1" &&
               record.scope.reason == TransferWaitingReason::kObservation;
  }));
}

TEST(ResolvedTransferWorkflow, WaitUntilAppliesPauseOnlyAtItsSafeCompletionBoundary)
{
  FakeTimeContext context;
  context.paused_operation_attempts = 2;
  context.pause_on_or_after_operation_call = 2;
  context.paused_operation_advance = 5s;
  ScriptedObservationProvider observations("paused-wait", context, {1.0}, 1s);
  FakeGraspProvider grasps;
  FakeSceneWorkflow scene;
  StrategyFixture strategies;
  auto configured_options = options(context);
  configured_options.scene_renew_period = 2s;
  const auto object_catalog = catalog();
  ResolvedTransferWorkflow workflow(
    std::move(configured_options), object_catalog, observations, grasps, scene,
    *strategies.registry, "test_strategy");

  const auto result = workflow.execute(absolute_goal(), context);

  ASSERT_TRUE(result.result.outcome.succeeded());
  EXPECT_EQ(context.paused_operation_attempts, 0);
  EXPECT_GE(scene.renew_count, 2);
  EXPECT_EQ(context.maintenance_gate_count, scene.renew_count);
  EXPECT_EQ(strategies.strategy->physical_side_effect_count, 1);
}

TEST(WaitLifecycleLogger, EmitsInfoEnterThrottledWaitingAndExitWithoutSpam)
{
  auto now = WaitLifecycleLogger::Clock::now();
  std::vector<WaitLifecycleRecord> records;
  WaitLifecycleLogger logger(
    [&records](WaitLifecycleRecord record) {
      records.emplace_back(std::move(record));
    },
    [&now]() {return now;});

  logger.enter({
        "goal-42", 2U, 4U, "cube_2", TransferWaitingReason::kHealthReadiness,
        "waiting for healthy motion backend"});
  logger.waiting();
  now += 4s;
  logger.waiting();
  now += 1s;
  logger.waiting();
  logger.waiting();
  logger.exit(WaitLifecycleOutcome::kReady, "health became ready");

  ASSERT_EQ(records.size(), 3U);
  EXPECT_EQ(records.at(0).event, WaitLifecycleEvent::kEnter);
  EXPECT_EQ(records.at(1).event, WaitLifecycleEvent::kWaiting);
  EXPECT_EQ(records.at(2).event, WaitLifecycleEvent::kExit);
  EXPECT_FALSE(records.at(0).outcome);
  EXPECT_FALSE(records.at(1).outcome);
  ASSERT_TRUE(records.at(2).outcome);
  EXPECT_EQ(*records.at(2).outcome, WaitLifecycleOutcome::kReady);
  EXPECT_TRUE(std::all_of(
    records.cbegin(), records.cend(), [](const auto & record) {
        return record.severity == WaitLogSeverity::kInfo &&
               record.scope.goal_id == "goal-42" &&
               record.scope.step_index == 2U && record.scope.step_count == 4U &&
               record.scope.object_id == "cube_2" &&
               record.scope.reason == TransferWaitingReason::kHealthReadiness;
    }));
  EXPECT_EQ(records.at(0).scope.diagnostic, "waiting for healthy motion backend");
  EXPECT_EQ(records.at(2).scope.diagnostic, "health became ready");
  EXPECT_EQ(records.at(0).elapsed, 0s);
  EXPECT_EQ(records.at(1).elapsed, 5s);
  EXPECT_EQ(records.at(2).elapsed, 5s);
}

TEST(ResolvedTransferWorkflow, PauseBeforeDispatchKeepsLeaseAndWaitDiagnosticsAlive)
{
  FakeTimeContext context;
  context.paused_operation_attempts = 3;
  context.pause_on_or_after_operation_call = 2;
  context.paused_operation_advance = 5s;
  ConfiguredObservationProvider observations(context, {1.0});
  FakeGraspProvider grasps;
  FakeSceneWorkflow scene;
  auto strategy = std::make_unique<GatedDispatchStrategy>();
  auto * strategy_view = strategy.get();
  std::vector<std::unique_ptr<TransferExecutionStrategy>> entries;
  entries.emplace_back(std::move(strategy));
  auto registry = StrategyRegistry::for_testing(std::move(entries));
  std::vector<WaitLifecycleRecord> records;
  auto configured_options = options(context);
  configured_options.scene_renew_period = 2s;
  configured_options.wait_logger = std::make_shared<WaitLifecycleLogger>(
    [&records](WaitLifecycleRecord record) {
      records.emplace_back(std::move(record));
    },
    [&context]() {return context.now;});
  const auto object_catalog = catalog();
  ResolvedTransferWorkflow workflow(
    std::move(configured_options), object_catalog, observations, grasps, scene,
    registry, "gated_dispatch");

  const auto result = workflow.execute(
    absolute_goal(), context, {}, "paused-dispatch-goal");

  ASSERT_TRUE(result.result.outcome.succeeded());
  EXPECT_EQ(strategy_view->dispatch_count, 1);
  EXPECT_GE(scene.renew_count, 3);
  EXPECT_GE(context.heartbeat_count, 3);
  EXPECT_EQ(context.maintenance_gate_count, scene.renew_count);
  const auto exit = std::find_if(
    records.cbegin(), records.cend(), [](const auto & record) {
      return record.event == WaitLifecycleEvent::kExit &&
             record.scope.reason == TransferWaitingReason::kPlanning;
    });
  ASSERT_NE(exit, records.cend());
  ASSERT_TRUE(exit->outcome);
  EXPECT_EQ(*exit->outcome, WaitLifecycleOutcome::kReady);
  EXPECT_EQ(exit->scope.diagnostic, "waiting to dispatch planned motion");
  EXPECT_TRUE(std::any_of(
    records.cbegin(), records.cend(), [](const auto & record) {
        return record.event == WaitLifecycleEvent::kWaiting &&
               record.scope.reason == TransferWaitingReason::kPlanning;
    }));
}

TEST(ResolvedTransferWorkflow, WaitExitReportsCanceledWithLastDiagnostic)
{
  FakeTimeContext context;
  context.stop_on_wait = 1;
  ScriptedObservationProvider observations("cancel", context, {1.0}, 10s);
  FakeGraspProvider grasps;
  FakeSceneWorkflow scene;
  StrategyFixture strategies;
  std::vector<WaitLifecycleRecord> records;
  auto configured_options = options(context);
  configured_options.wait_logger = std::make_shared<WaitLifecycleLogger>(
    [&records](WaitLifecycleRecord record) {
      records.emplace_back(std::move(record));
    },
    [&context]() {return context.now;});
  const auto object_catalog = catalog();
  ResolvedTransferWorkflow workflow(
    std::move(configured_options), object_catalog, observations, grasps, scene,
    *strategies.registry, "test_strategy");

  const auto result = workflow.execute(absolute_goal(), context);

  EXPECT_EQ(result.result.outcome.error_code(), ErrorCode::kCanceled);
  const auto exit = std::find_if(
    records.cbegin(), records.cend(), [](const auto & record) {
      return record.event == WaitLifecycleEvent::kExit &&
             record.scope.reason == TransferWaitingReason::kObservation;
    });
  ASSERT_NE(exit, records.cend());
  ASSERT_TRUE(exit->outcome);
  EXPECT_EQ(*exit->outcome, WaitLifecycleOutcome::kCanceled);
  EXPECT_EQ(exit->scope.diagnostic, "waiting for stamped object observation");
}

TEST(ResolvedTransferWorkflow, WaitExitDistinguishesShutdownAndProviderError)
{
  for (const auto shutdown : {false, true}) {
    FakeTimeContext context;
    FakeGraspProvider grasps;
    FakeSceneWorkflow scene;
    StrategyFixture strategies;
    std::vector<WaitLifecycleRecord> records;
    auto configured_options = options(context);
    configured_options.wait_logger = std::make_shared<WaitLifecycleLogger>(
      [&records](WaitLifecycleRecord record) {
        records.emplace_back(std::move(record));
      },
      [&context]() {return context.now;});
    const auto object_catalog = catalog();

    std::unique_ptr<ObservationProvider> observations;
    if (shutdown) {
      context.stop_on_wait = 1;
      context.stopped_cause = TaskStopCause::kShutdown;
      observations = std::make_unique<ScriptedObservationProvider>(
        "shutdown", context, std::vector<double>{1.0}, 10s);
    } else {
      observations = std::make_unique<FailingObservationProvider>();
    }
    ResolvedTransferWorkflow workflow(
      std::move(configured_options), object_catalog, *observations, grasps, scene,
      *strategies.registry, "test_strategy");

    const auto result = workflow.execute(absolute_goal(), context);

    EXPECT_FALSE(result.result.outcome.succeeded());
    const auto exit = std::find_if(
      records.cbegin(), records.cend(), [](const auto & record) {
        return record.event == WaitLifecycleEvent::kExit &&
               record.scope.reason == TransferWaitingReason::kObservation;
      });
    ASSERT_NE(exit, records.cend());
    ASSERT_TRUE(exit->outcome);
    EXPECT_EQ(
      *exit->outcome,
      shutdown ? WaitLifecycleOutcome::kShutdown : WaitLifecycleOutcome::kError);
    EXPECT_EQ(exit->scope.diagnostic, "waiting for stamped object observation");
  }
}

TEST(ResolvedTransferWorkflow, LongLeaseAcquireEmitsElapsedWaitingLifecycle)
{
  FakeTimeContext context;
  ConfiguredObservationProvider observations(context, {1.0});
  FakeGraspProvider grasps;
  FakeSceneWorkflow scene;
  scene.acquire_wait = 130s;
  StrategyFixture strategies;
  std::vector<WaitLifecycleRecord> records;
  auto configured_options = options(context);
  configured_options.wait_logger = std::make_shared<WaitLifecycleLogger>(
    [&records](WaitLifecycleRecord record) {
      records.emplace_back(std::move(record));
    },
    [&context]() {return context.now;});
  const auto object_catalog = catalog();
  ResolvedTransferWorkflow workflow(
    std::move(configured_options), object_catalog, observations, grasps, scene,
    *strategies.registry, "test_strategy");

  const auto result = workflow.execute(
    absolute_goal(), context, {}, "long-acquire-goal");

  ASSERT_TRUE(result.result.outcome.succeeded());
  ASSERT_GE(records.size(), 3U);
  EXPECT_EQ(records.front().event, WaitLifecycleEvent::kEnter);
  EXPECT_EQ(records.front().scope.reason, TransferWaitingReason::kSceneReadiness);
  EXPECT_EQ(records.front().scope.goal_id, "long-acquire-goal");
  EXPECT_EQ(records.front().scope.object_id, "cube_1");
  EXPECT_EQ(records.at(1).event, WaitLifecycleEvent::kWaiting);
  EXPECT_GE(records.at(1).elapsed, 130s);
  EXPECT_EQ(records.at(2).event, WaitLifecycleEvent::kExit);
  EXPECT_GE(records.at(2).elapsed, 130s);
}

TEST(ResolvedTransferWorkflow, LongSceneStagingRenewsGoalLeaseBeforeGrasp)
{
  FakeTimeContext context;
  ConfiguredObservationProvider observations(context, {1.0});
  FakeGraspProvider grasps;
  FakeSceneWorkflow scene;
  scene.stage_wait = 71s;
  StrategyFixture strategies;
  const auto object_catalog = catalog();
  ResolvedTransferWorkflow workflow(
    options(context), object_catalog, observations, grasps, scene,
    *strategies.registry, "test_strategy");

  const auto result = workflow.execute(absolute_goal(), context);

  ASSERT_TRUE(result.result.outcome.succeeded());
  EXPECT_EQ(scene.stage_count, 1);
  EXPECT_EQ(scene.renew_count, 2);
  EXPECT_EQ(grasps.call_count, 1);
  EXPECT_EQ(strategies.strategy->physical_side_effect_count, 1);
}

TEST(ResolvedTransferWorkflow, AmbiguousSceneStagingFailureRetainsGoalLease)
{
  FakeTimeContext context;
  ConfiguredObservationProvider observations(context, {1.0});
  FakeGraspProvider grasps;
  FakeSceneWorkflow scene;
  scene.stage_failure = ErrorCode::kSceneFailed;
  StrategyFixture strategies;
  const auto object_catalog = catalog();
  ResolvedTransferWorkflow workflow(
    options(context), object_catalog, observations, grasps, scene,
    *strategies.registry, "test_strategy");

  const auto result = workflow.execute(absolute_goal(), context);

  EXPECT_EQ(result.result.outcome.error_code(), ErrorCode::kSceneFailed);
  EXPECT_EQ(result.result.outcome.disposition(), Disposition::kSceneUncertain);
  EXPECT_EQ(scene.stage_count, 1);
  EXPECT_EQ(grasps.call_count, 0);
  EXPECT_EQ(strategies.strategy->call_count, 0);
  EXPECT_EQ(scene.release_count, 0);
}

TEST(ResolvedTransferWorkflow, UnsafeTerminalObjectDispositionRetainsGoalLease)
{
  for (const auto disposition : {
        Disposition::kHoldingObject,
        Disposition::kSceneUncertain,
        Disposition::kUnknown})
  {
    SCOPED_TRACE(static_cast<int>(disposition));
    FakeTimeContext context;
    ConfiguredObservationProvider observations(context, {1.0});
    FakeGraspProvider grasps;
    FakeSceneWorkflow scene;
    TerminalStrategyFixture strategies(TaskOutcome::make_failure(
        ErrorCode::kExecutionFailed, disposition, 0U,
        "scripted terminal object state"));
    const auto object_catalog = catalog();
    ResolvedTransferWorkflow workflow(
      options(context), object_catalog, observations, grasps, scene,
      *strategies.registry, "terminal_outcome");

    const auto result = workflow.execute(absolute_goal(), context);

    EXPECT_EQ(result.result.outcome.error_code(), ErrorCode::kExecutionFailed);
    EXPECT_EQ(result.result.outcome.disposition(), disposition);
    EXPECT_TRUE(result.result.outcome.resources_release_safe());
    EXPECT_EQ(strategies.strategy->call_count, 1);
    EXPECT_EQ(scene.release_count, 0);
  }
}

TEST(ResolvedTransferWorkflow, SafeTerminalSourceDispositionReleasesGoalLease)
{
  FakeTimeContext context;
  ConfiguredObservationProvider observations(context, {1.0});
  FakeGraspProvider grasps;
  FakeSceneWorkflow scene;
  TerminalStrategyFixture strategies(TaskOutcome::make_failure(
      ErrorCode::kPlanningFailed, Disposition::kObjectAtSource, 0U,
      "planning rejected before motion"));
  const auto object_catalog = catalog();
  ResolvedTransferWorkflow workflow(
    options(context), object_catalog, observations, grasps, scene,
    *strategies.registry, "terminal_outcome");

  const auto result = workflow.execute(absolute_goal(), context);

  EXPECT_EQ(result.result.outcome.error_code(), ErrorCode::kPlanningFailed);
  EXPECT_EQ(result.result.outcome.disposition(), Disposition::kObjectAtSource);
  EXPECT_TRUE(result.result.outcome.resources_release_safe());
  EXPECT_EQ(scene.release_count, 1);
}

TEST(ResolvedTransferWorkflow, PhysicalCommandQuarantineRetainsOtherwiseSafeGoalLease)
{
  FakeTimeContext context;
  ConfiguredObservationProvider observations(context, {1.0});
  FakeGraspProvider grasps;
  FakeSceneWorkflow scene;
  TerminalStrategyFixture strategies(TaskOutcome::make_failure(
      ErrorCode::kExecutionFailed, Disposition::kObjectAtSource, 1U,
      "accepted motion command has no terminal result",
      ResourceReleaseDisposition::kQuarantineRequired));
  const auto object_catalog = catalog();
  ResolvedTransferWorkflow workflow(
    options(context), object_catalog, observations, grasps, scene,
    *strategies.registry, "terminal_outcome");

  const auto result = workflow.execute(absolute_goal(), context);

  EXPECT_EQ(result.result.outcome.disposition(), Disposition::kObjectAtSource);
  EXPECT_EQ(result.result.outcome.recovery_count(), 1U);
  EXPECT_EQ(
    result.result.outcome.resource_release(),
    ResourceReleaseDisposition::kQuarantineRequired);
  EXPECT_EQ(scene.release_count, 0);
}

TEST(ResolvedTransferWorkflow, LeaseFailureDoesNotEraseStrategyQuarantineEvidence)
{
  FakeTimeContext context;
  ConfiguredObservationProvider observations(context, {1.0});
  FakeGraspProvider grasps;
  FakeSceneWorkflow scene;
  scene.renew_failure = ErrorCode::kSceneFailed;
  TerminalStrategyFixture strategies(
    TaskOutcome::make_failure(
      ErrorCode::kExecutionFailed, Disposition::kHoldingObject, 2U,
      "accepted command remained active",
      ResourceReleaseDisposition::kQuarantineRequired),
    [&context](TransferExecutionContext & strategy_context) {
      context.now += 2s;
      EXPECT_EQ(
        strategy_context.heartbeat(),
        TransferExecutionControlResult::kStopped);
    });
  auto configured_options = options(context);
  configured_options.scene_renew_period = 1s;
  const auto object_catalog = catalog();
  ResolvedTransferWorkflow workflow(
    std::move(configured_options), object_catalog, observations, grasps, scene,
    *strategies.registry, "terminal_outcome");

  const auto result = workflow.execute(absolute_goal(), context);

  EXPECT_EQ(result.result.outcome.error_code(), ErrorCode::kSceneFailed);
  EXPECT_EQ(result.result.outcome.disposition(), Disposition::kHoldingObject);
  EXPECT_EQ(result.result.outcome.recovery_count(), 2U);
  EXPECT_EQ(
    result.result.outcome.resource_release(),
    ResourceReleaseDisposition::kQuarantineRequired);
  EXPECT_NE(result.result.outcome.detail().find("renewal"), std::string::npos);
  EXPECT_NE(result.result.outcome.detail().find("remained active"), std::string::npos);
  EXPECT_EQ(scene.renew_count, 1);
  EXPECT_EQ(scene.release_count, 0);
}

TEST(ResolvedTransferWorkflow, CheckpointSlicedWaitEmitsThrottledWaitingRecord)
{
  FakeTimeContext context;
  CheckpointSlicedObservationProvider observations(context);
  FakeGraspProvider grasps;
  FakeSceneWorkflow scene;
  StrategyFixture strategies;
  std::vector<WaitLifecycleRecord> records;
  auto configured_options = options(context);
  configured_options.wait_logger = std::make_shared<WaitLifecycleLogger>(
    [&records](WaitLifecycleRecord record) {
      records.emplace_back(std::move(record));
    },
    [&context]() {return context.now;});
  const auto object_catalog = catalog();
  ResolvedTransferWorkflow workflow(
    std::move(configured_options), object_catalog, observations, grasps, scene,
    *strategies.registry, "test_strategy");

  const auto result = workflow.execute(
    absolute_goal(), context, {}, "checkpoint-goal");

  ASSERT_TRUE(result.result.outcome.succeeded());
  EXPECT_TRUE(std::any_of(
    records.cbegin(), records.cend(), [](const auto & record) {
        return record.event == WaitLifecycleEvent::kWaiting &&
               record.scope.goal_id == "checkpoint-goal" &&
               record.scope.reason == TransferWaitingReason::kObservation;
    }));
}

TEST(ResolvedTransferWorkflow, CancellationDuringSceneStagingPreventsGraspAndStrategy)
{
  FakeTimeContext context;
  context.stop_on_wait = 1;
  ConfiguredObservationProvider observations(context, {1.0});
  FakeGraspProvider grasps;
  FakeSceneWorkflow scene;
  scene.stage_wait = 70s;
  StrategyFixture strategies;
  const auto object_catalog = catalog();
  ResolvedTransferWorkflow workflow(
    options(context), object_catalog, observations, grasps, scene,
    *strategies.registry, "test_strategy");

  const auto result = workflow.execute(absolute_goal(), context);

  EXPECT_EQ(result.result.outcome.error_code(), ErrorCode::kCanceled);
  EXPECT_EQ(scene.stage_count, 1);
  EXPECT_EQ(grasps.call_count, 0);
  EXPECT_EQ(strategies.strategy->physical_side_effect_count, 0);
  EXPECT_TRUE(result.result.outcome.resources_release_safe());
  EXPECT_EQ(scene.release_count, 1);
}

TEST(ResolvedTransferWorkflow, CancellationDuringObservationPreventsLaterPhysicalSideEffect)
{
  FakeTimeContext context;
  context.stop_on_wait = 1;
  ScriptedObservationProvider observations("cancel", context, {1.0}, 10s);
  FakeGraspProvider grasps;
  FakeSceneWorkflow scene;
  StrategyFixture strategies;
  const auto object_catalog = catalog();
  ResolvedTransferWorkflow workflow(
    options(context), object_catalog, observations, grasps, scene, *strategies.registry,
    "test_strategy");

  const auto result = workflow.execute(absolute_goal(), context);

  EXPECT_FALSE(result.result.outcome.succeeded());
  EXPECT_EQ(result.result.outcome.error_code(), ErrorCode::kCanceled);
  EXPECT_EQ(result.result.failed_step_index, 0U);
  EXPECT_EQ(grasps.call_count, 0);
  EXPECT_EQ(strategies.strategy->physical_side_effect_count, 0);
  EXPECT_TRUE(result.result.outcome.resources_release_safe());
  EXPECT_EQ(scene.release_count, 1);
}

TEST(ResolvedTransferWorkflow, TestStrategyChangesStagesAndUnknownSelectionFailsStartup)
{
  FakeTimeContext context;
  ConfiguredObservationProvider observations(context, {1.0});
  FakeGraspProvider grasps;
  FakeSceneWorkflow scene;
  StrategyFixture strategies(
    {Phase::kApproaching, Phase::kLifting},
    TransferWaitingReason::kHealthReadiness);
  const auto object_catalog = catalog();
  ResolvedTransferWorkflow workflow(
    options(context), object_catalog, observations, grasps, scene, *strategies.registry,
    "test_strategy");
  std::vector<TransferSequenceFeedbackSnapshot> feedback;

  const auto result = workflow.execute(
    absolute_goal(), context,
    [&feedback](TransferSequenceFeedbackSnapshot value) {
      feedback.emplace_back(std::move(value));
    });

  ASSERT_TRUE(result.result.outcome.succeeded());
  EXPECT_TRUE(std::any_of(
    feedback.cbegin(), feedback.cend(), [](const auto & value) {
        return value.current_state.phase == Phase::kApproaching &&
               value.waiting_reason == TransferWaitingReason::kHealthReadiness;
    }));
  EXPECT_TRUE(std::any_of(
    feedback.cbegin(), feedback.cend(), [](const auto & value) {
        return value.current_state.phase == Phase::kLifting &&
               value.waiting_reason == TransferWaitingReason::kNone;
    }));

  EXPECT_THROW(
    ResolvedTransferWorkflow(
      options(context), object_catalog, observations, grasps, scene,
      *strategies.registry, "unknown_strategy"),
    std::invalid_argument);

  workflow.request_stop();
  EXPECT_TRUE(observations.stop_forwarded);
  EXPECT_TRUE(grasps.stop_forwarded);
  EXPECT_TRUE(strategies.strategy->stop_forwarded);
  EXPECT_TRUE(scene.stop_forwarded);
}

TEST(ResolvedTransferWorkflow, ProviderExceptionPreservesCompletedPrefix)
{
  FakeTimeContext context;
  // The second observation throws out_of_range, like a failed provider callback.
  ConfiguredObservationProvider observations(context, {1.0});
  FakeGraspProvider grasps;
  FakeSceneWorkflow scene;
  StrategyFixture strategies;
  const auto object_catalog = catalog();
  ResolvedTransferWorkflow workflow(
    options(context), object_catalog, observations, grasps, scene,
    *strategies.registry, "test_strategy");

  const auto result = workflow.execute(translated_return_goal(), context);

  EXPECT_EQ(result.result.outcome.error_code(), ErrorCode::kInternalError);
  EXPECT_EQ(result.result.completed_step_count, 1U);
  EXPECT_EQ(result.result.failed_step_index, 1U);
  ASSERT_EQ(result.result.step_outcomes.size(), 2U);
  ASSERT_EQ(result.step_elapsed.size(), 2U);
  EXPECT_TRUE(result.result.step_outcomes.front().succeeded());
  EXPECT_EQ(result.result.step_outcomes.front().disposition(), Disposition::kObjectAtTarget);
  EXPECT_FALSE(result.result.step_outcomes.back().succeeded());
  EXPECT_EQ(strategies.strategy->call_count, 1);
  EXPECT_EQ(scene.release_count, 0);
}

TEST(ResolvedTransferWorkflow, ReadbackExceptionPreservesConfirmedStrategyOutcome)
{
  FakeTimeContext context;
  ConfiguredObservationProvider observations(context, {1.0});
  FakeGraspProvider grasps;
  FakeSceneWorkflow scene;
  scene.on_revalidate = [&scene]() {
      if (scene.revalidate_count == 2) {
        throw std::runtime_error("readback unavailable after completed motion");
      }
    };
  TerminalStrategyFixture strategies(TaskOutcome::make_success(
      Disposition::kObjectAtTarget, 2U, "confirmed transfer after recovery"));
  const auto object_catalog = catalog();
  ResolvedTransferWorkflow workflow(
    options(context), object_catalog, observations, grasps, scene,
    *strategies.registry, "terminal_outcome");

  const auto result = workflow.execute(absolute_goal(), context);

  EXPECT_EQ(result.result.outcome.error_code(), ErrorCode::kInternalError);
  EXPECT_EQ(result.result.outcome.disposition(), Disposition::kObjectAtTarget);
  EXPECT_EQ(result.result.outcome.recovery_count(), 2U);
  EXPECT_TRUE(result.result.outcome.resources_release_safe());
  EXPECT_EQ(result.result.failed_step_index, 0U);
  ASSERT_EQ(result.result.step_outcomes.size(), 1U);
  EXPECT_EQ(result.result.step_outcomes.front().disposition(), Disposition::kObjectAtTarget);
  EXPECT_EQ(scene.release_count, 0);
}

TEST(ResolvedTransferWorkflow, StrategyExceptionQuarantinesUnprovenPhysicalCompletion)
{
  FakeTimeContext context;
  ConfiguredObservationProvider observations(context, {1.0});
  FakeGraspProvider grasps;
  FakeSceneWorkflow scene;
  TerminalStrategyFixture strategies(
    TaskOutcome::make_success(Disposition::kObjectAtTarget),
    [](TransferExecutionContext &) {throw std::runtime_error("controller callback failed");});
  const auto object_catalog = catalog();
  ResolvedTransferWorkflow workflow(
    options(context), object_catalog, observations, grasps, scene,
    *strategies.registry, "terminal_outcome");

  const auto result = workflow.execute(absolute_goal(), context);

  EXPECT_EQ(result.result.outcome.error_code(), ErrorCode::kInternalError);
  EXPECT_EQ(
    result.result.outcome.resource_release(), ResourceReleaseDisposition::kQuarantineRequired);
  EXPECT_EQ(result.result.failed_step_index, 0U);
  EXPECT_TRUE(strategies.strategy->stop_forwarded);
  EXPECT_EQ(scene.release_count, 0);
}

TEST(ResolvedTransferWorkflow, CancellationClearsAcknowledgedPauseBeforeDispatch)
{
  FakeTimeContext context;
  context.paused_operation_attempts = 2;
  context.stop_on_wait = 1;
  ConfiguredObservationProvider observations(context, {1.0});
  FakeGraspProvider grasps;
  FakeSceneWorkflow scene;
  StrategyFixture strategies;
  const auto object_catalog = catalog();
  ResolvedTransferWorkflow workflow(
    options(context), object_catalog, observations, grasps, scene,
    *strategies.registry, "test_strategy");
  std::vector<TransferSequenceFeedbackSnapshot> feedback;

  const auto result = workflow.execute(
    absolute_goal(), context, [&feedback](TransferSequenceFeedbackSnapshot state) {
      feedback.emplace_back(std::move(state));
    });

  EXPECT_EQ(result.result.outcome.error_code(), ErrorCode::kCanceled);
  EXPECT_TRUE(std::any_of(feedback.cbegin(), feedback.cend(), [](const auto & state) {
      return state.current_state.paused;
    }));
  ASSERT_FALSE(feedback.empty());
  EXPECT_FALSE(feedback.back().current_state.paused);
  EXPECT_EQ(strategies.strategy->physical_side_effect_count, 0);
  EXPECT_EQ(scene.release_count, 1);
}

TEST(ResolvedTransferWorkflow, PauseAtNextStepDoesNotReuseCompletedStepProgress)
{
  FakeTimeContext context;
  context.paused_operation_attempts = 1;
  context.pause_on_or_after_operation_call = 2;
  ConfiguredObservationProvider observations(context, {1.0, 3.0});
  FakeGraspProvider grasps;
  FakeSceneWorkflow scene;
  StrategyFixture strategies;
  const auto object_catalog = catalog();
  ResolvedTransferWorkflow workflow(
    options(context), object_catalog, observations, grasps, scene,
    *strategies.registry, "test_strategy");
  std::vector<TransferSequenceFeedbackSnapshot> feedback;

  const auto result = workflow.execute(
    translated_return_goal(), context, [&feedback](TransferSequenceFeedbackSnapshot state) {
      feedback.emplace_back(std::move(state));
    });

  ASSERT_TRUE(result.result.outcome.succeeded());
  const auto paused = std::find_if(feedback.cbegin(), feedback.cend(), [](const auto & state) {
        return state.current_state.paused;
    });
  ASSERT_NE(paused, feedback.cend());
  EXPECT_EQ(paused->step_index, 1U);
  EXPECT_NE(paused->current_state.phase, Phase::kCompleted);
  EXPECT_FLOAT_EQ(paused->current_state.progress, 0.0F);
  EXPECT_FLOAT_EQ(paused->overall_progress, 0.5F);
}

TEST(ResolvedTransferWorkflowActionAdapter, AcknowledgesSafePauseWhileRenewingLease)
{
  FakeTimeContext clock;
  ConfiguredObservationProvider observations(clock, {1.0});
  FakeGraspProvider grasps;
  FakeSceneWorkflow scene;
  auto strategy = std::make_unique<GatedDispatchStrategy>();
  auto * dispatch_strategy = strategy.get();
  std::vector<std::unique_ptr<TransferExecutionStrategy>> entries;
  entries.emplace_back(std::move(strategy));
  auto strategies = StrategyRegistry::for_testing(std::move(entries));
  const auto object_catalog = catalog();
  auto workflow_options = options(clock);
  workflow_options.now = []() {return Clock::now();};
  workflow_options.scene_renew_period = 1ms;
  ResolvedTransferWorkflow workflow(
    std::move(workflow_options), object_catalog, observations, grasps, scene,
    strategies, "gated_dispatch");
  manipulation::task::ExecutionControl control;
  detail::TransferSequenceSession session;
  ASSERT_TRUE(session.mark_worker_starting());
  ASSERT_TRUE(session.mark_worker_running());
  ASSERT_TRUE(session.mark_backend_invoked());
  bool pause_requested = false;
  bool paused_during_renewal = false;
  std::vector<TransferSequenceFeedbackSnapshot> feedback;
  scene.on_renew = [&]() {
      if (pause_requested && session.pause_requested()) {
        EXPECT_EQ(dispatch_strategy->dispatch_count, 0);
        paused_during_renewal = paused_during_renewal ||
          (!feedback.empty() && feedback.back().current_state.paused);
        if (scene.renew_count >= 2) {
          control.resume();
          EXPECT_TRUE(session.set_pause_requested(false));
        }
      }
    };
  auto task_context = detail::TransferSequenceTaskContextFactory::create(
    control, Clock::time_point::max(),
    [&](TransferSequenceFeedbackSnapshot state) {
      feedback.emplace_back(std::move(state));
      if (!pause_requested && scene.acquire_count == 1) {
        pause_requested = true;
        control.request_pause();
        EXPECT_TRUE(session.set_pause_requested(true));
      }
    },
    [&session]() {return session.stop_cause();}, []() {},
    [&session](const TransferSequenceTaskContext::Operation & operation) {
      return session.run_operation(Clock::time_point::max(), operation);
    },
    [&session](const TransferSequenceTaskContext::Operation & operation) {
      return session.run_maintenance_operation(Clock::time_point::max(), operation);
    });

  const auto callbacks = make_resolved_transfer_workflow_callbacks(workflow);
  const auto result = callbacks.execute(absolute_goal(), task_context);

  ASSERT_TRUE(result.result.outcome.succeeded());
  EXPECT_TRUE(pause_requested);
  EXPECT_TRUE(paused_during_renewal);
  EXPECT_GE(scene.renew_count, 2);
  EXPECT_EQ(dispatch_strategy->dispatch_count, 1);
  const auto paused = std::find_if(feedback.cbegin(), feedback.cend(), [](const auto & state) {
        return state.current_state.paused && !state.current_state.pause_pending;
    });
  ASSERT_NE(paused, feedback.cend());
  EXPECT_TRUE(std::any_of(paused + 1, feedback.cend(), [](const auto & state) {
      return !state.current_state.paused && !state.current_state.pause_pending;
    }));
  EXPECT_FALSE(feedback.back().current_state.paused);
}

TEST(ResolvedTransferWorkflowActionAdapter, MapsWaitStopAndSuppressesDuplicateBasicFeedback)
{
  manipulation::task::ExecutionControl control;
  std::vector<TransferSequenceFeedbackSnapshot> typed_feedback;
  auto gate_result = TaskOperationGateResult::kPaused;
  int operation_count = 0;
  auto task_context = detail::TransferSequenceTaskContextFactory::create(
    control, TransferSequenceTaskContext::TimePoint::max(),
    [&typed_feedback](TransferSequenceFeedbackSnapshot feedback) {
      typed_feedback.emplace_back(std::move(feedback));
    },
    [&control]() {
      return control.cancellation_requested() ? TaskStopCause::kClientCanceled :
             TaskStopCause::kNone;
    },
    []() {},
    [&gate_result](const TransferSequenceTaskContext::Operation & operation) {
      if (gate_result == TaskOperationGateResult::kExecuted) {
        operation();
      }
      return gate_result;
    },
    [](const TransferSequenceTaskContext::Operation & operation) {
      operation();
      return TaskOperationGateResult::kExecuted;
    });
  TransferSequenceExecutionContextAdapter adapter(task_context);

  EXPECT_EQ(adapter.heartbeat(), TransferExecutionControlResult::kContinue);
  EXPECT_EQ(adapter.stop_cause(), TaskStopCause::kNone);
  EXPECT_EQ(
    adapter.run_operation([&operation_count]() {++operation_count;}),
    TaskOperationGateResult::kPaused);
  EXPECT_EQ(operation_count, 0);
  gate_result = TaskOperationGateResult::kExecuted;
  EXPECT_EQ(
    adapter.run_operation([&operation_count]() {++operation_count;}),
    TaskOperationGateResult::kExecuted);
  EXPECT_EQ(operation_count, 1);
  EXPECT_EQ(
    adapter.run_maintenance_operation([&operation_count]() {++operation_count;}),
    TaskOperationGateResult::kExecuted);
  EXPECT_EQ(operation_count, 2);
  EXPECT_EQ(
    adapter.wait_until(TransferSequenceExecutionContextAdapter::Clock::now()),
    TransferExecutionControlResult::kContinue);
  manipulation::task::TaskStateSnapshot basic;
  basic.phase = Phase::kPlanning;
  adapter.report_feedback(std::move(basic));
  EXPECT_TRUE(typed_feedback.empty());

  control.request_cancel();
  EXPECT_TRUE(adapter.stop_requested());
  EXPECT_EQ(adapter.stop_cause(), TaskStopCause::kClientCanceled);
  EXPECT_EQ(adapter.heartbeat(), TransferExecutionControlResult::kStopped);
  EXPECT_EQ(adapter.checkpoint(), TransferExecutionControlResult::kStopped);
  EXPECT_EQ(
    adapter.wait_until(TransferSequenceExecutionContextAdapter::Clock::now()),
    TransferExecutionControlResult::kStopped);
}

}  // namespace
}  // namespace manipulation::workflow
