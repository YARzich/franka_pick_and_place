// Responsibility: Resolves and executes V2 transfer intent through explicit goal-scoped ports.

#include "manipulation/workflow/resolved_transfer_workflow.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "manipulation/workflow/scene_lease_release_policy.hpp"
#include "detail/transfer_target_resolution.hpp"
#include "manipulation_msgs/msg/transfer_object_step_v2.hpp"

namespace manipulation::workflow
{
namespace
{

using Clock = ResolvedTransferWorkflowOptions::Clock;
using Step = manipulation_msgs::msg::TransferObjectStepV2;
using ErrorCode = manipulation::task::TaskErrorCode;
using Disposition = manipulation::task::ObjectDisposition;
using ResourceReleaseDisposition = manipulation::task::ResourceReleaseDisposition;

struct WorkflowFailure
{
  ErrorCode code{ErrorCode::kInternalError};
  std::string detail;
};

manipulation::task::TaskOutcome make_failure(
  const WorkflowFailure & failure,
  const Disposition disposition = Disposition::kUnknown,
  const std::uint16_t recovery_count = 0U,
  const ResourceReleaseDisposition resource_release =
  ResourceReleaseDisposition::kReleaseSafe)
{
  return manipulation::task::TaskOutcome::make_failure(
    failure.code == ErrorCode::kNone ? ErrorCode::kInternalError : failure.code,
    disposition, recovery_count, failure.detail, resource_release);
}

manipulation::task::TaskOutcome make_failure_preserving_evidence(
  const WorkflowFailure & failure,
  const manipulation::task::TaskOutcome & evidence)
{
  auto detail = failure.detail;
  if (!evidence.detail().empty()) {
    if (!detail.empty()) {
      detail += "; ";
    }
    detail += evidence.detail();
  }
  return make_failure(
    {failure.code, std::move(detail)}, evidence.disposition(),
    evidence.recovery_count(), evidence.resource_release());
}

Disposition failure_disposition(const ErrorCode code) noexcept
{
  return code == ErrorCode::kSceneConflict || code == ErrorCode::kSceneFailed ?
         Disposition::kSceneUncertain : Disposition::kUnknown;
}

WorkflowFailure stopped_or(
  const TransferExecutionContext & context,
  WorkflowFailure failure)
{
  if (context.stop_requested()) {
    return {ErrorCode::kCanceled, "transfer workflow was stopped"};
  }
  return failure;
}

WaitLifecycleOutcome stopped_wait_outcome(
  const TransferExecutionContext & context) noexcept
{
  switch (context.stop_cause()) {
    case TaskStopCause::kClientCanceled:
      return WaitLifecycleOutcome::kCanceled;
    case TaskStopCause::kShutdown:
      return WaitLifecycleOutcome::kShutdown;
    case TaskStopCause::kDeadlineExceeded:
    case TaskStopCause::kNone:
      return WaitLifecycleOutcome::kError;
  }
  return WaitLifecycleOutcome::kError;
}

template<typename ResultT>
WorkflowFailure result_failure(const ResultT & result, const std::string & fallback)
{
  return {
    result.error_code() == ErrorCode::kNone ? ErrorCode::kInternalError : result.error_code(),
    result.detail().empty() ? fallback : result.detail()};
}

TransferSequenceExecution no_step_failure(const WorkflowFailure & failure)
{
  auto outcome = make_failure(failure);
  return {{outcome, 0U, std::nullopt, {}}, {}};
}

TransferRecoveryPolicy recovery_policy(const std::uint8_t value)
{
  switch (value) {
    case Step::RECOVERY_NONE:
      return TransferRecoveryPolicy::kNone;
    case Step::RECOVERY_SAFE_STOP:
      return TransferRecoveryPolicy::kSafeStop;
    case Step::RECOVERY_RETREAT:
      return TransferRecoveryPolicy::kRetreat;
    case Step::RECOVERY_RETREAT_AND_HOME:
      return TransferRecoveryPolicy::kRetreatAndHome;
    default:
      throw std::invalid_argument("validated V2 step contains an unknown recovery policy");
  }
}

void report_sequence_feedback(
  TransferExecutionContext & context,
  const ResolvedTransferFeedbackCallback & callback,
  const std::size_t step_index,
  const std::size_t step_count,
  const std::string & object_id,
  const TransferWaitingReason waiting_reason,
  manipulation::task::TaskStateSnapshot state)
{
  const float local_progress = std::clamp(state.progress, 0.0F, 1.0F);
  const float overall_progress = step_count == 0U ? 0.0F :
    std::clamp(
    (static_cast<float>(step_index) + local_progress) /
    static_cast<float>(step_count), 0.0F, 1.0F);
  context.report_feedback(state);
  if (callback) {
    callback({
          step_index, step_count, overall_progress, std::move(state), object_id,
          waiting_reason});
  }
}

/** Keep lifecycle logging observable while the goal-scoped lease does not yet exist. */
class WaitLoggingExecutionContext final : public TransferExecutionContext
{
public:
  WaitLoggingExecutionContext(
    TransferExecutionContext & downstream,
    const std::shared_ptr<WaitLifecycleLogger> & logger) noexcept
  : downstream_(downstream), logger_(logger)
  {
  }

  [[nodiscard]] TransferExecutionControlResult checkpoint() override
  {
    tick();
    return downstream_.checkpoint();
  }

  [[nodiscard]] bool stop_requested() const noexcept override
  {
    return downstream_.stop_requested();
  }

  [[nodiscard]] TransferExecutionControlResult wait_until(const TimePoint deadline) override
  {
    tick();
    const auto result = downstream_.wait_until(deadline);
    tick();
    return result;
  }

  [[nodiscard]] TransferExecutionControlResult heartbeat() override
  {
    tick();
    return downstream_.heartbeat();
  }

  [[nodiscard]] TaskOperationGateResult run_operation(
    const std::function<void()> & operation) override
  {
    tick();
    return downstream_.run_operation(operation);
  }

  [[nodiscard]] TaskOperationGateResult run_maintenance_operation(
    const std::function<void()> & operation) override
  {
    tick();
    return downstream_.run_maintenance_operation(operation);
  }

  [[nodiscard]] TaskStopCause stop_cause() const noexcept override
  {
    return downstream_.stop_cause();
  }

  void report_feedback(manipulation::task::TaskStateSnapshot feedback) override
  {
    downstream_.report_feedback(std::move(feedback));
  }

  void report_waiting(
    const TransferWaitingReason reason,
    manipulation::task::TaskStateSnapshot feedback) override
  {
    tick();
    downstream_.report_waiting(reason, std::move(feedback));
  }

private:
  void tick()
  {
    if (logger_) {
      logger_->waiting();
    }
  }

  TransferExecutionContext & downstream_;
  std::shared_ptr<WaitLifecycleLogger> logger_;
};

class GoalSceneLeaseGuard final
{
public:
  GoalSceneLeaseGuard(
    TransferSceneWorkflowPort & port,
    const PlanningSceneLease & lease)
  : port_(port), snapshot_(lease.snapshot()), proof_(lease.access_proof())
  {
  }

  ~GoalSceneLeaseGuard()
  {
    if (release_permitted_) {
      port_.release(proof_);
    }
  }

  GoalSceneLeaseGuard(const GoalSceneLeaseGuard &) = delete;
  GoalSceneLeaseGuard & operator=(const GoalSceneLeaseGuard &) = delete;

  [[nodiscard]] const PlanningSceneSnapshot & snapshot() const noexcept
  {
    return snapshot_;
  }

  [[nodiscard]] const PlanningSceneAccessProof & proof() const noexcept
  {
    return proof_;
  }

  void mark_strategy_started() noexcept
  {
    strategy_started_ = true;
  }

  void mark_state_uncertain() noexcept
  {
    state_uncertain_ = true;
    release_permitted_ = false;
  }

  [[nodiscard]] bool state_uncertain() const noexcept
  {
    return state_uncertain_;
  }

  /**
   * Permit destructor cleanup only after complete terminal evidence proves it.
   *
   * Before strategy dispatch, an ordinary failure/cancel is known to have no
   * physical side effect and may release despite its naturally unknown object
   * disposition. Once strategy execution starts, the shared release policy
   * owns both object-disposition and physical-command quarantine decisions.
   */
  void finalize(const manipulation::task::TaskOutcome & outcome) noexcept
  {
    if (state_uncertain_) {
      return;
    }
    if (!strategy_started_) {
      release_permitted_ = outcome.resources_release_safe() &&
        outcome.disposition() != Disposition::kHoldingObject &&
        outcome.disposition() != Disposition::kSceneUncertain;
      return;
    }
    release_permitted_ =
      scene_lease_release_decision(outcome) == SceneLeaseReleaseDecision::kRelease;
  }

  [[nodiscard]] std::optional<WorkflowFailure> renew(TransferExecutionContext & context)
  {
    const auto result = port_.renew(proof_, context);
    if (!result.succeeded()) {
      if (result.error_code() == ErrorCode::kSceneFailed) {
        mark_state_uncertain();
      }
      return result_failure(result, "planning scene lease renewal failed");
    }
    const auto & renewed = *result.value();
    if (renewed.lease_id() != proof_.lease_id() ||
      renewed.coordinator_session_id() != proof_.coordinator_session_id() ||
      renewed.revision() != proof_.revision())
    {
      mark_state_uncertain();
      return WorkflowFailure{
        ErrorCode::kSceneConflict,
        "planning scene lease renewal changed identity, session, or revision"};
    }
    proof_ = renewed;
    return std::nullopt;
  }

  [[nodiscard]] std::optional<WorkflowFailure> revalidate(
    TransferExecutionContext & context)
  {
    const auto result = port_.revalidate(proof_, context);
    if (!result.succeeded()) {
      if (result.error_code() == ErrorCode::kSceneFailed) {
        mark_state_uncertain();
      }
      return result_failure(result, "planning scene lease revalidation failed");
    }
    const auto & lease = *result.value();
    if (lease.access_proof().lease_id() != proof_.lease_id() ||
      lease.access_proof().coordinator_session_id() != proof_.coordinator_session_id())
    {
      mark_state_uncertain();
      return WorkflowFailure{
        ErrorCode::kSceneConflict,
        "planning scene lease revalidation changed lease or coordinator session"};
    }
    snapshot_ = lease.snapshot();
    proof_ = lease.access_proof();
    return std::nullopt;
  }

  [[nodiscard]] std::optional<WorkflowFailure> stage_observation(
    const ObjectInstance & instance,
    const ObjectProfile & profile,
    const ObjectObservation & observation,
    TransferExecutionContext & context)
  {
    const auto result = port_.stage_observation(
      instance, profile, observation, proof_, context);
    if (!result.succeeded()) {
      if (result.error_code() == ErrorCode::kSceneFailed) {
        mark_state_uncertain();
      }
      return result_failure(result, "observed object scene staging failed");
    }
    const auto & lease = *result.value();
    if (lease.access_proof().lease_id() != proof_.lease_id() ||
      lease.access_proof().coordinator_session_id() != proof_.coordinator_session_id())
    {
      mark_state_uncertain();
      return WorkflowFailure{
        ErrorCode::kSceneConflict,
        "object staging changed planning scene lease or coordinator session"};
    }
    snapshot_ = lease.snapshot();
    proof_ = lease.access_proof();
    return std::nullopt;
  }

private:
  TransferSceneWorkflowPort & port_;
  PlanningSceneSnapshot snapshot_;
  PlanningSceneAccessProof proof_;
  bool strategy_started_{false};
  bool state_uncertain_{false};
  bool release_permitted_{false};
};

class LeaseAwareExecutionContext final : public TransferExecutionContext
{
public:
  LeaseAwareExecutionContext(
    TransferExecutionContext & downstream,
    GoalSceneLeaseGuard & lease,
    Clock::duration renew_period,
    ResolvedTransferWorkflowOptions::NowFunction now,
    ResolvedTransferFeedbackCallback feedback,
    const std::size_t step_count,
    std::shared_ptr<WaitLifecycleLogger> wait_logger,
    std::string goal_id)
  : downstream_(downstream), lease_(lease), renew_period_(renew_period), now_(std::move(now)),
    feedback_(std::move(feedback)), step_count_(step_count),
    wait_logger_(std::move(wait_logger)), goal_id_(std::move(goal_id)),
    next_renew_(now_() + renew_period_)
  {
  }

  ~LeaseAwareExecutionContext() override
  {
    if (waiting_reason_ == TransferWaitingReason::kNone) {
      return;
    }
    if (failure_) {
      close_wait(WaitLifecycleOutcome::kError, failure_->detail);
    } else if (downstream_.stop_requested()) {
      close_wait(stopped_wait_outcome(downstream_));
    } else {
      close_wait(WaitLifecycleOutcome::kError);
    }
  }

  void set_active_step(
    const std::size_t step_index,
    std::string object_id)
  {
    close_wait(WaitLifecycleOutcome::kError);
    step_index_ = step_index;
    object_id_ = std::move(object_id);
    last_feedback_ = {};
    last_feedback_.phase = manipulation::task::TaskPhase::kWaitingForResources;
    last_feedback_.detail = "waiting at transfer step boundary";
  }

  void set_waiting(
    const TransferWaitingReason waiting_reason,
    std::string diagnostic = {})
  {
    if (waiting_reason == waiting_reason_) {
      if (!diagnostic.empty()) {
        waiting_diagnostic_ = std::move(diagnostic);
      }
      return;
    }
    close_wait(WaitLifecycleOutcome::kReady);
    waiting_reason_ = waiting_reason;
    waiting_diagnostic_ = std::move(diagnostic);
    if (wait_logger_ && waiting_reason_ != TransferWaitingReason::kNone) {
      wait_logger_->enter({
            goal_id_, step_index_, step_count_, object_id_, waiting_reason_,
            waiting_diagnostic_});
    }
  }

  [[nodiscard]] TransferExecutionControlResult checkpoint() override
  {
    const auto result = run_operation([]() {});
    return result == TaskOperationGateResult::kExecuted ?
           TransferExecutionControlResult::kContinue :
           TransferExecutionControlResult::kStopped;
  }

  [[nodiscard]] bool stop_requested() const noexcept override
  {
    return downstream_.stop_requested() || failure_.has_value();
  }

  [[nodiscard]] TaskStopCause stop_cause() const noexcept override
  {
    return downstream_.stop_cause();
  }

  [[nodiscard]] TransferExecutionControlResult heartbeat() override
  {
    tick_wait_logger();
    if (downstream_.heartbeat() == TransferExecutionControlResult::kStopped) {
      acknowledge_pause(false);
      close_wait(stopped_wait_outcome(downstream_));
      return TransferExecutionControlResult::kStopped;
    }
    if (renew_if_due()) {
      acknowledge_pause(false);
      close_wait(WaitLifecycleOutcome::kError, failure_->detail);
      return TransferExecutionControlResult::kStopped;
    }
    return TransferExecutionControlResult::kContinue;
  }

  [[nodiscard]] TaskOperationGateResult run_operation(
    const std::function<void()> & operation) override
  {
    if (!operation) {
      throw std::invalid_argument("lease-aware transfer operation must not be empty");
    }
    while (true) {
      if (heartbeat() == TransferExecutionControlResult::kStopped) {
        return TaskOperationGateResult::kStopped;
      }
      const auto result = [&]() {
          try {
            return downstream_.run_operation(operation);
          } catch (...) {
            acknowledge_pause(false);
            throw;
          }
        }();
      if (result == TaskOperationGateResult::kExecuted) {
        acknowledge_pause(false);
        return result;
      }
      if (result == TaskOperationGateResult::kStopped) {
        acknowledge_pause(false);
        close_wait(stopped_wait_outcome(downstream_));
        return result;
      }

      // The operation gate has reached a safe boundary without dispatching.
      // Publish the acknowledgement here: ordinary waits deliberately do not
      // enter ExecutionControl::checkpoint(), so renewal can continue.
      acknowledge_pause(true);
      const auto now = now_();
      const auto maximum_duration = TimePoint::max() - now;
      constexpr auto control_heartbeat = std::chrono::milliseconds{100};
      const auto control_deadline =
        control_heartbeat >= maximum_duration ? TimePoint::max() :
        now + control_heartbeat;
      const auto wake_time = std::min(control_deadline, next_renew_);
      if (downstream_.wait_until(wake_time) == TransferExecutionControlResult::kStopped) {
        acknowledge_pause(false);
        close_wait(stopped_wait_outcome(downstream_));
        return TaskOperationGateResult::kStopped;
      }
    }
  }

  [[nodiscard]] TaskOperationGateResult run_maintenance_operation(
    const std::function<void()> & operation) override
  {
    tick_wait_logger();
    return downstream_.run_maintenance_operation(operation);
  }

  [[nodiscard]] TransferExecutionControlResult wait_until(const TimePoint deadline) override
  {
    while (now_() < deadline) {
      if (heartbeat() == TransferExecutionControlResult::kStopped) {
        return TransferExecutionControlResult::kStopped;
      }
      const auto wake_time = std::min(deadline, next_renew_);
      if (downstream_.wait_until(wake_time) == TransferExecutionControlResult::kStopped) {
        close_wait(stopped_wait_outcome(downstream_));
        return TransferExecutionControlResult::kStopped;
      }
    }
    const auto gate_result = run_operation([]() {});
    return gate_result == TaskOperationGateResult::kExecuted ?
           TransferExecutionControlResult::kContinue :
           TransferExecutionControlResult::kStopped;
  }

  void report_feedback(manipulation::task::TaskStateSnapshot state) override
  {
    state.progress = std::max(state.progress, last_feedback_.progress);
    last_feedback_ = state;
    state.paused = state.paused || pause_acknowledged_;
    if (state.paused) {
      state.pause_pending = false;
    }
    report_sequence_feedback(
      downstream_, feedback_, step_index_, step_count_, object_id_, waiting_reason_,
      std::move(state));
  }

  void report_waiting(
    const TransferWaitingReason reason,
    manipulation::task::TaskStateSnapshot state) override
  {
    set_waiting(reason, state.detail);
    if (state.step_count == 0U) {
      // Transport-only waits do not redefine the strategy's reached checkpoints.
      state.step_index = last_feedback_.step_index;
      state.step_count = last_feedback_.step_count;
      state.retry_count = last_feedback_.retry_count;
      state.recovery_count = last_feedback_.recovery_count;
    }
    report_feedback(std::move(state));
  }

  [[nodiscard]] const std::optional<WorkflowFailure> & failure() const noexcept
  {
    return failure_;
  }

  [[nodiscard]] const manipulation::task::TaskStateSnapshot & current_state() const noexcept
  {
    return last_feedback_;
  }

private:
  void acknowledge_pause(const bool paused)
  {
    if (pause_acknowledged_ == paused) {
      return;
    }
    pause_acknowledged_ = paused;
    report_feedback(last_feedback_);
  }

  void tick_wait_logger()
  {
    if (wait_logger_) {
      wait_logger_->waiting();
    }
  }

  void close_wait(
    const WaitLifecycleOutcome outcome,
    std::string diagnostic = {})
  {
    if (wait_logger_ && waiting_reason_ != TransferWaitingReason::kNone) {
      if (diagnostic.empty()) {
        diagnostic = waiting_diagnostic_;
      }
      wait_logger_->exit(outcome, std::move(diagnostic));
    }
    waiting_reason_ = TransferWaitingReason::kNone;
    waiting_diagnostic_.clear();
  }

  [[nodiscard]] bool renew_if_due()
  {
    if (failure_ || now_() < next_renew_) {
      return failure_.has_value();
    }
    failure_ = lease_.renew(downstream_);
    next_renew_ = now_() + renew_period_;
    return failure_.has_value();
  }

  TransferExecutionContext & downstream_;
  GoalSceneLeaseGuard & lease_;
  Clock::duration renew_period_;
  ResolvedTransferWorkflowOptions::NowFunction now_;
  ResolvedTransferFeedbackCallback feedback_;
  std::size_t step_count_{0U};
  std::size_t step_index_{0U};
  std::string object_id_;
  TransferWaitingReason waiting_reason_{TransferWaitingReason::kNone};
  std::string waiting_diagnostic_;
  std::shared_ptr<WaitLifecycleLogger> wait_logger_;
  std::string goal_id_;
  TimePoint next_renew_;
  std::optional<WorkflowFailure> failure_;
  manipulation::task::TaskStateSnapshot last_feedback_;
  bool pause_acknowledged_{false};
};

std::vector<ObjectInstance> unique_goal_instances(
  const ResolvedTransferWorkflow::Goal & goal,
  const ObjectCatalog & catalog)
{
  std::unordered_set<std::string> seen;
  std::vector<ObjectInstance> instances;
  instances.reserve(goal.steps.size());
  for (const auto & step : goal.steps) {
    if (seen.insert(step.object_id).second) {
      instances.emplace_back(catalog.require_instance(step.object_id));
    }
  }
  return instances;
}

}  // namespace

class ResolvedTransferWorkflow::Impl final
{
public:
  Impl(
    ResolvedTransferWorkflowOptions options,
    const ObjectCatalog & catalog,
    ObservationProvider & observation_provider,
    GraspProvider & grasp_provider,
    TransferSceneWorkflowPort & scene,
    StrategyRegistry & strategies,
    const std::string_view selected_strategy_id)
  : options_(std::move(options)), catalog_(catalog), observation_provider_(observation_provider),
    grasp_provider_(grasp_provider), scene_(scene),
    strategy_(strategies.require(selected_strategy_id))
  {
    if (options_.scene_renew_period <= Clock::duration::zero()) {
      throw std::invalid_argument("scene_renew_period must be positive");
    }
    if (!options_.now) {
      throw std::invalid_argument("resolved transfer workflow requires a clock function");
    }
  }

  [[nodiscard]] TransferSequenceAdmission admit(const Goal & goal) const
  {
    const auto validation = validate_transfer_object_sequence_v2_goal(
      goal, options_.validation);
    if (!validation.valid) {
      return {false, Disposition::kUnknown, validation.reason};
    }
    for (const auto & step : goal.steps) {
      const auto * instance = catalog_.find_instance(step.object_id);
      if (!instance) {
        return {
          false, Disposition::kUnknown,
          "object catalog has no physical instance '" + step.object_id + "'"};
      }
      if (!catalog_.find_profile(instance->profile_id())) {
        return {
          false, Disposition::kUnknown,
          "object catalog has no bound profile for '" + step.object_id + "'"};
      }
    }
    return {true, Disposition::kUnknown, {}};
  }

  [[nodiscard]] TransferSequenceExecution execute(
    const Goal & goal,
    TransferExecutionContext & context,
    ResolvedTransferFeedbackCallback feedback,
    std::string goal_id)
  {
    const auto admission = admit(goal);
    if (!admission.accepted) {
      return no_step_failure({ErrorCode::kInvalidGoal, admission.reason});
    }
    if (context.checkpoint() == TransferExecutionControlResult::kStopped) {
      return no_step_failure({ErrorCode::kCanceled,
                 "transfer workflow was stopped before acquire"});
    }

    manipulation::task::TaskStateSnapshot acquiring;
    acquiring.phase = manipulation::task::TaskPhase::kWaitingForResources;
    acquiring.detail = "acquiring goal-scoped planning scene lease";
    const auto & acquiring_object = goal.steps.front().object_id;
    if (options_.wait_logger) {
      options_.wait_logger->enter({
          goal_id, 0U, goal.steps.size(), acquiring_object,
          TransferWaitingReason::kSceneReadiness,
          acquiring.detail});
    }
    report_sequence_feedback(
      context, feedback, 0U, goal.steps.size(), acquiring_object,
      TransferWaitingReason::kSceneReadiness, std::move(acquiring));

    WaitLoggingExecutionContext acquire_context(context, options_.wait_logger);
    const auto acquired = scene_.acquire(
      unique_goal_instances(goal, catalog_), acquire_context);
    if (options_.wait_logger) {
      if (acquired.succeeded()) {
        options_.wait_logger->exit(
          WaitLifecycleOutcome::kReady, "goal-scoped scene lease acquired");
      } else {
        options_.wait_logger->exit(
          context.stop_requested() ? stopped_wait_outcome(context) :
          WaitLifecycleOutcome::kError,
          acquired.detail().empty() ?
          "goal-scoped scene lease acquisition failed" : acquired.detail());
      }
    }
    if (!acquired.succeeded()) {
      return no_step_failure(stopped_or(
        context, result_failure(acquired, "planning scene lease acquisition failed")));
    }

    GoalSceneLeaseGuard lease(scene_, *acquired.value());
    LeaseAwareExecutionContext step_context(
      context, lease, options_.scene_renew_period, options_.now, feedback,
      goal.steps.size(), options_.wait_logger, std::move(goal_id));
    std::unordered_map<std::string, geometry_msgs::msg::PoseStamped> initial_sources;
    std::vector<manipulation::task::TaskOutcome> outcomes;
    std::vector<Clock::duration> elapsed;
    outcomes.reserve(goal.steps.size());
    elapsed.reserve(goal.steps.size());

    for (std::size_t index = 0U; index < goal.steps.size(); ++index) {
      last_step_outcome_.reset();
      const auto started_at = options_.now();
      const auto & step = goal.steps[index];
      bool strategy_dispatched = false;
      bool step_threw = false;
      std::optional<WorkflowFailure> failed;
      try {
        step_context.set_active_step(index, step.object_id);
        failed = execute_step(
          step, step_context, context, lease, initial_sources, strategy_dispatched);
      } catch (const std::exception & error) {
        step_threw = true;
        failed = WorkflowFailure{
          ErrorCode::kInternalError,
          std::string{"resolved transfer step threw: "} + error.what()};
      } catch (...) {
        step_threw = true;
        failed = WorkflowFailure{
          ErrorCode::kInternalError, "resolved transfer step threw an unknown exception"};
      }
      elapsed.emplace_back(options_.now() - started_at);
      if (failed) {
        if (step_threw && strategy_dispatched && !last_step_outcome_) {
          strategy_.request_stop();
        }
        const auto terminal_failure = stopped_or(context, *failed);
        auto outcome =
          last_step_outcome_ && (!last_step_outcome_->succeeded() || step_threw) ?
          make_failure_preserving_evidence(terminal_failure, *last_step_outcome_) :
          make_failure(
          terminal_failure,
          lease.state_uncertain() ? Disposition::kSceneUncertain :
          failure_disposition(terminal_failure.code), 0U,
          step_threw && strategy_dispatched ? ResourceReleaseDisposition::kQuarantineRequired :
          ResourceReleaseDisposition::kReleaseSafe);
        outcomes.emplace_back(outcome);
        // An exception may have interrupted a scene mutation before it returned
        // evidence. Preserve the completed prefix and physical outcome, but do
        // not authorize scene-lease release from that incomplete boundary.
        if (!step_threw) {
          lease.finalize(outcome);
        }
        return {{outcome, index, index, std::move(outcomes)}, std::move(elapsed)};
      }

      if (!last_step_outcome_) {
        auto outcome = make_failure({
            ErrorCode::kInternalError,
            "resolved transfer step completed without strategy outcome evidence"});
        outcomes.emplace_back(outcome);
        lease.finalize(outcome);
        return {{outcome, index, index, std::move(outcomes)}, std::move(elapsed)};
      }
      const auto & outcome = *last_step_outcome_;
      outcomes.emplace_back(outcome);
      if (!outcome.succeeded()) {
        lease.finalize(outcome);
        return {{outcome, index, index, std::move(outcomes)}, std::move(elapsed)};
      }
      if (outcome.disposition() != Disposition::kObjectAtTarget) {
        auto invalid_outcome = make_failure(
          {
            ErrorCode::kInternalError,
            "strategy returned success without confirmed target disposition"},
          outcome.disposition(), outcome.recovery_count(), outcome.resource_release());
        outcomes.back() = invalid_outcome;
        lease.finalize(invalid_outcome);
        return {
          {invalid_outcome, index, index, std::move(outcomes)},
          std::move(elapsed)};
      }
    }

    auto aggregate = manipulation::task::TaskOutcome::make_success(
      Disposition::kObjectAtTarget, "all resolved transfer steps completed");
    lease.finalize(aggregate);
    return {{aggregate, goal.steps.size(), std::nullopt, std::move(outcomes)},
      std::move(elapsed)};
  }

  void request_stop() noexcept
  {
    observation_provider_.request_stop();
    grasp_provider_.request_stop();
    strategy_.request_stop();
    scene_.request_stop();
  }

private:
  [[nodiscard]] std::optional<WorkflowFailure> execute_step(
    const Step & step,
    LeaseAwareExecutionContext & step_context,
    TransferExecutionContext & root_context,
    GoalSceneLeaseGuard & lease,
    std::unordered_map<std::string, geometry_msgs::msg::PoseStamped> & initial_sources,
    bool & strategy_dispatched)
  {
    if (step_context.checkpoint() == TransferExecutionControlResult::kStopped) {
      return step_context.failure().value_or(
        WorkflowFailure{ErrorCode::kCanceled, "transfer step was stopped"});
    }

    const auto & instance = catalog_.require_instance(step.object_id);
    const auto & profile = catalog_.profile_for(step.object_id);
    publish_wait(
      step_context, manipulation::task::TaskPhase::kTracking,
      TransferWaitingReason::kObservation, "waiting for stamped object observation");
    const auto observed = observation_provider_.observe(instance, profile, step_context);
    if (step_context.failure()) {
      return step_context.failure();
    }
    if (!observed.succeeded()) {
      return stopped_or(
        root_context, result_failure(observed, "object observation failed"));
    }
    const auto observation = *observed.value();
    if (observation.object_id() != instance.object_id()) {
      return WorkflowFailure{
        ErrorCode::kInternalError, "observation provider returned another object instance"};
    }
    if (observation.pose().header.frame_id != options_.validation.planning_frame) {
      return WorkflowFailure{
        ErrorCode::kTfError, "object observation is not expressed in planning_frame"};
    }
    initial_sources.emplace(instance.object_id(), observation.pose());
    const auto target = detail::resolve_transfer_target(
      step, observation, profile, initial_sources);

    publish_wait(
      step_context, manipulation::task::TaskPhase::kWaitingForResources,
      TransferWaitingReason::kSceneReadiness,
      "staging observed catalog geometry in the planning scene");
    if (const auto failure = lease.stage_observation(
        instance, profile, observation, step_context))
    {
      return stopped_or(root_context, *failure);
    }

    publish_wait(
      step_context, manipulation::task::TaskPhase::kPlanning,
      TransferWaitingReason::kGraspGeneration, "waiting for ranked grasp candidates");
    const auto generated = grasp_provider_.generate(
      observation, profile, lease.snapshot(), step_context);
    if (step_context.failure()) {
      return step_context.failure();
    }
    if (!generated.succeeded()) {
      return stopped_or(
        root_context, result_failure(generated, "grasp generation failed"));
    }

    std::vector<GraspCandidate> ranked;
    try {
      ranked = rank_grasp_candidates(observation, *generated.value());
    } catch (const std::exception & error) {
      return WorkflowFailure{
        ErrorCode::kInternalError,
        std::string{"grasp provider violated correlation/ranking contract: "} + error.what()};
    }

    publish_wait(
      step_context, manipulation::task::TaskPhase::kWaitingForResources,
      TransferWaitingReason::kSceneReadiness, "revalidating planning scene before execution");
    if (const auto failure = lease.revalidate(step_context)) {
      return stopped_or(root_context, *failure);
    }

    ResolvedTransferTask resolved = ResolvedTransferTask::create(
      instance, profile, observation, ranked.front(), lease.snapshot(), lease.proof(),
      target, step.return_home, recovery_policy(step.recovery_policy));
    step_context.set_waiting(TransferWaitingReason::kNone);
    lease.mark_strategy_started();
    strategy_dispatched = true;
    last_step_outcome_.emplace(strategy_.execute(resolved, step_context));
    if (step_context.failure()) {
      return step_context.failure();
    }
    if (root_context.stop_requested() && last_step_outcome_->succeeded()) {
      return WorkflowFailure{ErrorCode::kCanceled, "strategy returned success after stop request"};
    }
    if (!last_step_outcome_->succeeded()) {
      return std::nullopt;
    }

    publish_wait(
      step_context, manipulation::task::TaskPhase::kWaitingForResources,
      TransferWaitingReason::kSceneReadiness, "confirming scene revision after execution");
    if (const auto failure = lease.revalidate(step_context)) {
      return stopped_or(root_context, *failure);
    }

    auto completed = step_context.current_state();
    completed.phase = manipulation::task::TaskPhase::kCompleted;
    completed.progress = 1.0F;
    completed.detail = "resolved transfer step completed";
    step_context.set_waiting(TransferWaitingReason::kNone);
    step_context.report_feedback(std::move(completed));
    return std::nullopt;
  }

  static void publish_wait(
    LeaseAwareExecutionContext & context,
    const manipulation::task::TaskPhase phase,
    const TransferWaitingReason reason,
    std::string detail)
  {
    context.set_waiting(reason, detail);
    auto state = context.current_state();
    state.phase = phase;
    state.detail = std::move(detail);
    context.report_feedback(std::move(state));
  }

  ResolvedTransferWorkflowOptions options_;
  const ObjectCatalog & catalog_;
  ObservationProvider & observation_provider_;
  GraspProvider & grasp_provider_;
  TransferSceneWorkflowPort & scene_;
  TransferExecutionStrategy & strategy_;
  std::optional<manipulation::task::TaskOutcome> last_step_outcome_;
};

ResolvedTransferWorkflow::ResolvedTransferWorkflow(
  ResolvedTransferWorkflowOptions options,
  const ObjectCatalog & catalog,
  ObservationProvider & observation_provider,
  GraspProvider & grasp_provider,
  TransferSceneWorkflowPort & scene,
  StrategyRegistry & strategies,
  const std::string_view selected_strategy_id)
: impl_(std::make_unique<Impl>(
    std::move(options), catalog, observation_provider, grasp_provider, scene,
    strategies, selected_strategy_id))
{
}

ResolvedTransferWorkflow::~ResolvedTransferWorkflow() = default;

TransferSequenceAdmission ResolvedTransferWorkflow::admit(const Goal & goal) const
{
  return impl_->admit(goal);
}

TransferSequenceExecution ResolvedTransferWorkflow::execute(
  const Goal & goal,
  TransferExecutionContext & context,
  ResolvedTransferFeedbackCallback feedback,
  std::string goal_id)
{
  try {
    return impl_->execute(
      goal, context, std::move(feedback), std::move(goal_id));
  } catch (const std::exception & error) {
    return no_step_failure({
        ErrorCode::kInternalError,
        std::string{"resolved transfer workflow failed closed: "} + error.what()});
  }
}

void ResolvedTransferWorkflow::request_stop() noexcept
{
  impl_->request_stop();
}

}  // namespace manipulation::workflow
