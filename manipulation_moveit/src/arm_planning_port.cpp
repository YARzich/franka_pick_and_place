// Responsibility: Applies bounded planning retries, cooperative stop, and scene revision gates.

#include "manipulation_moveit/arm_planning_port.hpp"

#include <algorithm>
#include <condition_variable>
#include <exception>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include "trajectory_state_validation.hpp"

namespace manipulation::moveit
{
namespace detail
{

struct PlanningStopState
{
  mutable std::mutex mutex;
  std::condition_variable cv;
  PlanningStopCause cause{PlanningStopCause::kNone};
};

}  // namespace detail
namespace
{

class InterruptiblePlanningBackoff final : public IPlanningBackoff
{
public:
  PlanningStopCause wait(
    const std::chrono::steady_clock::duration duration,
    const PlanningStopToken & stop_token) override
  {
    constexpr auto kHeartbeatSlice = std::chrono::milliseconds{5};
    const auto started_at = std::chrono::steady_clock::now();
    const auto maximum_duration =
      std::chrono::steady_clock::time_point::max() - started_at;
    const auto deadline = duration >= maximum_duration ?
      std::chrono::steady_clock::time_point::max() : started_at + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      if (const auto cause = stop_token.checkpoint();
        cause != PlanningStopCause::kNone)
      {
        return cause;
      }
      const auto now = std::chrono::steady_clock::now();
      if (now >= deadline) {
        break;
      }
      if (const auto cause = stop_token.wait_for(
          std::min<std::chrono::steady_clock::duration>(
            kHeartbeatSlice, deadline - now));
        cause != PlanningStopCause::kNone)
      {
        return cause;
      }
    }
    return stop_token.checkpoint();
  }
};

PlanningOutcome stop_outcome(const PlanningStopCause cause) noexcept
{
  switch (cause) {
    case PlanningStopCause::kNone:
      return PlanningOutcome::kInternalError;
    case PlanningStopCause::kCanceled:
      return PlanningOutcome::kCanceled;
    case PlanningStopCause::kShutdown:
      return PlanningOutcome::kShutdown;
  }
  return PlanningOutcome::kInternalError;
}

std::string stop_detail(const PlanningStopCause cause)
{
  switch (cause) {
    case PlanningStopCause::kNone:
      return "planning stop cause was not set";
    case PlanningStopCause::kCanceled:
      return "planning was canceled";
    case PlanningStopCause::kShutdown:
      return "planning stopped because the owner is shutting down";
  }
  return "planning stopped for an unknown reason";
}

ArmPlanningResult stopped_result(const PlanningStopCause cause)
{
  return ArmPlanningResult::failure(stop_outcome(cause), stop_detail(cause));
}

std::chrono::steady_clock::duration next_backoff(
  const std::chrono::steady_clock::duration current,
  const std::chrono::steady_clock::duration maximum) noexcept
{
  if (current >= maximum || current > maximum / 2) {
    return maximum;
  }
  return std::min(current * 2, maximum);
}

void validate_retry_policy(const PlanningRetryPolicy & policy)
{
  if (policy.max_attempts == 0U) {
    throw std::invalid_argument("planning retry policy must allow at least one attempt");
  }
  if (policy.initial_backoff <= std::chrono::steady_clock::duration::zero() ||
    policy.maximum_backoff < policy.initial_backoff)
  {
    throw std::invalid_argument(
            "planning retry backoff must be positive and bounded by its maximum");
  }
}

std::optional<std::string> validate_trajectory_start(
  const ArmPlanningRequest & request,
  const ArmPlanningResult & result)
{
  if (!result.succeeded()) {
    return std::nullopt;
  }
  constexpr double kPlanningStartTolerance = 1.0e-6;
  const auto check = detail::check_trajectory_start_state(
    request.scene_snapshot.robot_state, result.trajectory(), kPlanningStartTolerance);
  if (check.status == detail::TrajectoryStartStateStatus::kMatch) {
    return std::nullopt;
  }
  return check.detail;
}

}  // namespace

bool PlanningSceneRevision::operator==(const PlanningSceneRevision & other) const noexcept
{
  return session_id == other.session_id && revision == other.revision;
}

bool PlanningSceneRevision::operator!=(const PlanningSceneRevision & other) const noexcept
{
  return !(*this == other);
}

PlanningFailureClass classifyPlanningOutcome(const PlanningOutcome outcome) noexcept
{
  switch (outcome) {
    case PlanningOutcome::kSuccess:
      return PlanningFailureClass::kNone;
    case PlanningOutcome::kBudgetExhausted:
      return PlanningFailureClass::kRetryable;
    case PlanningOutcome::kCanceled:
    case PlanningOutcome::kShutdown:
      return PlanningFailureClass::kStopped;
    case PlanningOutcome::kInvalidRequest:
    case PlanningOutcome::kInvalidStartState:
    case PlanningOutcome::kInvalidGoalState:
    case PlanningOutcome::kSceneRefreshRequired:
    case PlanningOutcome::kSceneUnavailable:
    case PlanningOutcome::kPlannerUnavailable:
    case PlanningOutcome::kPlanningFailed:
    case PlanningOutcome::kInternalError:
      return PlanningFailureClass::kNonRetryable;
  }
  return PlanningFailureClass::kNonRetryable;
}

ArmPlanningResult ArmPlanningResult::success(
  moveit_msgs::msg::RobotTrajectory trajectory,
  std::string detail)
{
  return ArmPlanningResult(
    PlanningOutcome::kSuccess, std::move(trajectory), std::move(detail), 0);
}

ArmPlanningResult ArmPlanningResult::failure(
  const PlanningOutcome outcome,
  std::string detail,
  const std::int32_t backend_error_code)
{
  if (outcome == PlanningOutcome::kSuccess) {
    throw std::invalid_argument("a failed planning result cannot carry kSuccess");
  }
  return ArmPlanningResult(
    outcome, moveit_msgs::msg::RobotTrajectory{}, std::move(detail), backend_error_code);
}

ArmPlanningResult::ArmPlanningResult(
  const PlanningOutcome outcome,
  moveit_msgs::msg::RobotTrajectory trajectory,
  std::string detail,
  const std::int32_t backend_error_code)
: outcome_(outcome),
  trajectory_(std::move(trajectory)),
  detail_(std::move(detail)),
  backend_error_code_(backend_error_code)
{
}

void ArmPlanningResult::record_attempt_evidence(
  const std::size_t attempt_count,
  PlanningSceneRevision scene_revision)
{
  attempt_count_ = attempt_count;
  planned_scene_revision_ = std::move(scene_revision);
}

PlanningOutcome ArmPlanningResult::outcome() const noexcept
{
  return outcome_;
}

PlanningFailureClass ArmPlanningResult::failure_class() const noexcept
{
  return classifyPlanningOutcome(outcome_);
}

bool ArmPlanningResult::succeeded() const noexcept
{
  return outcome_ == PlanningOutcome::kSuccess;
}

bool ArmPlanningResult::retryable() const noexcept
{
  return failure_class() == PlanningFailureClass::kRetryable;
}

const moveit_msgs::msg::RobotTrajectory & ArmPlanningResult::trajectory() const noexcept
{
  return trajectory_;
}

std::int32_t ArmPlanningResult::backend_error_code() const noexcept
{
  return backend_error_code_;
}

std::size_t ArmPlanningResult::attempt_count() const noexcept
{
  return attempt_count_;
}

const PlanningSceneRevision & ArmPlanningResult::planned_scene_revision() const noexcept
{
  return planned_scene_revision_;
}

const std::string & ArmPlanningResult::detail() const noexcept
{
  return detail_;
}

PlanningStopToken::PlanningStopToken(
  std::shared_ptr<detail::PlanningStopState> state,
  manipulation::workflow::TransferExecutionContext * execution_context)
: state_(std::move(state)), execution_context_(execution_context)
{
}

bool PlanningStopToken::stop_requested() const noexcept
{
  return cause() != PlanningStopCause::kNone;
}

PlanningStopCause PlanningStopToken::cause() const noexcept
{
  if (state_) {
    const std::lock_guard<std::mutex> lock(state_->mutex);
    if (state_->cause != PlanningStopCause::kNone) {
      return state_->cause;
    }
  }
  if (!execution_context_ || !execution_context_->stop_requested()) {
    return PlanningStopCause::kNone;
  }
  return execution_context_->stop_cause() ==
         manipulation::workflow::TaskStopCause::kShutdown ?
         PlanningStopCause::kShutdown : PlanningStopCause::kCanceled;
}

PlanningStopCause PlanningStopToken::checkpoint() const
{
  if (const auto current = cause(); current != PlanningStopCause::kNone) {
    return current;
  }
  if (execution_context_ &&
    execution_context_->checkpoint() ==
    manipulation::workflow::TransferExecutionControlResult::kStopped)
  {
    const auto stopped = cause();
    return stopped == PlanningStopCause::kNone ?
           PlanningStopCause::kCanceled : stopped;
  }
  return cause();
}

PlanningStopCause PlanningStopToken::heartbeat() const
{
  if (const auto current = cause(); current != PlanningStopCause::kNone) {
    return current;
  }
  if (execution_context_ &&
    execution_context_->heartbeat() ==
    manipulation::workflow::TransferExecutionControlResult::kStopped)
  {
    const auto stopped = cause();
    return stopped == PlanningStopCause::kNone ?
           PlanningStopCause::kCanceled : stopped;
  }
  return cause();
}

manipulation::workflow::TaskOperationGateResult PlanningStopToken::run_operation(
  const std::function<void()> & operation) const
{
  if (!operation) {
    throw std::invalid_argument("planning operation must not be empty");
  }
  if (cause() != PlanningStopCause::kNone) {
    return manipulation::workflow::TaskOperationGateResult::kStopped;
  }
  if (execution_context_) {
    return execution_context_->run_operation(operation);
  }
  operation();
  return manipulation::workflow::TaskOperationGateResult::kExecuted;
}

PlanningStopCause PlanningStopToken::wait_for(
  const std::chrono::steady_clock::duration duration) const
{
  if (!state_) {
    return cause();
  }
  std::unique_lock<std::mutex> lock(state_->mutex);
  if (state_->cause == PlanningStopCause::kNone &&
    duration > std::chrono::steady_clock::duration::zero())
  {
    (void)state_->cv.wait_for(
      lock, duration, [this]() {return state_->cause != PlanningStopCause::kNone;});
  }
  lock.unlock();
  return cause();
}

ArmPlanningPort::ArmPlanningPort(
  IArmPlanningBackend & backend,
  IPlanningSceneRevisionHook & scene_revision_hook,
  PlanningRetryPolicy retry_policy)
: backend_(backend),
  scene_revision_hook_(scene_revision_hook),
  retry_policy_(retry_policy),
  owned_backoff_(std::make_unique<InterruptiblePlanningBackoff>()),
  backoff_(owned_backoff_.get()),
  stop_state_(std::make_shared<detail::PlanningStopState>())
{
  validate_retry_policy(retry_policy_);
}

ArmPlanningPort::ArmPlanningPort(
  IArmPlanningBackend & backend,
  IPlanningSceneRevisionHook & scene_revision_hook,
  PlanningRetryPolicy retry_policy,
  IPlanningBackoff & backoff)
: backend_(backend),
  scene_revision_hook_(scene_revision_hook),
  retry_policy_(retry_policy),
  backoff_(&backoff),
  stop_state_(std::make_shared<detail::PlanningStopState>())
{
  validate_retry_policy(retry_policy_);
}

ArmPlanningPort::~ArmPlanningPort()
{
  request_stop(PlanningStopCause::kShutdown);
}

void ArmPlanningPort::begin_goal()
{
  const std::lock_guard<std::mutex> plan_lock(plan_mutex_);
  const std::lock_guard<std::mutex> state_lock(stop_state_mutex_);
  stop_state_ = std::make_shared<detail::PlanningStopState>();
}

ArmPlanningReadiness ArmPlanningPort::cached_readiness() const
{
  return backend_.cached_readiness();
}

ArmPlanningResult ArmPlanningPort::plan(const ArmPlanningRequest & request)
{
  return plan_impl(request, nullptr);
}

ArmPlanningResult ArmPlanningPort::plan(
  const ArmPlanningRequest & request,
  manipulation::workflow::TransferExecutionContext & execution_context)
{
  return plan_impl(request, &execution_context);
}

ArmPlanningResult ArmPlanningPort::plan_impl(
  const ArmPlanningRequest & request,
  manipulation::workflow::TransferExecutionContext * execution_context)
{
  const std::lock_guard<std::mutex> plan_lock(plan_mutex_);
  const PlanningStopToken stop_token(current_stop_state(), execution_context);
  std::size_t completed_attempts = 0U;
  PlanningSceneRevision last_observed;

  if (request.scene_revision.session_id.empty()) {
    auto result = ArmPlanningResult::failure(
      PlanningOutcome::kInvalidRequest,
      "planning request must name an authoritative scene session");
    result.record_attempt_evidence(0U, {});
    return result;
  }
  if (request.scene_snapshot.is_diff || request.scene_snapshot.robot_state.is_diff) {
    auto result = ArmPlanningResult::failure(
      PlanningOutcome::kInvalidRequest,
      "planning scene input must be a complete immutable snapshot, not a diff");
    result.record_attempt_evidence(0U, {});
    return result;
  }

  auto delay = retry_policy_.initial_backoff;
  for (std::size_t attempt = 1U; attempt <= retry_policy_.max_attempts; ++attempt) {
    const auto stop_before_scene = stop_token.checkpoint();
    if (stop_before_scene != PlanningStopCause::kNone) {
      auto result = stopped_result(stop_before_scene);
      result.record_attempt_evidence(completed_attempts, last_observed);
      return result;
    }

    SceneRevisionCheck scene_check;
    try {
      scene_check = scene_revision_hook_.revalidate(request.scene_revision, stop_token);
    } catch (const std::exception & error) {
      auto result = ArmPlanningResult::failure(
        PlanningOutcome::kInternalError,
        std::string{"scene revision hook threw: "} + error.what());
      result.record_attempt_evidence(completed_attempts, last_observed);
      return result;
    } catch (...) {
      auto result = ArmPlanningResult::failure(
        PlanningOutcome::kInternalError, "scene revision hook threw an unknown exception");
      result.record_attempt_evidence(completed_attempts, last_observed);
      return result;
    }
    last_observed = scene_check.observed;

    const auto stop_after_scene = stop_token.checkpoint();
    if (stop_after_scene != PlanningStopCause::kNone) {
      auto result = stopped_result(stop_after_scene);
      result.record_attempt_evidence(completed_attempts, last_observed);
      return result;
    }
    bool scene_state_is_current = false;
    switch (scene_check.state) {
      case SceneRevisionState::kCurrent: {
          if (scene_check.observed != request.scene_revision) {
            auto result = ArmPlanningResult::failure(
              PlanningOutcome::kSceneRefreshRequired, std::move(scene_check.detail));
            result.record_attempt_evidence(completed_attempts, last_observed);
            return result;
          }
          scene_state_is_current = true;
          break;
        }
      case SceneRevisionState::kRefreshRequired: {
          auto result = ArmPlanningResult::failure(
            PlanningOutcome::kSceneRefreshRequired, std::move(scene_check.detail));
          result.record_attempt_evidence(completed_attempts, last_observed);
          return result;
        }
      case SceneRevisionState::kUnavailable: {
          auto result = ArmPlanningResult::failure(
            PlanningOutcome::kSceneUnavailable, std::move(scene_check.detail));
          result.record_attempt_evidence(completed_attempts, last_observed);
          return result;
        }
    }
    if (!scene_state_is_current) {
      auto result = ArmPlanningResult::failure(
        PlanningOutcome::kInternalError, "scene revision hook returned an unknown state");
      result.record_attempt_evidence(completed_attempts, last_observed);
      return result;
    }

    ArmPlanningResult result = ArmPlanningResult::failure(
      PlanningOutcome::kInternalError, "planning backend returned no result");
    try {
      result = backend_.plan_once(request, stop_token);
    } catch (const std::exception & error) {
      result = ArmPlanningResult::failure(
        PlanningOutcome::kInternalError,
        std::string{"planning backend threw: "} + error.what());
    } catch (...) {
      result = ArmPlanningResult::failure(
        PlanningOutcome::kInternalError, "planning backend threw an unknown exception");
    }
    completed_attempts = attempt;
    if (const auto stopped = stop_token.heartbeat();
      stopped != PlanningStopCause::kNone)
    {
      auto stopped_planning = stopped_result(stopped);
      stopped_planning.record_attempt_evidence(completed_attempts, last_observed);
      return stopped_planning;
    }
    if (const auto start_error = validate_trajectory_start(request, result)) {
      result = ArmPlanningResult::failure(
        PlanningOutcome::kInvalidStartState, *start_error);
    }
    result.record_attempt_evidence(completed_attempts, last_observed);
    if (!result.retryable() || attempt == retry_policy_.max_attempts) {
      return result;
    }

    const auto stopped = backoff_->wait(delay, stop_token);
    if (stopped != PlanningStopCause::kNone) {
      auto stopped_planning = stopped_result(stopped);
      stopped_planning.record_attempt_evidence(completed_attempts, last_observed);
      return stopped_planning;
    }
    if (const auto checkpoint = stop_token.checkpoint();
      checkpoint != PlanningStopCause::kNone)
    {
      auto stopped_planning = stopped_result(checkpoint);
      stopped_planning.record_attempt_evidence(completed_attempts, last_observed);
      return stopped_planning;
    }
    delay = next_backoff(delay, retry_policy_.maximum_backoff);
  }

  auto result = ArmPlanningResult::failure(
    PlanningOutcome::kInternalError, "planning retry loop terminated unexpectedly");
  result.record_attempt_evidence(completed_attempts, last_observed);
  return result;
}

void ArmPlanningPort::request_stop(const PlanningStopCause cause) noexcept
{
  if (cause == PlanningStopCause::kNone) {
    return;
  }
  const auto state = current_stop_state();
  if (state) {
    const std::lock_guard<std::mutex> lock(state->mutex);
    if (state->cause == PlanningStopCause::kNone || cause == PlanningStopCause::kShutdown) {
      state->cause = cause;
    }
    state->cv.notify_all();
  }
  backend_.request_stop();
}

std::shared_ptr<detail::PlanningStopState> ArmPlanningPort::current_stop_state() const
{
  const std::lock_guard<std::mutex> lock(stop_state_mutex_);
  return stop_state_;
}

}  // namespace manipulation::moveit
