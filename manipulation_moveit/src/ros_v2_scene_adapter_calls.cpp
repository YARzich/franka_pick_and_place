// Responsibility: Implements V2 lease lifecycle, full-scene reads, and revision checks.

#include "ros_v2_scene_adapter_impl.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "manipulation/workflow/transfer_execution_context.hpp"
#include "manipulation_msgs/msg/planning_scene_operation_result.hpp"

namespace manipulation::moveit
{
namespace
{

using ControlResult = manipulation::workflow::TransferExecutionControlResult;
using ErrorCode = manipulation::task::TaskErrorCode;
using OperationResult = manipulation_msgs::msg::PlanningSceneOperationResult;
using WaitingReason = manipulation::workflow::TransferWaitingReason;

std::chrono::steady_clock::duration duration_or_zero(
  const builtin_interfaces::msg::Duration & duration)
{
  if (duration.sec < 0 || duration.nanosec >= 1000000000U) {
    return std::chrono::steady_clock::duration::zero();
  }
  return std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::seconds{duration.sec} + std::chrono::nanoseconds{duration.nanosec});
}

}  // namespace

RosV2SceneAdapter::Impl::WaitControl RosV2SceneAdapter::Impl::context_control(
  manipulation::workflow::TransferExecutionContext & context,
  const std::uint64_t generation) const
{
  WaitControl control;
  control.stopped = [this, &context, generation]() {
      return context.stop_requested() || stop_state_.stopped(generation);
    };
  control.wait_one_slice = [this, &context, generation]() {
      if (context.stop_requested() ||
        stop_state_.stopped(generation))
      {
        return detail::InterruptibleWaitResult::kStopped;
      }
      if (context.wait_for(kWaitSlice) == ControlResult::kStopped ||
        context.stop_requested() ||
        stop_state_.stopped(generation))
      {
        return detail::InterruptibleWaitResult::kStopped;
      }
      return detail::InterruptibleWaitResult::kReady;
    };
  control.heartbeat = [this, &context, generation]() {
      if (context.heartbeat() == ControlResult::kStopped ||
        stop_state_.stopped(generation))
      {
        return detail::InterruptibleWaitResult::kStopped;
      }
      if (!context_ || !context_->is_valid()) {
        return detail::InterruptibleWaitResult::kShutdown;
      }
      return detail::InterruptibleWaitResult::kReady;
    };
  control.run_operation = [this, &context, generation](
    const std::function<void()> & operation)
    {
      bool stopped_inside_gate = false;
      const auto result = detail::dispatch_scene_operation(
        context, detail::SceneDispatchClass::kGoalSideEffect,
        [this, &operation, generation, &stopped_inside_gate]() {
          const std::lock_guard<std::mutex> operation_lock(operation_mutex_);
          if (stop_state_.stopped(generation)) {
            stopped_inside_gate = true;
            return;
          }
          operation();
        });
      return stopped_inside_gate ?
             manipulation::workflow::TaskOperationGateResult::kStopped : result;
    };
  control.report_wait = [&context](const std::string & label) {
      manipulation::task::TaskStateSnapshot state;
      state.phase = manipulation::task::TaskPhase::kWaitingForResources;
      state.detail = label;
      context.report_waiting(WaitingReason::kRosEndpoint, std::move(state));
    };
  return control;
}

RosV2SceneAdapter::Impl::WaitControl RosV2SceneAdapter::Impl::planning_control(
  const PlanningStopToken & stop_token,
  const std::uint64_t generation) const
{
  WaitControl control;
  control.stopped = [this, &stop_token, generation]() {
      return stop_token.stop_requested() || stop_state_.stopped(generation);
    };
  control.wait_one_slice = [this, &stop_token, generation]() {
      if (stop_token.stop_requested() ||
        stop_state_.stopped(generation))
      {
        return detail::InterruptibleWaitResult::kStopped;
      }
      if (stop_token.checkpoint() != PlanningStopCause::kNone ||
        stop_token.wait_for(kWaitSlice) != PlanningStopCause::kNone)
      {
        return detail::InterruptibleWaitResult::kStopped;
      }
      return detail::InterruptibleWaitResult::kReady;
    };
  control.heartbeat = [this, &stop_token, generation]() {
      if (stop_token.heartbeat() != PlanningStopCause::kNone ||
        stop_state_.stopped(generation))
      {
        return detail::InterruptibleWaitResult::kStopped;
      }
      if (!context_ || !context_->is_valid()) {
        return detail::InterruptibleWaitResult::kShutdown;
      }
      return detail::InterruptibleWaitResult::kReady;
    };
  control.run_operation = [this, &stop_token, generation](
    const std::function<void()> & operation)
    {
      bool stopped_inside_gate = false;
      const auto result = stop_token.run_operation(
        [this, &operation, generation, &stopped_inside_gate]() {
          const std::lock_guard<std::mutex> operation_lock(operation_mutex_);
          // The planning token gate owns the lifecycle session mutex.
          if (stop_state_.stopped(generation)) {
            stopped_inside_gate = true;
            return;
          }
          operation();
        });
      return stopped_inside_gate ?
             manipulation::workflow::TaskOperationGateResult::kStopped : result;
    };
  control.report_wait = [](const std::string &) {};
  return control;
}

RosV2SceneAdapter::Impl::WaitControl RosV2SceneAdapter::Impl::maintenance_control(
  manipulation::workflow::TransferExecutionContext & context,
  const std::uint64_t generation) const
{
  WaitControl control;
  control.stopped = [this, &context, generation]() {
      return context.stop_requested() || stop_state_.stopped(generation);
    };
  control.wait_one_slice = [this, &context, generation]() {
      if (context.heartbeat() == ControlResult::kStopped ||
        stop_state_.stopped(generation))
      {
        return detail::InterruptibleWaitResult::kStopped;
      }
      if (!context_ || !context_->is_valid()) {
        return detail::InterruptibleWaitResult::kShutdown;
      }
      std::this_thread::sleep_for(kWaitSlice);
      return detail::InterruptibleWaitResult::kReady;
    };
  control.heartbeat = [this, &context, generation]() {
      if (context.heartbeat() == ControlResult::kStopped ||
        stop_state_.stopped(generation))
      {
        return detail::InterruptibleWaitResult::kStopped;
      }
      if (!context_ || !context_->is_valid()) {
        return detail::InterruptibleWaitResult::kShutdown;
      }
      return detail::InterruptibleWaitResult::kReady;
    };
  control.run_operation = [this, &context, generation](
    const std::function<void()> & operation)
    {
      bool stopped_inside_gate = false;
      const auto result = detail::dispatch_scene_operation(
        context, detail::SceneDispatchClass::kLeaseMaintenance,
        [this, &operation, generation, &stopped_inside_gate]() {
          const std::lock_guard<std::mutex> operation_lock(operation_mutex_);
          if (stop_state_.stopped(generation)) {
            stopped_inside_gate = true;
            return;
          }
          operation();
        });
      return stopped_inside_gate ?
             manipulation::workflow::TaskOperationGateResult::kStopped : result;
    };
  control.report_wait = [&context](const std::string & label) {
      manipulation::task::TaskStateSnapshot state;
      state.phase = manipulation::task::TaskPhase::kWaitingForResources;
      state.detail = label;
      context.report_waiting(WaitingReason::kRosEndpoint, std::move(state));
    };
  return control;
}

std::optional<RosV2SceneAdapter::Impl::ActiveLease>
RosV2SceneAdapter::Impl::active_lease_for(
  const manipulation::workflow::PlanningSceneAccessProof & proof,
  std::string & detail_text) const
{
  const std::lock_guard<std::mutex> lock(active_mutex_);
  if (!active_lease_) {
    detail_text = "no active goal-scoped scene lease";
    return std::nullopt;
  }
  if (active_lease_->lease_id != proof.lease_id()) {
    detail_text = "scene proof does not identify the active lease";
    return std::nullopt;
  }
  if (!active_lease_->session_id.empty() &&
    active_lease_->session_id != proof.coordinator_session_id())
  {
    detail_text = "scene proof does not identify the active coordinator session";
    return std::nullopt;
  }
  return active_lease_;
}

void RosV2SceneAdapter::Impl::set_active_lease(ActiveLease lease)
{
  const std::lock_guard<std::mutex> lock(active_mutex_);
  active_lease_ = std::move(lease);
}

void RosV2SceneAdapter::Impl::clear_active_lease(const std::string & lease_id) noexcept
{
  const std::lock_guard<std::mutex> lock(active_mutex_);
  if (active_lease_ && active_lease_->lease_id == lease_id) {
    active_lease_.reset();
  }
}

builtin_interfaces::msg::Duration RosV2SceneAdapter::Impl::requested_lease_ttl() const
{
  builtin_interfaces::msg::Duration duration;
  duration.sec = static_cast<std::int32_t>(
    std::chrono::duration_cast<std::chrono::seconds>(kLeaseTtl).count());
  return duration;
}

RosV2SceneAdapter::Impl::FailureKind RosV2SceneAdapter::Impl::coordinator_failure(
  const OperationResult & result)
{
  if (result.success) {
    return FailureKind::kInternal;
  }
  switch (result.error_code) {
    case OperationResult::INVALID_REQUEST:
      return FailureKind::kInvalid;
    case OperationResult::RESOURCE_CONFLICT:
    case OperationResult::LEASE_NOT_FOUND:
    case OperationResult::LEASE_EXPIRED:
    case OperationResult::NOT_OWNER:
    case OperationResult::REVISION_MISMATCH:
    case OperationResult::SESSION_MISMATCH:
    case OperationResult::IDEMPOTENCY_CONFLICT:
    case OperationResult::OPERATION_IN_PROGRESS:
    case OperationResult::RECONCILIATION_REQUIRED:
    case OperationResult::RESOURCE_QUARANTINED:
      return FailureKind::kConflict;
    case OperationResult::DEPENDENCY_UNAVAILABLE:
      return FailureKind::kUnavailable;
    case OperationResult::BACKEND_FAILED:
    case OperationResult::TIMEOUT:
    case OperationResult::OPERATION_TICKET_INVALID:
    case OperationResult::OPERATION_CAPACITY_EXCEEDED:
      return FailureKind::kFailed;
    case OperationResult::NONE:
    case OperationResult::INTERNAL_ERROR:
      return FailureKind::kInternal;
  }
  return FailureKind::kInternal;
}

ErrorCode RosV2SceneAdapter::Impl::task_error(const FailureKind failure)
{
  switch (failure) {
    case FailureKind::kCanceled:
    case FailureKind::kShutdown:
      return ErrorCode::kCanceled;
    case FailureKind::kConflict:
      return ErrorCode::kSceneConflict;
    case FailureKind::kUnavailable:
      return ErrorCode::kResourceUnavailable;
    case FailureKind::kInvalid:
      return ErrorCode::kInvalidGoal;
    case FailureKind::kFailed:
    case FailureKind::kStateUncertain:
      return ErrorCode::kSceneFailed;
    case FailureKind::kInternal:
      return ErrorCode::kInternalError;
  }
  return ErrorCode::kInternalError;
}

CoordinatedSceneOutcome RosV2SceneAdapter::Impl::coordinated_outcome(
  const FailureKind failure)
{
  switch (failure) {
    case FailureKind::kCanceled:
      return CoordinatedSceneOutcome::kCanceled;
    case FailureKind::kShutdown:
      return CoordinatedSceneOutcome::kShutdown;
    case FailureKind::kConflict:
      return CoordinatedSceneOutcome::kConflict;
    case FailureKind::kUnavailable:
      return CoordinatedSceneOutcome::kUnavailable;
    case FailureKind::kInvalid:
    case FailureKind::kFailed:
      return CoordinatedSceneOutcome::kFailed;
    case FailureKind::kStateUncertain:
      return CoordinatedSceneOutcome::kStateUncertain;
    case FailureKind::kInternal:
      return CoordinatedSceneOutcome::kInternalError;
  }
  return CoordinatedSceneOutcome::kInternalError;
}

RosV2SceneAdapter::Impl::FullReadResult RosV2SceneAdapter::Impl::read_full_scene(
  const ActiveLease & active,
  const std::string & expected_session_id,
  const std::uint64_t minimum_revision,
  manipulation::workflow::TransferExecutionContext & context,
  const std::uint64_t generation)
{
  const auto control = context_control(context, generation);
  const auto stopped = [&control]() {
      return control.stopped && control.stopped();
    };
  const auto wait_before_retry = [&control]() {
      return control.wait_one_slice() == detail::InterruptibleWaitResult::kReady;
    };

  while (true) {
    if (stopped()) {
      return {{}, FailureKind::kCanceled, "full PlanningScene read was stopped"};
    }
    const auto before_call = call_service<GetState>(
      *state_client_, std::make_shared<GetState::Request>(), control,
      "waiting for coordinator state before full PlanningScene read");
    if (before_call.status != ServiceStatus::kResponse) {
      const auto failure = before_call.status == ServiceStatus::kStopped ?
        FailureKind::kCanceled :
        before_call.status == ServiceStatus::kShutdown ?
        FailureKind::kShutdown : FailureKind::kInternal;
      return {{}, failure, before_call.detail};
    }
    const auto before = detail::validate_coordinator_state(
      before_call.response->state, before_call.response->result);
    if (!before.valid) {
      const auto & state = before_call.response->state;
      if (!before_call.response->result.success) {
        return {
          {}, coordinator_failure(before_call.response->result), before.detail};
      }
      if (before_call.response->result.success &&
        (state.operation_in_progress || !state.backend_service_ready))
      {
        if (!wait_before_retry()) {
          return {{}, FailureKind::kCanceled, "scene readiness wait was stopped"};
        }
        continue;
      }
      return {{}, FailureKind::kConflict, before.detail};
    }
    if ((!expected_session_id.empty() &&
      before.state.session_id != expected_session_id) ||
      before.state.revision < minimum_revision)
    {
      return {
        {}, FailureKind::kConflict,
        "coordinator scene identity no longer matches required evidence"};
    }

    auto validation_request = std::make_shared<Validate::Request>();
    validation_request->lease_id = active.lease_id;
    validation_request->required_resource_keys = active.resource_keys;
    validation_request->expected_revision = before.state.revision;
    const auto validation = call_service<Validate>(
      *validate_client_, std::move(validation_request), control,
      "waiting for exact scene lease validation");
    if (validation.status != ServiceStatus::kResponse) {
      const auto failure = validation.status == ServiceStatus::kStopped ?
        FailureKind::kCanceled :
        validation.status == ServiceStatus::kShutdown ?
        FailureKind::kShutdown : FailureKind::kInternal;
      return {{}, failure, validation.detail};
    }
    const auto & validation_response = *validation.response;
    if (!validation_response.result.success) {
      const bool revision_race =
        validation_response.result.error_code == OperationResult::REVISION_MISMATCH &&
        validation_response.session_id == before.state.session_id &&
        validation_response.result.revision > before.state.revision;
      if (revision_race) {
        if (!wait_before_retry()) {
          return {{}, FailureKind::kCanceled, "scene revision retry was stopped"};
        }
        continue;
      }
      return {
        {}, coordinator_failure(validation_response.result),
        validation_response.result.message};
    }
    const auto remaining_ttl = duration_or_zero(validation_response.remaining_ttl);
    if (validation_response.result.error_code != OperationResult::NONE ||
      validation_response.session_id != before.state.session_id ||
      validation_response.result.revision != before.state.revision ||
      remaining_ttl <= kExpectedRenewPeriod)
    {
      return {
        {}, FailureKind::kConflict,
        "lease validation did not prove the bracketed session/revision and renewal horizon"};
    }

    const auto scene_call = call_service<GetScene>(
      *get_scene_client_, detail::make_full_planning_scene_request(), control,
      "waiting for complete authoritative PlanningScene");
    if (scene_call.status != ServiceStatus::kResponse) {
      const auto failure = scene_call.status == ServiceStatus::kStopped ?
        FailureKind::kCanceled :
        scene_call.status == ServiceStatus::kShutdown ?
        FailureKind::kShutdown : FailureKind::kInternal;
      return {{}, failure, scene_call.detail};
    }

    const auto after_call = call_service<GetState>(
      *state_client_, std::make_shared<GetState::Request>(), control,
      "waiting for coordinator state after full PlanningScene read");
    if (after_call.status != ServiceStatus::kResponse) {
      const auto failure = after_call.status == ServiceStatus::kStopped ?
        FailureKind::kCanceled :
        after_call.status == ServiceStatus::kShutdown ?
        FailureKind::kShutdown : FailureKind::kInternal;
      return {{}, failure, after_call.detail};
    }
    const auto after = detail::validate_coordinator_state(
      after_call.response->state, after_call.response->result);
    if (!after.valid) {
      if (!after_call.response->result.success) {
        return {
          {}, coordinator_failure(after_call.response->result), after.detail};
      }
      if (after.state.operation_in_progress || !after.state.backend_service_ready) {
        if (!wait_before_retry()) {
          return {{}, FailureKind::kCanceled, "scene readiness retry was stopped"};
        }
        continue;
      }
      return {{}, FailureKind::kConflict, after.detail};
    }
    const auto bracket = detail::check_full_scene_bracket(
      before, scene_call.response->scene, after,
      expected_session_id, minimum_revision);
    if (bracket.status == detail::FullSceneBracketStatus::kRetry) {
      if (!wait_before_retry()) {
        return {{}, FailureKind::kCanceled, "full scene bracket retry was stopped"};
      }
      continue;
    }
    if (bracket.status != detail::FullSceneBracketStatus::kAccepted) {
      return {
        {}, bracket.status == detail::FullSceneBracketStatus::kConflict ?
        FailureKind::kConflict : FailureKind::kInternal,
        bracket.detail};
    }

    try {
      auto snapshot = manipulation::workflow::PlanningSceneSnapshot::create(
        after.state.session_id, after.state.revision,
        std::move(scene_call.response->scene));
      auto proof = manipulation::workflow::PlanningSceneAccessProof::create(
        active.lease_id, after.state.session_id, after.state.revision);
      return {
        manipulation::workflow::PlanningSceneLease::create(
          std::move(snapshot), std::move(proof)),
        FailureKind::kInternal, {}};
    } catch (const std::exception & error) {
      return {
        {}, FailureKind::kInternal,
        std::string{"full scene evidence is invalid: "} + error.what()};
    }
  }
}

RosV2SceneAdapter::Impl::ProofCheckResult
RosV2SceneAdapter::Impl::validate_exact_proof(
  const manipulation::workflow::PlanningSceneAccessProof & proof,
  const ActiveLease & active,
  manipulation::workflow::TransferExecutionContext & context,
  const std::uint64_t generation)
{
  const auto control = context_control(context, generation);
  while (true) {
    const auto state_call = call_service<GetState>(
      *state_client_, std::make_shared<GetState::Request>(), control,
      "waiting for current coordinator state before scene mutation");
    if (state_call.status != ServiceStatus::kResponse) {
      const auto failure = state_call.status == ServiceStatus::kStopped ?
        FailureKind::kCanceled :
        state_call.status == ServiceStatus::kShutdown ?
        FailureKind::kShutdown : FailureKind::kInternal;
      return {false, failure, state_call.detail};
    }
    const auto state = detail::validate_coordinator_state(
      state_call.response->state, state_call.response->result);
    if (!state.valid) {
      if (state_call.response->result.success &&
        (state.state.operation_in_progress || !state.state.backend_service_ready))
      {
        if (control.wait_one_slice() != detail::InterruptibleWaitResult::kReady) {
          return {false, FailureKind::kCanceled, "scene readiness wait was stopped"};
        }
        continue;
      }
      return {
        false,
        state_call.response->result.success ? FailureKind::kConflict :
        coordinator_failure(state_call.response->result),
        state.detail};
    }
    if (state.state.session_id != proof.coordinator_session_id() ||
      state.state.revision != proof.revision())
    {
      return {
        false, FailureKind::kConflict,
        "scene changed after the supplied full snapshot was captured"};
    }
    break;
  }

  auto request = std::make_shared<Validate::Request>();
  request->lease_id = active.lease_id;
  request->required_resource_keys = active.resource_keys;
  request->expected_revision = proof.revision();
  const auto validation = call_service<Validate>(
    *validate_client_, std::move(request), control,
    "waiting for exact lease proof before scene mutation");
  if (validation.status != ServiceStatus::kResponse) {
    const auto failure = validation.status == ServiceStatus::kStopped ?
      FailureKind::kCanceled :
      validation.status == ServiceStatus::kShutdown ?
      FailureKind::kShutdown : FailureKind::kInternal;
    return {false, failure, validation.detail};
  }
  const auto & response = *validation.response;
  if (!response.result.success) {
    return {
      false, coordinator_failure(response.result), response.result.message};
  }
  if (response.result.error_code != OperationResult::NONE ||
    response.session_id != proof.coordinator_session_id() ||
    response.result.revision != proof.revision() ||
    duration_or_zero(response.remaining_ttl) <= kExpectedRenewPeriod)
  {
    return {
      false, FailureKind::kConflict,
      "coordinator validation contradicted the exact scene proof"};
  }
  return {true, FailureKind::kInternal, {}};
}

manipulation::workflow::PlanningSceneLeaseResult RosV2SceneAdapter::Impl::acquire(
  const std::vector<manipulation::workflow::ObjectInstance> & instances,
  manipulation::workflow::TransferExecutionContext & context)
{
  begin_goal();
  const auto generation = stop_state_.capture();
  std::vector<std::string> resources;
  try {
    resources = detail::make_goal_scene_resource_keys(instances, config_.touch_links);
  } catch (const std::exception & error) {
    return manipulation::workflow::PlanningSceneLeaseResult::failure(
      ErrorCode::kInvalidGoal, error.what());
  }

  const std::lock_guard<std::mutex> mutation_lock(mutation_mutex_);
  {
    const std::lock_guard<std::mutex> active_lock(active_mutex_);
    if (active_lease_) {
      return manipulation::workflow::PlanningSceneLeaseResult::failure(
        ErrorCode::kSceneConflict, "a goal-scoped scene lease is already active");
    }
  }
  auto request = std::make_shared<Acquire::Request>();
  request->owner_id = config_.owner_id;
  request->resource_keys = resources;
  request->requested_ttl = requested_lease_ttl();
  const auto call = call_service<Acquire>(
    *acquire_client_, std::move(request), context_control(context, generation),
    "waiting to acquire all goal scene resources");
  if (call.status != ServiceStatus::kResponse) {
    const auto failure = call.status == ServiceStatus::kStopped ?
      FailureKind::kCanceled :
      call.status == ServiceStatus::kShutdown ?
      FailureKind::kShutdown : FailureKind::kInternal;
    return manipulation::workflow::PlanningSceneLeaseResult::failure(
      task_error(failure), call.detail);
  }
  const auto & response = *call.response;
  if (!response.result.success) {
    if (!response.lease_id.empty()) {
      release_lease_id_best_effort(response.lease_id);
    }
    return manipulation::workflow::PlanningSceneLeaseResult::failure(
      task_error(coordinator_failure(response.result)), response.result.message);
  }
  if (response.result.error_code != OperationResult::NONE ||
    response.lease_id.empty() ||
    duration_or_zero(response.granted_ttl) <= kExpectedRenewPeriod * 2)
  {
    if (!response.lease_id.empty()) {
      release_lease_id_best_effort(response.lease_id);
    }
    return manipulation::workflow::PlanningSceneLeaseResult::failure(
      ErrorCode::kInternalError,
      "coordinator returned contradictory or insufficient lease evidence");
  }

  ActiveLease active{response.lease_id, {}, std::move(resources)};
  set_active_lease(active);
  auto full = read_full_scene(
    active, {}, response.result.revision, context, generation);
  if (!full.succeeded()) {
    clear_active_lease(active.lease_id);
    release_lease_id_best_effort(active.lease_id);
    return manipulation::workflow::PlanningSceneLeaseResult::failure(
      task_error(full.failure), std::move(full.detail));
  }
  active.session_id = full.lease->snapshot().coordinator_session_id();
  set_active_lease(active);
  return manipulation::workflow::PlanningSceneLeaseResult::success(
    std::move(*full.lease));
}

manipulation::workflow::PlanningSceneAccessProofResult
RosV2SceneAdapter::Impl::renew(
  const manipulation::workflow::PlanningSceneAccessProof & proof,
  manipulation::workflow::TransferExecutionContext & context)
{
  const auto generation = stop_state_.capture();
  std::string detail_text;
  const auto active = active_lease_for(proof, detail_text);
  if (!active) {
    return manipulation::workflow::PlanningSceneAccessProofResult::failure(
      ErrorCode::kSceneConflict, std::move(detail_text));
  }

  auto request = std::make_shared<Renew::Request>();
  request->lease_id = proof.lease_id();
  request->requested_ttl = requested_lease_ttl();
  const auto call = call_service<Renew>(
    *renew_client_, std::move(request), maintenance_control(context, generation),
    "waiting to renew goal-scoped scene ownership");
  if (call.status != ServiceStatus::kResponse) {
    const auto failure = call.status == ServiceStatus::kStopped ?
      FailureKind::kCanceled :
      call.status == ServiceStatus::kShutdown ?
      FailureKind::kShutdown : FailureKind::kInternal;
    return manipulation::workflow::PlanningSceneAccessProofResult::failure(
      task_error(failure), call.detail);
  }
  const auto & response = *call.response;
  if (!response.result.success) {
    return manipulation::workflow::PlanningSceneAccessProofResult::failure(
      task_error(coordinator_failure(response.result)), response.result.message);
  }
  if (response.result.error_code != OperationResult::NONE ||
    response.result.revision < proof.revision() ||
    duration_or_zero(response.granted_ttl) <= kExpectedRenewPeriod * 2)
  {
    return manipulation::workflow::PlanningSceneAccessProofResult::failure(
      ErrorCode::kSceneConflict,
      "lease renewal moved revision backward or returned insufficient TTL");
  }
  // RenewLease changes only ownership lifetime. A concurrent or just-completed
  // coordinator mutation may advance the global revision while the workflow
  // still owns the preceding immutable snapshot. Keep that proof unchanged;
  // bracketed revalidate() is the only operation that publishes a new revision.
  return manipulation::workflow::PlanningSceneAccessProofResult::success(proof);
}

manipulation::workflow::PlanningSceneLeaseResult
RosV2SceneAdapter::Impl::revalidate(
  const manipulation::workflow::PlanningSceneAccessProof & proof,
  manipulation::workflow::TransferExecutionContext & context)
{
  const auto generation = stop_state_.capture();
  std::string detail_text;
  const auto active = active_lease_for(proof, detail_text);
  if (!active) {
    return manipulation::workflow::PlanningSceneLeaseResult::failure(
      ErrorCode::kSceneConflict, std::move(detail_text));
  }
  auto full = read_full_scene(
    *active, proof.coordinator_session_id(), proof.revision(), context, generation);
  if (!full.succeeded()) {
    return manipulation::workflow::PlanningSceneLeaseResult::failure(
      task_error(full.failure), std::move(full.detail));
  }
  return manipulation::workflow::PlanningSceneLeaseResult::success(
    std::move(*full.lease));
}

CoordinatedSceneResult RosV2SceneAdapter::Impl::strategy_revalidate(
  const StrategySceneRevalidationRequest & request,
  manipulation::workflow::TransferExecutionContext & context)
{
  if (request.scene_snapshot.coordinator_session_id() !=
    request.access_proof.coordinator_session_id() ||
    request.scene_snapshot.revision() != request.access_proof.revision() ||
    request.scene_snapshot.scene().is_diff ||
    request.scene_snapshot.scene().robot_state.is_diff)
  {
    return CoordinatedSceneResult::failure(
      CoordinatedSceneOutcome::kConflict,
      "strategy supplied mismatched or partial scene evidence");
  }
  const auto result = revalidate(request.access_proof, context);
  if (!result.succeeded()) {
    FailureKind failure = FailureKind::kFailed;
    switch (result.error_code()) {
      case ErrorCode::kCanceled:
        failure = FailureKind::kCanceled;
        break;
      case ErrorCode::kSceneConflict:
        failure = FailureKind::kConflict;
        break;
      case ErrorCode::kResourceUnavailable:
        failure = FailureKind::kUnavailable;
        break;
      case ErrorCode::kInternalError:
        failure = FailureKind::kInternal;
        break;
      default:
        failure = FailureKind::kFailed;
        break;
    }
    return CoordinatedSceneResult::failure(
      coordinated_outcome(failure), result.detail());
  }
  const auto & lease = *result.value();
  if (lease.snapshot().revision() != request.scene_snapshot.revision()) {
    return CoordinatedSceneResult::failure(
      CoordinatedSceneOutcome::kConflict,
      "strategy scene revision changed during exact revalidation");
  }
  return CoordinatedSceneResult::success(lease, "scene evidence is current");
}

SceneRevisionCheck RosV2SceneAdapter::Impl::planning_revalidate(
  const PlanningSceneRevision & expected,
  const PlanningStopToken & stop_token)
{
  const auto generation = stop_state_.capture();
  const auto control = planning_control(stop_token, generation);
  while (true) {
    const auto call = call_service<GetState>(
      *state_client_, std::make_shared<GetState::Request>(), control,
      "waiting for coordinator revision before planning");
    if (call.status != ServiceStatus::kResponse) {
      return {
        SceneRevisionState::kUnavailable, {},
        call.detail.empty() ? "coordinator revision read was interrupted" : call.detail};
    }
    const auto state = detail::validate_coordinator_state(
      call.response->state, call.response->result);
    if (!state.valid) {
      if (call.response->result.success &&
        (call.response->state.operation_in_progress ||
        !call.response->state.backend_service_ready))
      {
        if (control.wait_one_slice() != detail::InterruptibleWaitResult::kReady) {
          return {
            SceneRevisionState::kUnavailable, {},
            "planning scene revision wait was canceled"};
        }
        continue;
      }
      return detail::make_planning_revision_check(expected, state, false);
    }
    return detail::make_planning_revision_check(expected, state, false);
  }
}

void RosV2SceneAdapter::Impl::release_lease_id_best_effort(
  const std::string & lease_id) noexcept
{
  if (lease_id.empty() || !context_ || !context_->is_valid() || !release_client_) {
    return;
  }
  try {
    auto request = std::make_shared<Release::Request>();
    request->lease_id = lease_id;
    (void)release_client_->async_send_request(std::move(request));
  } catch (...) {
    // Cleanup is intentionally best effort and must never block or throw.
  }
}

void RosV2SceneAdapter::Impl::release(
  const manipulation::workflow::PlanningSceneAccessProof & proof) noexcept
{
  bool owned = false;
  {
    const std::lock_guard<std::mutex> lock(active_mutex_);
    owned = active_lease_ && active_lease_->lease_id == proof.lease_id() &&
      (active_lease_->session_id.empty() ||
      active_lease_->session_id == proof.coordinator_session_id());
    if (owned) {
      active_lease_.reset();
    }
  }
  if (owned) {
    release_lease_id_best_effort(proof.lease_id());
  }
}

}  // namespace manipulation::moveit
