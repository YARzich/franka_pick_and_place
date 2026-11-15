// Responsibility: Implements strict coordinator identifiers, duration conversion and diff
//   validation helpers.

#include "manipulation/scene/coordinator_utils.hpp"

#include <chrono>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>

namespace manipulation::scene
{

SteadyDurationResult parse_positive_steady_duration(
  const builtin_interfaces::msg::Duration & duration)
{
  if (duration.sec < 0) {
    return {false, {}, "duration seconds must be non-negative"};
  }
  if (duration.nanosec >= 1'000'000'000U) {
    return {false, {}, "duration nanoseconds must be less than one billion"};
  }

  const auto seconds = std::chrono::seconds{duration.sec};
  const auto nanoseconds = std::chrono::nanoseconds{duration.nanosec};
  const auto value =
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(seconds) +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(nanoseconds);
  if (value <= std::chrono::steady_clock::duration::zero()) {
    return {false, {}, "duration must be positive"};
  }
  return {true, value, {}};
}

std::optional<builtin_interfaces::msg::Duration> to_duration_message(
  const std::chrono::steady_clock::duration duration)
{
  if (duration <= std::chrono::steady_clock::duration::zero()) {
    return std::nullopt;
  }

  const auto total_nanoseconds =
    std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
  if (total_nanoseconds <= 0) {
    return std::nullopt;
  }
  const auto seconds = total_nanoseconds / 1'000'000'000LL;
  if (seconds > std::numeric_limits<std::int32_t>::max()) {
    return std::nullopt;
  }

  builtin_interfaces::msg::Duration result;
  result.sec = static_cast<std::int32_t>(seconds);
  result.nanosec = static_cast<std::uint32_t>(
    total_nanoseconds % 1'000'000'000LL);
  return result;
}

BackendOperationResult make_backend_operation_result(
  const BackendCallOutcome outcome,
  std::string message)
{
  switch (outcome) {
    case BackendCallOutcome::kSuccess:
      return {true, std::move(message), RegistryError::kNone};
    case BackendCallOutcome::kUnavailable:
    case BackendCallOutcome::kRejected:
      return {false, std::move(message), RegistryError::kBackendFailed};
    case BackendCallOutcome::kTimeout:
      return {false, std::move(message), RegistryError::kTimeout};
    case BackendCallOutcome::kInternalError:
      return {false, std::move(message), RegistryError::kInternalError};
  }
  return {false, "unknown backend call outcome", RegistryError::kInternalError};
}

std::uint16_t to_interface_error(const RegistryError error)
{
  using Result = manipulation_msgs::msg::PlanningSceneOperationResult;
  switch (error) {
    case RegistryError::kNone:
      return Result::NONE;
    case RegistryError::kInvalidRequest:
      return Result::INVALID_REQUEST;
    case RegistryError::kResourceConflict:
      return Result::RESOURCE_CONFLICT;
    case RegistryError::kLeaseNotFound:
      return Result::LEASE_NOT_FOUND;
    case RegistryError::kLeaseExpired:
      return Result::LEASE_EXPIRED;
    case RegistryError::kNotOwner:
      return Result::NOT_OWNER;
    case RegistryError::kRevisionMismatch:
      return Result::REVISION_MISMATCH;
    case RegistryError::kBackendFailed:
      return Result::BACKEND_FAILED;
    case RegistryError::kTimeout:
      return Result::TIMEOUT;
    case RegistryError::kSessionMismatch:
      return Result::SESSION_MISMATCH;
    case RegistryError::kIdempotencyConflict:
      return Result::IDEMPOTENCY_CONFLICT;
    case RegistryError::kOperationInProgress:
      return Result::OPERATION_IN_PROGRESS;
    case RegistryError::kOperationTicketInvalid:
      return Result::OPERATION_TICKET_INVALID;
    case RegistryError::kOperationCapacityExceeded:
      return Result::OPERATION_CAPACITY_EXCEEDED;
    case RegistryError::kReconciliationRequired:
      return Result::RECONCILIATION_REQUIRED;
    case RegistryError::kDependencyUnavailable:
      return Result::DEPENDENCY_UNAVAILABLE;
    case RegistryError::kResourceQuarantined:
      return Result::RESOURCE_QUARANTINED;
    case RegistryError::kInternalError:
      return Result::INTERNAL_ERROR;
  }
  return Result::INTERNAL_ERROR;
}

manipulation_msgs::msg::PlanningSceneOperationResult to_interface_result(
  const RegistryStatus & status)
{
  manipulation_msgs::msg::PlanningSceneOperationResult result;
  result.success = status.success;
  result.error_code = to_interface_error(status.error);
  result.revision = status.revision;
  result.message = status.message;
  return result;
}

std::optional<manipulation_msgs::msg::PlanningSceneCoordinatorState>
to_interface_state(const RegistrySnapshot & snapshot)
{
  if (snapshot.active_lease_count > std::numeric_limits<std::uint32_t>::max() ||
    snapshot.operation_record_count > std::numeric_limits<std::uint32_t>::max() ||
    snapshot.operation_record_capacity > std::numeric_limits<std::uint32_t>::max() ||
    snapshot.quarantined_resource_keys.size() > std::numeric_limits<std::uint32_t>::max())
  {
    return std::nullopt;
  }

  manipulation_msgs::msg::PlanningSceneCoordinatorState result;
  result.session_id = snapshot.session_id;
  result.revision = snapshot.revision;
  result.active_lease_count = static_cast<std::uint32_t>(snapshot.active_lease_count);
  result.operation_record_count = static_cast<std::uint32_t>(
    snapshot.operation_record_count);
  result.operation_record_capacity = static_cast<std::uint32_t>(
    snapshot.operation_record_capacity);
  result.operation_capacity_exhausted = snapshot.operation_capacity_exhausted;
  result.operation_in_progress = snapshot.operation_in_progress;
  result.expired_lease_count = snapshot.expired_lease_count;
  result.quarantined_resource_count = static_cast<std::uint32_t>(
    snapshot.quarantined_resource_keys.size());
  result.quarantined_resource_keys = snapshot.quarantined_resource_keys;
  return result;
}

}  // namespace manipulation::scene
