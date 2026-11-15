// Responsibility: Implements validated task outcomes and shared terminal-state predicates.

#include "manipulation/task/task_status.hpp"

#include <stdexcept>
#include <utility>

namespace manipulation::task
{

TaskOutcome TaskOutcome::make_success(ObjectDisposition disposition, std::string detail)
{
  return TaskOutcome(
    TaskErrorCode::kNone, disposition, 0U, std::move(detail),
    ResourceReleaseDisposition::kReleaseSafe);
}

TaskOutcome TaskOutcome::make_success(
  const ObjectDisposition disposition,
  const std::uint16_t recovery_count,
  std::string detail)
{
  return TaskOutcome(
    TaskErrorCode::kNone, disposition, recovery_count, std::move(detail),
    ResourceReleaseDisposition::kReleaseSafe);
}

TaskOutcome TaskOutcome::make_failure(
  TaskErrorCode error_code, ObjectDisposition disposition,
  std::uint16_t recovery_count, std::string detail,
  const ResourceReleaseDisposition resource_release)
{
  if (error_code == TaskErrorCode::kNone) {
    throw std::invalid_argument("A failed task outcome requires a non-success error code");
  }
  return TaskOutcome(
    error_code, disposition, recovery_count, std::move(detail), resource_release);
}

bool TaskOutcome::succeeded() const noexcept
{
  return error_code_ == TaskErrorCode::kNone;
}

TaskErrorCode TaskOutcome::error_code() const noexcept
{
  return error_code_;
}

ObjectDisposition TaskOutcome::disposition() const noexcept
{
  return disposition_;
}

std::uint16_t TaskOutcome::recovery_count() const noexcept
{
  return recovery_count_;
}

ResourceReleaseDisposition TaskOutcome::resource_release() const noexcept
{
  return resource_release_;
}

bool TaskOutcome::resources_release_safe() const noexcept
{
  return resource_release_ == ResourceReleaseDisposition::kReleaseSafe;
}

const std::string & TaskOutcome::detail() const noexcept
{
  return detail_;
}

TaskOutcome::TaskOutcome(
  TaskErrorCode error_code, ObjectDisposition disposition,
  std::uint16_t recovery_count, std::string detail,
  const ResourceReleaseDisposition resource_release)
: error_code_(error_code),
  disposition_(disposition),
  recovery_count_(recovery_count),
  detail_(std::move(detail)),
  resource_release_(resource_release)
{
}

bool is_terminal(TaskPhase phase) noexcept
{
  return phase == TaskPhase::kCompleted || phase == TaskPhase::kCanceled ||
         phase == TaskPhase::kFailed;
}

bool requires_attention(ObjectDisposition disposition) noexcept
{
  return disposition == ObjectDisposition::kUnknown ||
         disposition == ObjectDisposition::kHoldingObject ||
         disposition == ObjectDisposition::kSceneUncertain;
}

}  // namespace manipulation::task
