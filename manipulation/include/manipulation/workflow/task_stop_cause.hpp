// Responsibility: Defines action-independent cooperative task stop states.

#ifndef MANIPULATION__WORKFLOW__TASK_STOP_CAUSE_HPP_
#define MANIPULATION__WORKFLOW__TASK_STOP_CAUSE_HPP_

#include <cstdint>

namespace manipulation::workflow
{

/// First terminal stop cause owned by a task action lifecycle.
enum class TaskStopCause : std::uint8_t
{
  kNone,
  kClientCanceled,
  kDeadlineExceeded,
  kShutdown,
};

/// Result of a backend operation that cooperates with pause/cancel/deadline.
enum class CooperativeTaskResult : std::uint8_t
{
  kContinue,
  kStopped,
};

}  // namespace manipulation::workflow

#endif  // MANIPULATION__WORKFLOW__TASK_STOP_CAUSE_HPP_
