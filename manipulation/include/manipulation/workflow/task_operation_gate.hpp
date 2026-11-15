// Responsibility: Defines the neutral result of linearizing a short task operation.

#ifndef MANIPULATION__WORKFLOW__TASK_OPERATION_GATE_HPP_
#define MANIPULATION__WORKFLOW__TASK_OPERATION_GATE_HPP_

#include <cstdint>

namespace manipulation::workflow
{

/// Result of arbitrating one operation against goal pause, stop, and deadline.
enum class TaskOperationGateResult : std::uint8_t
{
  kExecuted,
  kPaused,
  kStopped,
};

}  // namespace manipulation::workflow

#endif  // MANIPULATION__WORKFLOW__TASK_OPERATION_GATE_HPP_
