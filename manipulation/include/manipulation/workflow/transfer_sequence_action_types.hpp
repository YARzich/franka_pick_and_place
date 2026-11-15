// Responsibility: Defines action-independent state shared by ordered transfer lifecycles.

#ifndef MANIPULATION__WORKFLOW__TRANSFER_SEQUENCE_ACTION_TYPES_HPP_
#define MANIPULATION__WORKFLOW__TRANSFER_SEQUENCE_ACTION_TYPES_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "manipulation/task/task_status.hpp"
#include "manipulation/workflow/single_goal_task_worker.hpp"
#include "manipulation/workflow/task_stop_cause.hpp"
#include "manipulation/workflow/transfer_execution_context.hpp"

namespace manipulation::workflow
{

/** Complete record of one ordered transfer run. */
struct TransferSequenceResult
{
  manipulation::task::TaskOutcome outcome;
  std::size_t completed_step_count{0U};
  std::optional<std::size_t> failed_step_index;
  std::vector<manipulation::task::TaskOutcome> step_outcomes;
};

/** Sequence-level progress paired with the unmodified active-step state. */
struct TransferSequenceFeedbackSnapshot
{
  std::size_t step_index{0U};
  std::size_t step_count{0U};
  float overall_progress{0.0F};
  manipulation::task::TaskStateSnapshot current_state;
  std::string active_object_id;
  TransferWaitingReason waiting_reason{TransferWaitingReason::kNone};
};

/// Action-independent timing policy owned by the ordered-transfer lifecycle core.
struct TransferSequenceLifecycleConfig
{
  using Clock = std::chrono::steady_clock;

  Clock::duration heartbeat_period{std::chrono::milliseconds{25}};
  Clock::duration default_timeout{std::chrono::seconds{120}};

  /// Bounded deterministic pre-worker window used only by lifecycle fault tests.
  Clock::duration accepted_start_delay{Clock::duration::zero()};
};

/// Fast side-effect-free admission run while the shared task slot is provisionally held.
struct TransferSequenceAdmission
{
  bool accepted{false};
  /// Best-known state before worker preflight; unknown is valid before staging.
  manipulation::task::ObjectDisposition initial_disposition{
    manipulation::task::ObjectDisposition::kUnknown};
  std::string reason;
};

/// Complete backend return including the steady duration of every started step.
struct TransferSequenceExecution
{
  TransferSequenceResult result;
  std::vector<std::chrono::steady_clock::duration> step_elapsed;
};

using TransferSequenceRequestStopCallback =
  std::function<void(TaskStopCause)>;

/// Immutable machine-readable action lifecycle snapshot for diagnostics.
struct TransferSequenceActionServerSnapshot
{
  bool goal_reserved{false};
  bool active{false};
  manipulation::task::TaskPhase phase{manipulation::task::TaskPhase::kIdle};
  std::chrono::steady_clock::duration elapsed{};
  std::uint32_t sequence_step_index{0U};
  std::uint32_t sequence_step_count{0U};
  float overall_progress{0.0F};
  bool paused{false};
  bool pause_pending{false};
  TaskStopCause stop_cause{TaskStopCause::kNone};
  TaskWorkerState worker_state{TaskWorkerState::kIdle};
  bool backend_invoked{false};
  bool completion_pending{false};
};

}  // namespace manipulation::workflow

#endif  // MANIPULATION__WORKFLOW__TRANSFER_SEQUENCE_ACTION_TYPES_HPP_
