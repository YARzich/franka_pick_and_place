// Responsibility: Adapts the V2 action lifecycle context to the pure resolved workflow.

#ifndef MANIPULATION__WORKFLOW__RESOLVED_TRANSFER_WORKFLOW_ACTION_ADAPTER_HPP_
#define MANIPULATION__WORKFLOW__RESOLVED_TRANSFER_WORKFLOW_ACTION_ADAPTER_HPP_

#include <functional>
#include <string>

#include "manipulation/workflow/resolved_transfer_workflow.hpp"
#include "manipulation/workflow/transfer_object_sequence_v2_action_server.hpp"

namespace manipulation::workflow
{

/**
 * Borrowed control adapter used only while a V2 backend callback is executing.
 *
 * Basic TaskState feedback is intentionally not forwarded here: the runner's
 * enriched sequence callback is the sole path to the action context, preventing
 * duplicate typed feedback publication.
 */
class TransferSequenceExecutionContextAdapter final : public TransferExecutionContext
{
public:
  explicit TransferSequenceExecutionContextAdapter(
    TransferSequenceTaskContext & context) noexcept;

  [[nodiscard]] TransferExecutionControlResult checkpoint() override;
  [[nodiscard]] bool stop_requested() const noexcept override;
  [[nodiscard]] TransferExecutionControlResult wait_until(TimePoint deadline) override;
  [[nodiscard]] TransferExecutionControlResult heartbeat() override;
  [[nodiscard]] TaskOperationGateResult run_operation(
    const std::function<void()> & operation) override;
  [[nodiscard]] TaskOperationGateResult run_maintenance_operation(
    const std::function<void()> & operation) override;
  [[nodiscard]] TaskStopCause stop_cause() const noexcept override;
  void report_feedback(manipulation::task::TaskStateSnapshot feedback) override;
  void report_waiting(
    TransferWaitingReason reason,
    manipulation::task::TaskStateSnapshot feedback) override;

private:
  TransferSequenceTaskContext & context_;
};

using ResolvedTransferGoalIdProvider =
  std::function<std::string(const ResolvedTransferWorkflow::Goal &)>;

/**
 * Compose admission, execution, typed feedback, and stop propagation once.
 *
 * The workflow and optional ID provider must outlive callbacks copied into the
 * action server. An empty provider uses a stable diagnostic-only goal label.
 */
[[nodiscard]] TransferObjectSequenceV2BackendCallbacks
make_resolved_transfer_workflow_callbacks(
  ResolvedTransferWorkflow & workflow,
  ResolvedTransferGoalIdProvider goal_id_provider = {});

}  // namespace manipulation::workflow

#endif  // MANIPULATION__WORKFLOW__RESOLVED_TRANSFER_WORKFLOW_ACTION_ADAPTER_HPP_
