// Responsibility: Connects unlimited V2 lifecycle controls to pure transfer orchestration.

#include "manipulation/workflow/resolved_transfer_workflow_action_adapter.hpp"

#include <utility>

namespace manipulation::workflow
{
namespace
{

TransferExecutionControlResult adapt(const CooperativeTaskResult result) noexcept
{
  return result == CooperativeTaskResult::kContinue ?
         TransferExecutionControlResult::kContinue :
         TransferExecutionControlResult::kStopped;
}

}  // namespace

TransferSequenceExecutionContextAdapter::TransferSequenceExecutionContextAdapter(
  TransferSequenceTaskContext & context) noexcept
: context_(context)
{
}

TransferExecutionControlResult TransferSequenceExecutionContextAdapter::checkpoint()
{
  return adapt(context_.checkpoint());
}

bool TransferSequenceExecutionContextAdapter::stop_requested() const noexcept
{
  return context_.stop_requested();
}

TransferExecutionControlResult TransferSequenceExecutionContextAdapter::wait_until(
  const TimePoint deadline)
{
  return adapt(context_.wait_until(deadline));
}

TransferExecutionControlResult TransferSequenceExecutionContextAdapter::heartbeat()
{
  return adapt(context_.heartbeat());
}

TaskOperationGateResult TransferSequenceExecutionContextAdapter::run_operation(
  const std::function<void()> & operation)
{
  return context_.run_operation(operation);
}

TaskOperationGateResult
TransferSequenceExecutionContextAdapter::run_maintenance_operation(
  const std::function<void()> & operation)
{
  return context_.run_maintenance_operation(operation);
}

TaskStopCause TransferSequenceExecutionContextAdapter::stop_cause() const noexcept
{
  return context_.stop_cause();
}

void TransferSequenceExecutionContextAdapter::report_feedback(
  manipulation::task::TaskStateSnapshot)
{
}

void TransferSequenceExecutionContextAdapter::report_waiting(
  const TransferWaitingReason,
  manipulation::task::TaskStateSnapshot)
{
}

TransferObjectSequenceV2BackendCallbacks make_resolved_transfer_workflow_callbacks(
  ResolvedTransferWorkflow & workflow,
  ResolvedTransferGoalIdProvider goal_id_provider)
{
  return {
    [&workflow](const ResolvedTransferWorkflow::Goal & goal) {
      return workflow.admit(goal);
    },
    [&workflow, goal_id_provider](
      const ResolvedTransferWorkflow::Goal & goal,
      TransferSequenceTaskContext & action_context) {
      TransferSequenceExecutionContextAdapter execution_context(action_context);
      const auto goal_id = goal_id_provider ? goal_id_provider(goal) :
        std::string{"transfer-object-sequence-v2"};
      return workflow.execute(
        goal, execution_context,
        [&action_context](TransferSequenceFeedbackSnapshot feedback) {
          action_context.report_feedback(std::move(feedback));
        },
        goal_id);
    },
    [&workflow](const TaskStopCause) {
      workflow.request_stop();
    }};
}

}  // namespace manipulation::workflow
