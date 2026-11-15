// Responsibility: Exposes the narrow context-construction seam used by lifecycle code and
//   downstream backend tests.

#ifndef MANIPULATION__WORKFLOW__DETAIL__TRANSFER_SEQUENCE_TASK_CONTEXT_FACTORY_HPP_
#define MANIPULATION__WORKFLOW__DETAIL__TRANSFER_SEQUENCE_TASK_CONTEXT_FACTORY_HPP_

#include "manipulation/workflow/transfer_sequence_task_context.hpp"

namespace manipulation::workflow::detail
{

/** Construction boundary for lifecycle-owned borrowed context ports. */
class TransferSequenceTaskContextFactory final
{
public:
  [[nodiscard]] static TransferSequenceTaskContext create(
    manipulation::task::ExecutionControl & control,
    TransferSequenceTaskContext::TimePoint deadline,
    TransferSequenceTaskContext::FeedbackSink feedback_sink,
    TransferSequenceTaskContext::StopCauseProvider stop_cause_provider,
    TransferSequenceTaskContext::DeadlineCallback deadline_callback,
    TransferSequenceTaskContext::OperationGate operation_gate,
    TransferSequenceTaskContext::OperationGate maintenance_operation_gate);
};

}  // namespace manipulation::workflow::detail

#endif  // MANIPULATION__WORKFLOW__DETAIL__TRANSFER_SEQUENCE_TASK_CONTEXT_FACTORY_HPP_
