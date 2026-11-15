// Responsibility: Validates and normalizes rich sequence evidence before terminal delivery.

#ifndef WORKFLOW__DETAIL__TRANSFER_SEQUENCE_EXECUTION_POLICY_HPP_
#define WORKFLOW__DETAIL__TRANSFER_SEQUENCE_EXECUTION_POLICY_HPP_

#include <cstddef>
#include <optional>
#include <string>
#include <string_view>

#include "manipulation/workflow/transfer_sequence_action_types.hpp"

namespace manipulation::workflow::detail
{

/// Pure policy result retained separately from ROS action lifecycle delivery.
struct TransferSequenceExecutionPolicyResult
{
  TransferSequenceExecution execution;
  /// Present when invalid backend evidence was replaced by fail-closed evidence.
  std::optional<std::string> contract_error;
};

/**
 * Validate backend evidence and apply the authoritative outer stop cause.
 *
 * Invalid rich results are discarded and replaced by an INTERNAL_ERROR with
 * unknown disposition. An accepted cancel, deadline, or shutdown then owns the
 * terminal error while preserving valid disposition, recovery, cleanup detail,
 * and physical-command quarantine. Failed-step evidence is aligned with that
 * outer terminal outcome. Malformed evidence also preserves any conservative
 * quarantine requirement while discarding contradictory step details. The
 * action label is used only to identify the owning facade in diagnostics.
 */
[[nodiscard]] TransferSequenceExecutionPolicyResult
apply_transfer_sequence_execution_policy(
  TransferSequenceExecution execution,
  std::size_t requested_step_count,
  TaskStopCause stop_cause,
  std::string_view action_label);

}  // namespace manipulation::workflow::detail

#endif  // WORKFLOW__DETAIL__TRANSFER_SEQUENCE_EXECUTION_POLICY_HPP_
