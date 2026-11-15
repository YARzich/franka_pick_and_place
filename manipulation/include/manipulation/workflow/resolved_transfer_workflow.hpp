// Responsibility: Declares pure V2 intent resolution and ordered strategy orchestration.

#ifndef MANIPULATION__WORKFLOW__RESOLVED_TRANSFER_WORKFLOW_HPP_
#define MANIPULATION__WORKFLOW__RESOLVED_TRANSFER_WORKFLOW_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <string_view>

#include "manipulation/workflow/strategy_registry.hpp"
#include "manipulation/workflow/transfer_object_sequence_v2_goal_validation.hpp"
#include "manipulation/workflow/transfer_scene_workflow_port.hpp"
#include "manipulation/workflow/transfer_sequence_action_types.hpp"
#include "manipulation/workflow/wait_lifecycle_logger.hpp"
#include "manipulation_msgs/action/transfer_object_sequence_v2.hpp"

namespace manipulation::workflow
{

struct ResolvedTransferWorkflowOptions
{
  using Clock = std::chrono::steady_clock;
  using NowFunction = std::function<Clock::time_point()>;

  TransferObjectSequenceV2GoalValidationPolicy validation;
  Clock::duration scene_renew_period{std::chrono::seconds{10}};
  NowFunction now = []() {return Clock::now();};
  std::shared_ptr<WaitLifecycleLogger> wait_logger;
};

using ResolvedTransferFeedbackCallback =
  std::function<void(TransferSequenceFeedbackSnapshot)>;

/**
 * Resolve one accepted V2 goal through explicit catalog/provider/scene ports.
 *
 * The workflow owns no ROS entities and imposes no whole-goal deadline. It is
 * single-execution: an external lifecycle owns admission serialization and
 * adapts its cooperative control context and typed feedback callback.
 */
class ResolvedTransferWorkflow final
{
public:
  using Goal = manipulation_msgs::action::TransferObjectSequenceV2::Goal;

  ResolvedTransferWorkflow(
    ResolvedTransferWorkflowOptions options,
    const ObjectCatalog & catalog,
    ObservationProvider & observation_provider,
    GraspProvider & grasp_provider,
    TransferSceneWorkflowPort & scene,
    StrategyRegistry & strategies,
    std::string_view selected_strategy_id);
  ~ResolvedTransferWorkflow();

  ResolvedTransferWorkflow(const ResolvedTransferWorkflow &) = delete;
  ResolvedTransferWorkflow & operator=(const ResolvedTransferWorkflow &) = delete;
  ResolvedTransferWorkflow(ResolvedTransferWorkflow &&) = delete;
  ResolvedTransferWorkflow & operator=(ResolvedTransferWorkflow &&) = delete;

  /** Side-effect-free wire and catalog admission. */
  [[nodiscard]] TransferSequenceAdmission admit(const Goal & goal) const;

  /** Execute all steps in order with goal-scoped lease cleanup. */
  [[nodiscard]] TransferSequenceExecution execute(
    const Goal & goal,
    TransferExecutionContext & context,
    ResolvedTransferFeedbackCallback feedback = {},
    std::string goal_id = "unidentified-goal");

  /** Forward an external cancel/shutdown request to every active port. */
  void request_stop() noexcept;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace manipulation::workflow

#endif  // MANIPULATION__WORKFLOW__RESOLVED_TRANSFER_WORKFLOW_HPP_
