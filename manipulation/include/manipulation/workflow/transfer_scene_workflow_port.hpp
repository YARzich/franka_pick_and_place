// Responsibility: Defines goal-scoped scene lease evidence and operations for V2 orchestration.

#ifndef MANIPULATION__WORKFLOW__TRANSFER_SCENE_WORKFLOW_PORT_HPP_
#define MANIPULATION__WORKFLOW__TRANSFER_SCENE_WORKFLOW_PORT_HPP_

#include <vector>

#include "manipulation/workflow/transfer_workflow_ports.hpp"

namespace manipulation::workflow
{

/** Full immutable scene read and matching mutation authorization evidence. */
class PlanningSceneLease final
{
public:
  [[nodiscard]] static PlanningSceneLease create(
    PlanningSceneSnapshot snapshot,
    PlanningSceneAccessProof access_proof);

  [[nodiscard]] const PlanningSceneSnapshot & snapshot() const noexcept;
  [[nodiscard]] const PlanningSceneAccessProof & access_proof() const noexcept;

private:
  PlanningSceneLease(
    PlanningSceneSnapshot snapshot,
    PlanningSceneAccessProof access_proof);

  PlanningSceneSnapshot snapshot_;
  PlanningSceneAccessProof access_proof_;
};

using PlanningSceneLeaseResult = ProviderResult<PlanningSceneLease>;
using PlanningSceneAccessProofResult = ProviderResult<PlanningSceneAccessProof>;

/**
 * Goal-scoped scene ownership port used by the pure resolved workflow.
 *
 * Acquire covers every unique object instance in the accepted goal. Renewal
 * extends ownership without changing the scene revision. Revalidation returns
 * a fresh complete read and matching proof after possible scene mutations.
 */
class TransferSceneWorkflowPort
{
public:
  virtual ~TransferSceneWorkflowPort() = default;

  [[nodiscard]] virtual PlanningSceneLeaseResult acquire(
    const std::vector<ObjectInstance> & instances,
    TransferExecutionContext & context) = 0;

  [[nodiscard]] virtual PlanningSceneAccessProofResult renew(
    const PlanningSceneAccessProof & access_proof,
    TransferExecutionContext & context) = 0;

  /**
   * Authoritatively stage catalog geometry at the fresh observed object pose.
   *
   * A successful result includes the complete post-mutation scene and the new
   * matching revision. Grasp generation must consume this returned snapshot.
   */
  [[nodiscard]] virtual PlanningSceneLeaseResult stage_observation(
    const ObjectInstance & instance,
    const ObjectProfile & profile,
    const ObjectObservation & observation,
    const PlanningSceneAccessProof & access_proof,
    TransferExecutionContext & context) = 0;

  /**
   * Read the current scene under the same lease/session.
   *
   * The supplied proof revision may predate mutations made by the execution
   * strategy under that lease; the successful result supplies current evidence.
   */
  [[nodiscard]] virtual PlanningSceneLeaseResult revalidate(
    const PlanningSceneAccessProof & access_proof,
    TransferExecutionContext & context) = 0;

  /** Best-effort, non-blocking release suitable for an RAII cleanup path. */
  virtual void release(const PlanningSceneAccessProof & access_proof) noexcept = 0;

  /// Non-blocking propagation to any coordinator request in progress.
  virtual void request_stop() noexcept = 0;
};

}  // namespace manipulation::workflow

#endif  // MANIPULATION__WORKFLOW__TRANSFER_SCENE_WORKFLOW_PORT_HPP_
