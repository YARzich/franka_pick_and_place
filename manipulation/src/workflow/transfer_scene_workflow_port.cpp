// Responsibility: Validates that scene snapshots and access proofs describe one lease revision.

#include "manipulation/workflow/transfer_scene_workflow_port.hpp"

#include <stdexcept>
#include <utility>

namespace manipulation::workflow
{

PlanningSceneLease PlanningSceneLease::create(
  PlanningSceneSnapshot snapshot,
  PlanningSceneAccessProof access_proof)
{
  if (snapshot.coordinator_session_id() != access_proof.coordinator_session_id() ||
    snapshot.revision() != access_proof.revision())
  {
    throw std::invalid_argument(
            "planning scene lease snapshot and access proof identify different revisions");
  }
  return PlanningSceneLease(std::move(snapshot), std::move(access_proof));
}

PlanningSceneLease::PlanningSceneLease(
  PlanningSceneSnapshot snapshot,
  PlanningSceneAccessProof access_proof)
: snapshot_(std::move(snapshot)), access_proof_(std::move(access_proof))
{
}

const PlanningSceneSnapshot & PlanningSceneLease::snapshot() const noexcept
{
  return snapshot_;
}

const PlanningSceneAccessProof & PlanningSceneLease::access_proof() const noexcept
{
  return access_proof_;
}

}  // namespace manipulation::workflow
