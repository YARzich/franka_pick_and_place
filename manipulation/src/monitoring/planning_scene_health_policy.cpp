// Responsibility: Implements PlanningScene health classification from service and freshness
//   evidence.

#include "manipulation/monitoring/planning_scene_health_policy.hpp"

#include <algorithm>
#include <chrono>
#include <stdexcept>

namespace manipulation::monitoring
{
namespace
{

void addIssue(ComponentHealth & component, HealthIssueCode issue, HealthLevel level)
{
  if (std::find(component.issues.begin(), component.issues.end(), issue) ==
    component.issues.end())
  {
    component.issues.push_back(issue);
  }
  component.level = std::max(component.level, level);
}

ComponentHealth classifyFreshness(const HealthInputs & inputs)
{
  const auto & freshness = inputs.planning_scene_state;
  ComponentHealth component;
  component.id = HealthComponentId::kPlanningScene;
  component.required = true;
  component.level = freshness.level;
  component.observed = freshness.observed;
  component.startup_grace_active = freshness.startup_grace_active;
  component.age_semantics = freshness.observed ?
    ObservationAgeSemantics::kSinceLastObservation :
    ObservationAgeSemantics::kSinceMonitorStart;
  component.observation_age = freshness.age;
  const bool state_is_stale =
    freshness.level == HealthLevel::kStale || freshness.level == HealthLevel::kError;

  if (!freshness.observed) {
    if (component.level == HealthLevel::kOk) {
      component.level = freshness.startup_grace_active ?
        HealthLevel::kWarn : HealthLevel::kStale;
    }
    addIssue(
      component, HealthIssueCode::kPlanningSceneStateUnavailable,
      component.level);
    component.detail = "PlanningScene coordinator state is unavailable";
  } else if (freshness.level == HealthLevel::kWarn) {
    addIssue(
      component, HealthIssueCode::kPlanningSceneStateDelayed,
      HealthLevel::kWarn);
    component.detail = "PlanningScene coordinator state is delayed";
  } else if (state_is_stale) {
    addIssue(
      component, HealthIssueCode::kPlanningSceneStateStale,
      freshness.level);
    component.detail = "PlanningScene coordinator state is stale";
  } else {
    component.detail = "PlanningScene coordinator state is fresh";
  }
  return component;
}

}  // namespace

ComponentHealth classifyPlanningScene(const HealthInputs & inputs)
{
  auto component = classifyFreshness(inputs);
  if (!inputs.planning_scene_service_available) {
    const auto level = inputs.startup_grace_active ?
      HealthLevel::kWarn : HealthLevel::kError;
    addIssue(component, HealthIssueCode::kPlanningSceneCoordinatorUnavailable, level);
    component.detail = "PlanningScene coordinator service is unavailable";
  }

  // Internal flags are trustworthy only while the typed state snapshot is fresh.
  if (inputs.planning_scene_service_available &&
    inputs.planning_scene_state.level == HealthLevel::kOk)
  {
    if (!inputs.planning_scene_backend_ready) {
      addIssue(
        component, HealthIssueCode::kPlanningSceneBackendUnavailable,
        HealthLevel::kError);
      component.detail = "PlanningScene authoritative backend is unavailable";
    }
    if (inputs.planning_scene_backend_error_code != 0U) {
      addIssue(component, HealthIssueCode::kPlanningSceneBackendError, HealthLevel::kError);
      component.detail = "PlanningScene coordinator reports a backend error";
    }
    if (inputs.planning_scene_reconciliation_required) {
      addIssue(
        component, HealthIssueCode::kPlanningSceneReconciliationRequired,
        HealthLevel::kError);
      component.detail = "PlanningScene authoritative reconciliation is required";
    }
    if (inputs.planning_scene_unreconciled_lease_expiry_count > 0U) {
      addIssue(component, HealthIssueCode::kPlanningSceneLeaseExpired, HealthLevel::kError);
      component.detail = "PlanningScene lease expiry has not been reconciled";
    }
    if (inputs.planning_scene_operation_capacity_exhausted) {
      addIssue(
        component, HealthIssueCode::kPlanningSceneOperationCapacityExhausted,
        HealthLevel::kError);
      component.detail = "PlanningScene idempotency ledger is full";
    }
  }

  if (inputs.planning_scene_service_available &&
    inputs.planning_scene_state.level == HealthLevel::kOk &&
    inputs.planning_scene_operation_in_progress)
  {
    if (inputs.planning_scene_operation_stuck_after <=
      std::chrono::nanoseconds::zero())
    {
      throw std::invalid_argument(
              "PlanningScene operation stuck threshold must be positive");
    }
    const bool stuck = inputs.planning_scene_operation_age &&
      *inputs.planning_scene_operation_age >= inputs.planning_scene_operation_stuck_after;
    if (stuck) {
      addIssue(
        component, HealthIssueCode::kPlanningSceneOperationStuck,
        HealthLevel::kError);
      component.detail = "PlanningScene operation exceeded its steady-time limit";
    } else {
      addIssue(
        component, HealthIssueCode::kPlanningSceneOperationInProgress,
        HealthLevel::kWarn);
      component.detail = "PlanningScene operation is in progress";
    }
  }
  return component;
}

}  // namespace manipulation::monitoring
