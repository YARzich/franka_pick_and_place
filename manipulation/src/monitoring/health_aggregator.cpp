// Responsibility: Applies component health policies and derives the fail-closed aggregate result.

#include "manipulation/monitoring/health_aggregator.hpp"

#include <algorithm>
#include <set>
#include <stdexcept>
#include <utility>

#include "manipulation/monitoring/perception_health_policy.hpp"
#include "manipulation/monitoring/planning_scene_health_policy.hpp"

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

ComponentHealth classifyFreshness(
  HealthComponentId id,
  bool required,
  const FreshnessStatus & freshness,
  HealthIssueCode unavailable,
  HealthIssueCode delayed,
  HealthIssueCode stale,
  std::string healthy_detail,
  std::string unavailable_detail,
  std::string delayed_detail,
  std::string stale_detail)
{
  ComponentHealth component;
  component.id = id;
  component.required = required;
  component.level = freshness.level;
  if (!freshness.observed && component.level == HealthLevel::kOk) {
    component.level = freshness.startup_grace_active ?
      HealthLevel::kWarn : HealthLevel::kStale;
  }
  component.observed = freshness.observed;
  component.startup_grace_active = freshness.startup_grace_active;
  component.age_semantics = freshness.observed ?
    ObservationAgeSemantics::kSinceLastObservation :
    ObservationAgeSemantics::kSinceMonitorStart;
  component.observation_age = freshness.age;

  if (!freshness.observed) {
    addIssue(component, unavailable, component.level);
    component.detail = std::move(unavailable_detail);
    return component;
  }
  if (freshness.level == HealthLevel::kWarn) {
    addIssue(component, delayed, HealthLevel::kWarn);
    component.detail = std::move(delayed_detail);
    return component;
  }
  if (freshness.level == HealthLevel::kStale ||
    freshness.level == HealthLevel::kError)
  {
    addIssue(component, stale, freshness.level);
    component.detail = std::move(stale_detail);
    return component;
  }
  component.detail = std::move(healthy_detail);
  return component;
}

ComponentHealth classifyAvailability(
  HealthComponentId id,
  bool available,
  bool startup_grace_active,
  HealthIssueCode unavailable_issue,
  std::string available_detail,
  std::string unavailable_detail)
{
  ComponentHealth component;
  component.id = id;
  component.required = true;
  component.level = HealthLevel::kOk;
  component.observed = true;
  component.age_semantics = ObservationAgeSemantics::kNotApplicable;
  component.detail = available ? std::move(available_detail) : std::move(unavailable_detail);
  if (!available) {
    component.startup_grace_active = startup_grace_active;
    addIssue(
      component, unavailable_issue,
      startup_grace_active ? HealthLevel::kWarn : HealthLevel::kError);
  }
  return component;
}

ComponentHealth classifyJointStates(const HealthInputs & inputs)
{
  auto component = classifyFreshness(
    HealthComponentId::kJointStates, true, inputs.joint_states,
    HealthIssueCode::kJointStatesUnavailable,
    HealthIssueCode::kJointStatesDelayed,
    HealthIssueCode::kJointStatesStale,
    "Joint states are fresh", "Joint states are unavailable",
    "Joint states are delayed", "Joint states are stale");

  if (!inputs.joint_profile_configured) {
    addIssue(component, HealthIssueCode::kRequiredJointsMissing, HealthLevel::kError);
    component.detail = "Required joint profile is not configured";
  } else if (inputs.joint_states.observed && !inputs.missing_joints.empty()) {
    const bool use_as_summary = component.level <= HealthLevel::kError;
    addIssue(component, HealthIssueCode::kRequiredJointsMissing, HealthLevel::kError);
    if (use_as_summary) {
      component.detail = "Required joints are missing";
    }
  }
  if (inputs.joint_states.observed && inputs.joint_stamp_ns == 0) {
    addIssue(component, HealthIssueCode::kJointStampMissing, HealthLevel::kWarn);
    if (component.level == HealthLevel::kWarn) {
      component.detail = "JointState has a zero timestamp";
    }
  } else if (inputs.joint_states.observed && inputs.joint_stamp_outside_tolerance) {
    const bool use_as_summary = component.level <= HealthLevel::kError;
    addIssue(
      component, HealthIssueCode::kJointStampOutsideTolerance, HealthLevel::kError);
    if (use_as_summary) {
      component.detail = "JointState timestamp is outside tolerance";
    }
  }
  return component;
}

ComponentHealth classifyClock(const HealthInputs & inputs)
{
  if (!inputs.simulation_clock_required) {
    ComponentHealth component;
    component.id = HealthComponentId::kSimulationClock;
    component.required = false;
    component.level = HealthLevel::kOk;
    component.detail = "Simulation clock is not required";
    return component;
  }
  return classifyFreshness(
    HealthComponentId::kSimulationClock, true, inputs.simulation_clock,
    HealthIssueCode::kSimulationClockUnavailable,
    HealthIssueCode::kSimulationClockDelayed,
    HealthIssueCode::kSimulationClockStalled,
    "Simulation clock is progressing", "Simulation clock is unavailable",
    "Simulation clock is delayed", "Simulation clock is stalled");
}

ComponentHealth classifyControllers(const HealthInputs & inputs)
{
  auto component = classifyFreshness(
    HealthComponentId::kControllers, true, inputs.controllers,
    HealthIssueCode::kControllerStateUnavailable,
    HealthIssueCode::kControllerStateDelayed,
    HealthIssueCode::kControllerStateStale,
    "Required controllers are active", "Controller state is unavailable",
    "Controller state is delayed", "Controller state is stale");
  if (!inputs.controller_profile_configured) {
    addIssue(
      component, HealthIssueCode::kRequiredControllersInactive,
      HealthLevel::kError);
    component.detail = "Required controller profile is not configured";
  } else if (inputs.controllers.observed && !inputs.inactive_controllers.empty()) {
    const bool use_as_summary = component.level <= HealthLevel::kError;
    addIssue(
      component, HealthIssueCode::kRequiredControllersInactive, HealthLevel::kError);
    if (use_as_summary) {
      component.detail = "Required controllers are not active";
    }
  }
  return component;
}

}  // namespace

SystemHealth HealthAggregator::aggregate(const HealthInputs & inputs) const
{
  SystemHealth health;
  health.sequence = inputs.sequence;
  health.steady_uptime = inputs.steady_uptime;
  health.components.reserve(7U + inputs.perception_pipelines.size());
  health.components.push_back(classifyJointStates(inputs));
  health.components.push_back(classifyClock(inputs));
  health.components.push_back(classifyControllers(inputs));
  health.components.push_back(classifyAvailability(
      HealthComponentId::kRobotTf, inputs.robot_tf_available,
      inputs.startup_grace_active, HealthIssueCode::kRobotTfUnavailable,
      "Robot TF is available", "Robot TF is unavailable"));
  health.components.push_back(classifyAvailability(
      HealthComponentId::kMotionBackend, inputs.motion_backend_available,
      inputs.startup_grace_active, HealthIssueCode::kMotionBackendUnavailable,
      "Motion backend action is ready", "Motion backend action is unavailable"));
  health.components.push_back(classifyAvailability(
      HealthComponentId::kGripper, inputs.gripper_available,
      inputs.startup_grace_active, HealthIssueCode::kGripperUnavailable,
      "Gripper action is ready", "Gripper action is unavailable"));
  health.components.push_back(classifyPlanningScene(inputs));
  std::set<HealthComponentId> perception_ids;
  for (const auto & input : inputs.perception_pipelines) {
    if (!perception_ids.insert(input.id).second) {
      throw std::invalid_argument("duplicate perception component in health input");
    }
    health.components.push_back(classifyPerceptionPipeline(input));
  }

  health.overall_level = HealthLevel::kOk;
  health.motion_ready = true;
  for (const auto & component : health.components) {
    if (!component.required) {
      continue;
    }
    health.overall_level = std::max(health.overall_level, component.level);
    // WARN is intentionally not motion-ready: it represents degraded or
    // startup data that should settle before a new command is accepted.
    health.motion_ready = health.motion_ready && component.level == HealthLevel::kOk;
  }
  return health;
}

const ComponentHealth & findComponent(const SystemHealth & health, HealthComponentId id)
{
  const auto found = std::find_if(
    health.components.begin(), health.components.end(),
    [id](const ComponentHealth & component) {return component.id == id;});
  if (found == health.components.end()) {
    throw std::logic_error("health aggregate is missing a required component");
  }
  return *found;
}

}  // namespace manipulation::monitoring
