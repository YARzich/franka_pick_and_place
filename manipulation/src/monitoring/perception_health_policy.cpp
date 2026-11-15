// Responsibility: Validates diagnostic bindings and classifies each perception pipeline
//   fail-closed.

#include "manipulation/monitoring/perception_health_policy.hpp"

#include <algorithm>
#include <cstddef>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>

namespace manipulation::monitoring
{
namespace
{

bool isPerceptionComponent(const HealthComponentId id) noexcept
{
  return id == HealthComponentId::kStereoPerception ||
         id == HealthComponentId::kObjectDetection ||
         id == HealthComponentId::kPoseEstimation;
}

HealthComponentId perceptionComponentFromValue(const std::int64_t value)
{
  switch (value) {
    case static_cast<std::int64_t>(HealthComponentId::kStereoPerception):
      return HealthComponentId::kStereoPerception;
    case static_cast<std::int64_t>(HealthComponentId::kObjectDetection):
      return HealthComponentId::kObjectDetection;
    case static_cast<std::int64_t>(HealthComponentId::kPoseEstimation):
      return HealthComponentId::kPoseEstimation;
    default:
      throw std::invalid_argument(
              "perception component_id must identify a perception component");
  }
}

void addIssue(ComponentHealth & component, HealthIssueCode issue, HealthLevel level)
{
  if (std::find(component.issues.begin(), component.issues.end(), issue) ==
    component.issues.end())
  {
    component.issues.push_back(issue);
  }
  component.level = std::max(component.level, level);
}

std::string defaultReportedDetail(const PerceptionPipelineInput & input)
{
  switch (input.reported_level) {
    case HealthLevel::kOk:
      return input.task_name + " reports OK";
    case HealthLevel::kWarn:
      return input.task_name + " reports WARN";
    case HealthLevel::kError:
      return input.task_name + " reports ERROR";
    case HealthLevel::kStale:
      return input.task_name + " reports STALE";
  }
  return input.task_name + " reports an invalid level";
}

}  // namespace

std::vector<PerceptionPipelineDescriptor> makePerceptionPipelineDescriptors(
  const std::vector<std::string> & task_names,
  const std::vector<std::int64_t> & component_ids)
{
  if (task_names.size() != component_ids.size()) {
    throw std::invalid_argument(
            "perception task_names and component_ids must have equal length");
  }

  std::vector<PerceptionPipelineDescriptor> result;
  result.reserve(task_names.size());
  for (std::size_t index = 0U; index < task_names.size(); ++index) {
    if (task_names[index].empty()) {
      throw std::invalid_argument("perception task name must not be empty");
    }
    result.push_back(PerceptionPipelineDescriptor{
        task_names[index], perceptionComponentFromValue(component_ids[index])});
  }
  return result;
}

std::optional<PerceptionPipelineDescriptor> findPerceptionPipeline(
  const std::vector<PerceptionPipelineDescriptor> & pipelines,
  const std::string_view task_name)
{
  const auto found = std::find_if(
    pipelines.begin(), pipelines.end(),
    [task_name](const PerceptionPipelineDescriptor & descriptor) {
      return descriptor.task_name == task_name;
    });
  if (found == pipelines.end()) {
    return std::nullopt;
  }
  return *found;
}

void validatePerceptionDiagnosticBindings(
  const std::vector<PerceptionPipelineDescriptor> & pipelines,
  const std::vector<std::string> & status_names,
  const std::vector<std::string> & required_tasks)
{
  if (pipelines.size() != status_names.size()) {
    throw std::invalid_argument(
            "perception pipelines and status_names must have equal length");
  }
  std::map<std::string, HealthComponentId> configured_tasks;
  std::set<std::string> bound_status_names;
  std::set<std::pair<std::string, std::string>> bindings;
  for (std::size_t index = 0U; index < pipelines.size(); ++index) {
    const auto & descriptor = pipelines[index];
    const auto & task_name = descriptor.task_name;
    const auto & status_name = status_names[index];
    if (task_name.empty()) {
      throw std::invalid_argument("perception task name must not be empty");
    }
    if (!isPerceptionComponent(descriptor.component_id)) {
      throw std::invalid_argument(
              "perception binding uses a non-perception component_id");
    }
    const auto [configured, inserted] = configured_tasks.emplace(
      task_name, descriptor.component_id);
    if (!inserted && configured->second != descriptor.component_id) {
      throw std::invalid_argument(
              "one perception task cannot map to multiple component_ids: " + task_name);
    }
    if (status_name.empty()) {
      throw std::invalid_argument("perception diagnostic status name must not be empty");
    }
    if (!bindings.emplace(task_name, status_name).second) {
      throw std::invalid_argument(
              "duplicate perception diagnostic binding: " + task_name + " -> " + status_name);
    }
    if (!bound_status_names.insert(status_name).second) {
      throw std::invalid_argument(
              "duplicate perception diagnostic status name: " + status_name);
    }
  }

  std::set<std::string> required;
  for (const auto & task_name : required_tasks) {
    if (!required.insert(task_name).second) {
      throw std::invalid_argument("duplicate required perception task: " + task_name);
    }
    if (configured_tasks.count(task_name) == 0U) {
      throw std::invalid_argument(
              "required perception task has no diagnostic binding: " + task_name);
    }
  }
}

HealthLevel healthLevelFromDiagnostic(std::uint8_t level) noexcept
{
  switch (level) {
    case 0U:
      return HealthLevel::kOk;
    case 1U:
      return HealthLevel::kWarn;
    case 2U:
      return HealthLevel::kError;
    case 3U:
      return HealthLevel::kStale;
    default:
      return HealthLevel::kError;
  }
}

ComponentHealth classifyPerceptionPipeline(const PerceptionPipelineInput & input)
{
  if (input.task_name.empty()) {
    throw std::invalid_argument("perception input task name must not be empty");
  }
  if (!isPerceptionComponent(input.id)) {
    throw std::invalid_argument("perception input has a non-perception component_id");
  }
  ComponentHealth component;
  component.id = input.id;
  component.required = input.required;
  component.level = input.freshness.level;
  if (!input.freshness.observed && component.level == HealthLevel::kOk) {
    component.level = input.freshness.startup_grace_active ?
      HealthLevel::kWarn : HealthLevel::kStale;
  }
  component.observed = input.freshness.observed;
  component.startup_grace_active = input.freshness.startup_grace_active;
  component.age_semantics = input.freshness.observed ?
    ObservationAgeSemantics::kSinceLastObservation :
    ObservationAgeSemantics::kSinceMonitorStart;
  component.observation_age = input.freshness.age;

  if (!input.freshness.observed) {
    addIssue(
      component, HealthIssueCode::kPerceptionDiagnosticUnavailable,
      component.level);
    component.detail = input.task_name + " diagnostic is unavailable";
    return component;
  }
  const bool freshness_is_stale =
    input.freshness.level == HealthLevel::kStale ||
    input.freshness.level == HealthLevel::kError;
  if (input.freshness.level == HealthLevel::kWarn) {
    addIssue(
      component, HealthIssueCode::kPerceptionDiagnosticDelayed,
      HealthLevel::kWarn);
    component.detail = input.task_name + " diagnostic is delayed";
  } else if (freshness_is_stale) {
    addIssue(
      component, HealthIssueCode::kPerceptionDiagnosticStale,
      input.freshness.level);
    component.detail = input.task_name + " diagnostic is stale";
  } else {
    component.detail = input.task_name + " diagnostic is fresh";
  }

  const auto before_reported_level = component.level;
  const bool reported_failure =
    input.reported_level == HealthLevel::kError ||
    input.reported_level == HealthLevel::kStale;
  if (input.reported_level == HealthLevel::kWarn) {
    addIssue(
      component, HealthIssueCode::kPerceptionPipelineReportedWarn,
      HealthLevel::kWarn);
  } else if (reported_failure) {
    addIssue(
      component, HealthIssueCode::kPerceptionPipelineReportedError,
      input.reported_level);
  }
  if (input.reported_level != HealthLevel::kOk &&
    input.reported_level >= before_reported_level)
  {
    component.detail = input.reported_detail.empty() ?
      defaultReportedDetail(input) : input.reported_detail;
  }
  return component;
}

}  // namespace manipulation::monitoring
