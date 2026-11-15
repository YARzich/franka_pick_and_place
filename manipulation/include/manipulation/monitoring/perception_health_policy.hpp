// Responsibility: Validates configured perception diagnostic bindings and maps them to health
//   components without depending on a detector or pose-estimator backend.

#ifndef MANIPULATION__MONITORING__PERCEPTION_HEALTH_POLICY_HPP_
#define MANIPULATION__MONITORING__PERCEPTION_HEALTH_POLICY_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "manipulation/monitoring/health_aggregator.hpp"

namespace manipulation::monitoring
{

/// Stable mapping from an exact diagnostic task name to a typed component.
struct PerceptionPipelineDescriptor
{
  std::string task_name;
  HealthComponentId component_id;
};

/// Build typed descriptors from aligned ROS parameter arrays.
[[nodiscard]] std::vector<PerceptionPipelineDescriptor>
makePerceptionPipelineDescriptors(
  const std::vector<std::string> & task_names,
  const std::vector<std::int64_t> & component_ids);

/// Resolve an exact configured task name; prefixes and suffix matches are rejected.
[[nodiscard]] std::optional<PerceptionPipelineDescriptor>
findPerceptionPipeline(
  const std::vector<PerceptionPipelineDescriptor> & pipelines,
  std::string_view task_name);

/// Validate exact binding rows and the unique required-task selection.
/**
 * A stable task may repeat to bind mutually exclusive producers. Full status
 * names remain globally unique, so every received status has one owner.
 */
void validatePerceptionDiagnosticBindings(
  const std::vector<PerceptionPipelineDescriptor> & pipelines,
  const std::vector<std::string> & status_names,
  const std::vector<std::string> & required_tasks);

/// Convert a diagnostic level byte; unknown values fail closed as ERROR.
[[nodiscard]] HealthLevel healthLevelFromDiagnostic(std::uint8_t level) noexcept;

/// Classify freshness and the upstream-reported level without reading clocks.
[[nodiscard]] ComponentHealth classifyPerceptionPipeline(
  const PerceptionPipelineInput & input);

}  // namespace manipulation::monitoring

#endif  // MANIPULATION__MONITORING__PERCEPTION_HEALTH_POLICY_HPP_
