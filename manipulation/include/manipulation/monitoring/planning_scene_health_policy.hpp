// Responsibility: Classifies PlanningScene availability, freshness and coordinator integrity
//   evidence.

#ifndef MANIPULATION__MONITORING__PLANNING_SCENE_HEALTH_POLICY_HPP_
#define MANIPULATION__MONITORING__PLANNING_SCENE_HEALTH_POLICY_HPP_

#include "manipulation/monitoring/health_aggregator.hpp"

namespace manipulation::monitoring
{

/// Classify coordinator freshness, backend consistency and bounded capacity.
[[nodiscard]] ComponentHealth classifyPlanningScene(const HealthInputs & inputs);

}  // namespace manipulation::monitoring

#endif  // MANIPULATION__MONITORING__PLANNING_SCENE_HEALTH_POLICY_HPP_
