// Responsibility: Declares serialization of internal health state for ROS publication and services.

#ifndef MONITORING__HEALTH_MESSAGE_ADAPTER_HPP_
#define MONITORING__HEALTH_MESSAGE_ADAPTER_HPP_

#include "builtin_interfaces/msg/time.hpp"
#include "manipulation/monitoring/health_aggregator.hpp"
#include "manipulation_msgs/msg/system_health_snapshot.hpp"

namespace manipulation::monitoring::detail
{

/// Map the internal model to the stable wire contract with explicit enum maps.
[[nodiscard]] manipulation_msgs::msg::SystemHealthSnapshot toHealthMessage(
  const SystemHealth & health,
  const builtin_interfaces::msg::Time & sample_stamp);

}  // namespace manipulation::monitoring::detail

#endif  // MONITORING__HEALTH_MESSAGE_ADAPTER_HPP_
