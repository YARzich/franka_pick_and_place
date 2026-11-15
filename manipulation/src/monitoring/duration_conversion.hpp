// Responsibility: Declares the common duration conversion used while reading monitor parameters.

#ifndef MONITORING__DURATION_CONVERSION_HPP_
#define MONITORING__DURATION_CONVERSION_HPP_

#include <chrono>
#include <string>

namespace manipulation::monitoring::detail
{

/// Convert non-negative seconds to the nanosecond representation used by ROS.
/**
 * Throws before conversion when the input is non-finite, negative, or exceeds
 * the signed 64-bit nanosecond range used by rclcpp::Duration.
 */
std::chrono::nanoseconds secondsToDuration(
  double value, const std::string & parameter_name);

}  // namespace manipulation::monitoring::detail

#endif  // MONITORING__DURATION_CONVERSION_HPP_
