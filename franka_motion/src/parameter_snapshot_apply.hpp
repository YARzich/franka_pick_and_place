// Responsibility: Declares atomic installation of one local ROS parameter snapshot.

#ifndef PARAMETER_SNAPSHOT_APPLY_HPP_
#define PARAMETER_SNAPSHOT_APPLY_HPP_

#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace franka_motion::detail
{

/**
 * Apply a complete snapshot or leave the destination without new declarations.
 *
 * Returns an operator-readable error when declaration or the atomic parameter
 * callback fails. Existing parameters retain the values enforced by rclcpp's
 * atomic update contract.
 */
std::optional<std::string> apply_parameter_snapshot_atomically(
  const rclcpp::Node::SharedPtr & node,
  const std::vector<rclcpp::Parameter> & snapshot);

}  // namespace franka_motion::detail

#endif  // PARAMETER_SNAPSHOT_APPLY_HPP_
