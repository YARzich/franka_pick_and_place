// Responsibility: Provides strict declaration and validation of required non-empty string
//   parameters.

#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"

namespace franka_motion
{
/// Declare a typed parameter only when absent, preserving launch/CLI overrides.
///
/// The node is borrowed and must not be concurrently mutating the same
/// declaration. Type mismatch follows rclcpp's parameter exception contract.
template<typename T>
T declareOrGetParameter(
  const rclcpp::Node::SharedPtr & node,
  const std::string & name,
  const T & default_value)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter<T>(name, default_value);
  }
  return node->get_parameter(name).get_parameter_value().get<T>();
}
}  // namespace franka_motion
