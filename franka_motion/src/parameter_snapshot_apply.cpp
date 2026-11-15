// Responsibility: Installs a complete local parameter snapshot with declaration rollback.

#include "parameter_snapshot_apply.hpp"

#include <exception>
#include <string>
#include <vector>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"

namespace franka_motion::detail
{

std::optional<std::string> apply_parameter_snapshot_atomically(
  const rclcpp::Node::SharedPtr & node,
  const std::vector<rclcpp::Parameter> & snapshot)
{
  if (snapshot.empty()) {
    return std::nullopt;
  }

  std::vector<std::string> newly_declared_names;
  newly_declared_names.reserve(snapshot.size());
  const auto rollback_declarations = [&node, &newly_declared_names]() {
      for (auto name = newly_declared_names.rbegin();
        name != newly_declared_names.rend(); ++name)
      {
        try {
          node->undeclare_parameter(*name);
        } catch (const std::exception & error) {
          RCLCPP_ERROR(
            node->get_logger(), "Cannot roll back parameter declaration %s: %s",
            name->c_str(), error.what());
        }
      }
    };

  try {
    for (const auto & parameter : snapshot) {
      if (!node->has_parameter(parameter.get_name())) {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.dynamic_typing = true;
        node->declare_parameter(
          parameter.get_name(), rclcpp::ParameterValue{}, descriptor);
        newly_declared_names.push_back(parameter.get_name());
      }
    }
    const auto result = node->set_parameters_atomically(snapshot);
    if (result.successful) {
      return std::nullopt;
    }
    rollback_declarations();
    return result.reason.empty() ?
           "parameter callback rejected snapshot" : result.reason;
  } catch (const std::exception & error) {
    rollback_declarations();
    return error.what();
  }
}

}  // namespace franka_motion::detail
