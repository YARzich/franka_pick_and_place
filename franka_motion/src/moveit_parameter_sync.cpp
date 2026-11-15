// Responsibility: Orchestrates MoveIt snapshot copy and validates Panda timing limits.

#include "franka_motion/moveit_parameter_sync.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include "moveit_parameter_snapshot.hpp"
#include "parameter_snapshot_client.hpp"
#include "parameter_snapshot_helper_node.hpp"

namespace franka_motion
{
namespace
{

constexpr std::array<const char *, 9> kPandaTimedJoints{{
  "panda_joint1", "panda_joint2", "panda_joint3",
  "panda_joint4", "panda_joint5", "panda_joint6",
  "panda_joint7", "panda_finger_joint1", "panda_finger_joint2"}};

}  // namespace

bool has_non_empty_string_parameter(
  const rclcpp::Node::SharedPtr & node,
  const std::string & name)
{
  if (!node->has_parameter(name)) {
    return false;
  }
  const auto parameter = node->get_parameter(name);
  return parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING &&
         !parameter.as_string().empty();
}

bool has_parameter_prefix(
  const rclcpp::Node::SharedPtr & node,
  const std::string & prefix)
{
  return !node->list_parameters({prefix}, 10).names.empty();
}

std::vector<std::string> panda_urdf_position_limit_parameter_names()
{
  std::vector<std::string> names;
  names.reserve(kPandaTimedJoints.size() * 2U);
  for (const auto * joint_name : kPandaTimedJoints) {
    const std::string prefix =
      "robot_description_planning.joint_limits." + std::string(joint_name);
    names.push_back(prefix + ".min_position");
    names.push_back(prefix + ".max_position");
  }
  return names;
}

bool copy_moveit_parameters_from_move_group(
  const rclcpp::Node::SharedPtr & node,
  const MoveItParameterSyncOptions & options)
{
  return copy_moveit_parameters_from_move_group(node, options, {});
}

bool copy_moveit_parameters_from_move_group(
  const rclcpp::Node::SharedPtr & node,
  const MoveItParameterSyncOptions & options,
  const std::function<bool()> & stop_requested)
{
  if (detail::cooperative_stop_requested(stop_requested)) {
    return false;
  }
  auto helper = detail::make_parameter_snapshot_helper_node(
    options.helper_node_name, node->get_node_base_interface()->get_context());
  detail::ParameterSnapshotClient client(
    helper, options.move_group_node_name, stop_requested);
  if (!client.wait_for_service(options.service_timeout)) {
    RCLCPP_ERROR(node->get_logger(), "%s", options.service_unavailable_message.c_str());
    return false;
  }

  detail::OptionalParameterNameSet manifest;
  if (!detail::read_prefixed_names_manifest(node, client, options, manifest)) {
    return false;
  }
  if (detail::cooperative_stop_requested(stop_requested)) {
    return false;
  }
  if (!detail::copy_named_parameters(
      node, client, options.required_names, true, options.service_timeout) ||
    !detail::copy_named_parameters(
      node, client, options.optional_names, false, options.optional_timeout))
  {
    return false;
  }

  const detail::ParameterNameSet legacy_uninitialized_names(
    options.legacy_uninitialized_parameter_names.begin(),
    options.legacy_uninitialized_parameter_names.end());
  for (const auto & prefix : options.required_prefixes) {
    if (!detail::copy_prefixed_parameters(
        node, client, prefix, legacy_uninitialized_names, manifest, true,
        options.service_timeout))
    {
      return false;
    }
  }
  for (const auto & prefix : options.optional_prefixes) {
    if (!detail::copy_prefixed_parameters(
        node, client, prefix, legacy_uninitialized_names, manifest, false,
        options.optional_timeout))
    {
      return false;
    }
  }
  return !detail::cooperative_stop_requested(stop_requested);
}

bool publish_moveit_parameter_snapshot_manifest(
  const rclcpp::Node::SharedPtr & node,
  const std::vector<std::string> & prefixes)
{
  if (!node) {
    return false;
  }

  auto names = node->list_parameters(prefixes, 10).names;
  std::sort(names.begin(), names.end());
  names.erase(std::unique(names.begin(), names.end()), names.end());
  if (names.empty()) {
    RCLCPP_ERROR(node->get_logger(), "MoveIt prefix snapshot is empty");
    return false;
  }

  const std::string manifest_name{kMoveItParameterSnapshotManifest};
  if (node->has_parameter(manifest_name)) {
    return node->set_parameters_atomically(
      {rclcpp::Parameter(manifest_name, names)}).successful;
  }
  node->declare_parameter<std::vector<std::string>>(manifest_name, names);
  return true;
}

bool has_required_panda_acceleration_limits(const rclcpp::Node::SharedPtr & node)
{
  for (const auto * joint_name : kPandaTimedJoints) {
    const std::string prefix =
      "robot_description_planning.joint_limits." + std::string(joint_name);
    const std::string has_limit_name = prefix + ".has_acceleration_limits";
    const std::string max_limit_name = prefix + ".max_acceleration";
    if (!node->has_parameter(has_limit_name) ||
      node->get_parameter(has_limit_name).get_type() !=
      rclcpp::ParameterType::PARAMETER_BOOL ||
      !node->get_parameter(has_limit_name).as_bool())
    {
      RCLCPP_ERROR(node->get_logger(), "Не получен %s=true", has_limit_name.c_str());
      return false;
    }
    if (!node->has_parameter(max_limit_name) ||
      node->get_parameter(max_limit_name).get_type() !=
      rclcpp::ParameterType::PARAMETER_DOUBLE)
    {
      RCLCPP_ERROR(
        node->get_logger(), "Не получен положительный %s", max_limit_name.c_str());
      return false;
    }
    const double max_acceleration = node->get_parameter(max_limit_name).as_double();
    if (!std::isfinite(max_acceleration) || max_acceleration <= 0.0) {
      RCLCPP_ERROR(
        node->get_logger(), "Не получен конечный положительный %s",
        max_limit_name.c_str());
      return false;
    }
  }
  return true;
}

}  // namespace franka_motion
