// Responsibility: Declares complete named/prefixed MoveIt snapshot copy operations.

#ifndef MOVEIT_PARAMETER_SNAPSHOT_HPP_
#define MOVEIT_PARAMETER_SNAPSHOT_HPP_

#include <chrono>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "franka_motion/moveit_parameter_sync.hpp"

#include "parameter_snapshot_client.hpp"

namespace franka_motion::detail
{

using ParameterNameSet = std::unordered_set<std::string>;
using OptionalParameterNameSet = std::optional<ParameterNameSet>;

bool copy_named_parameters(
  const rclcpp::Node::SharedPtr & node,
  ParameterSnapshotClient & client,
  const std::vector<std::string> & names,
  bool required,
  std::chrono::seconds timeout);

bool copy_prefixed_parameters(
  const rclcpp::Node::SharedPtr & node,
  ParameterSnapshotClient & client,
  const std::string & prefix,
  const ParameterNameSet & legacy_uninitialized_names,
  const OptionalParameterNameSet & manifest,
  bool required,
  std::chrono::seconds timeout);

bool read_prefixed_names_manifest(
  const rclcpp::Node::SharedPtr & node,
  ParameterSnapshotClient & client,
  const MoveItParameterSyncOptions & options,
  OptionalParameterNameSet & manifest);

}  // namespace franka_motion::detail

#endif  // MOVEIT_PARAMETER_SNAPSHOT_HPP_
