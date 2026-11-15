// Responsibility: Copies complete named or prefixed MoveIt snapshots without partial updates.

#include "moveit_parameter_snapshot.hpp"

#include <algorithm>
#include <exception>
#include <iterator>
#include <string>
#include <vector>

#include "parameter_snapshot_apply.hpp"

namespace franka_motion::detail
{
namespace
{

using SteadyClock = ParameterSnapshotClient::Clock;
constexpr auto kInitialManifestReadTimeout = std::chrono::seconds{2};

bool belongs_to_prefix(const std::string & name, const std::string & prefix)
{
  return name == prefix ||
         (name.size() > prefix.size() && name.compare(0, prefix.size(), prefix) == 0 &&
         name[prefix.size()] == '.');
}

}  // namespace

bool copy_named_parameters(
  const rclcpp::Node::SharedPtr & node,
  ParameterSnapshotClient & client,
  const std::vector<std::string> & names,
  const bool required,
  const std::chrono::seconds timeout)
{
  std::vector<rclcpp::Parameter> snapshot;
  snapshot.reserve(names.size());
  for (const auto & name : names) {
    const auto parameter = client.wait_for_parameter(
      name, SteadyClock::now() + timeout);
    if (parameter) {
      snapshot.push_back(*parameter);
      continue;
    }
    if (required) {
      RCLCPP_ERROR(
        node->get_logger(),
        "Источник MoveIt-параметров не содержит параметр %s", name.c_str());
      return false;
    }
    RCLCPP_WARN(
      node->get_logger(), "Параметр %s не найден, пропускаю", name.c_str());
  }

  const auto error = apply_parameter_snapshot_atomically(node, snapshot);
  if (!error) {
    return true;
  }
  if (required) {
    RCLCPP_ERROR(
      node->get_logger(), "Required MoveIt parameter snapshot was rejected: %s",
      error->c_str());
    return false;
  }
  RCLCPP_WARN(
    node->get_logger(),
    "Optional MoveIt parameter snapshot was rejected; skipping it: %s",
    error->c_str());
  return true;
}

bool copy_prefixed_parameters(
  const rclcpp::Node::SharedPtr & node,
  ParameterSnapshotClient & client,
  const std::string & prefix,
  const ParameterNameSet & legacy_uninitialized_names,
  const OptionalParameterNameSet & manifest,
  const bool required,
  const std::chrono::seconds timeout)
{
  const auto deadline = SteadyClock::now() + timeout;
  std::vector<std::string> names;
  if (manifest) {
    std::copy_if(
      manifest->begin(), manifest->end(), std::back_inserter(names),
      [&prefix](const auto & name) {return belongs_to_prefix(name, prefix);});
    std::sort(names.begin(), names.end());
  } else {
    names = client.wait_for_parameter_names(prefix, 10, deadline);
  }

  if (names.empty()) {
    if (required) {
      RCLCPP_ERROR(
        node->get_logger(),
        "Источник MoveIt-параметров не содержит параметры %s.*", prefix.c_str());
      return false;
    }
    RCLCPP_WARN(
      node->get_logger(), "Параметры %s.* не найдены, пропускаю", prefix.c_str());
    return true;
  }

  std::vector<rclcpp::Parameter> snapshot;
  snapshot.reserve(names.size());
  bool complete = true;
  for (const auto & name : names) {
    if (!rclcpp::ok() || SteadyClock::now() >= deadline) {
      complete = false;
      break;
    }
    try {
      const auto parameters = client.get_parameters({name}, deadline);
      if (!parameters) {
        complete = false;
        break;
      }
      if (parameters->empty()) {
        if (!manifest && legacy_uninitialized_names.count(name) != 0U) {
          continue;
        }
        complete = false;
        break;
      }
      if (parameters->size() != 1U) {
        complete = false;
        break;
      }
      if (parameters->front().get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
        if (!manifest && legacy_uninitialized_names.count(name) != 0U) {
          continue;
        }
        complete = false;
        break;
      }
      snapshot.push_back(parameters->front());
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        node->get_logger(), "Не удалось прочитать %s: %s",
        name.c_str(), error.what());
      complete = false;
      break;
    }
  }

  if (!complete || (required && snapshot.empty())) {
    if (required) {
      RCLCPP_ERROR(
        node->get_logger(),
        "Не удалось получить целостный snapshot параметров %s.*", prefix.c_str());
      return false;
    }
    RCLCPP_WARN(
      node->get_logger(),
      "Optional MoveIt snapshot %s.* is incomplete; skipping the entire prefix",
      prefix.c_str());
    return true;
  }
  if (snapshot.empty()) {
    RCLCPP_WARN(
      node->get_logger(),
      "Optional MoveIt snapshot %s.* contains only known uninitialized legacy values; "
      "skipping the prefix", prefix.c_str());
    return true;
  }

  const auto error = apply_parameter_snapshot_atomically(node, snapshot);
  if (!error) {
    return true;
  }
  if (required) {
    RCLCPP_ERROR(
      node->get_logger(), "Required MoveIt snapshot %s.* was rejected: %s",
      prefix.c_str(), error->c_str());
    return false;
  }
  RCLCPP_WARN(
    node->get_logger(), "Optional MoveIt snapshot %s.* was rejected; skipping it: %s",
    prefix.c_str(), error->c_str());
  return true;
}

bool read_prefixed_names_manifest(
  const rclcpp::Node::SharedPtr & node,
  ParameterSnapshotClient & client,
  const MoveItParameterSyncOptions & options,
  OptionalParameterNameSet & manifest)
{
  if (options.prefixed_names_manifest.empty()) {
    return true;
  }
  try {
    const auto deadline = SteadyClock::now() + options.service_timeout;
    const auto parameters = client.get_parameters_with_retry(
      {options.prefixed_names_manifest}, deadline, kInitialManifestReadTimeout);
    if (!parameters) {
      RCLCPP_ERROR(
        node->get_logger(), "Timed out reading MoveIt snapshot manifest %s",
        options.prefixed_names_manifest.c_str());
      return false;
    }
    // rclcpp omits an absent or explicit NOT_SET parameter from a successful reply.
    if (parameters->empty()) {
      return true;
    }
    if (parameters->size() != 1U) {
      RCLCPP_ERROR(
        node->get_logger(), "Invalid response while reading MoveIt snapshot manifest %s",
        options.prefixed_names_manifest.c_str());
      return false;
    }
    if (parameters->front().get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
      return true;
    }
    if (parameters->front().get_type() != rclcpp::ParameterType::PARAMETER_STRING_ARRAY) {
      RCLCPP_ERROR(
        node->get_logger(), "MoveIt snapshot manifest %s must be a string array",
        options.prefixed_names_manifest.c_str());
      return false;
    }
    const auto & names = parameters->front().as_string_array();
    if (names.empty()) {
      RCLCPP_ERROR(
        node->get_logger(), "MoveIt snapshot manifest %s is empty",
        options.prefixed_names_manifest.c_str());
      return false;
    }
    manifest.emplace(names.begin(), names.end());
    return true;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(
      node->get_logger(), "Cannot read MoveIt snapshot manifest %s: %s",
      options.prefixed_names_manifest.c_str(), error.what());
    return false;
  }
}

}  // namespace franka_motion::detail
