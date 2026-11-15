// Responsibility: Parses and validates cuRobo YAML trajectories before constructing MoveIt
//   responses.

#include "planning_io.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <builtin_interfaces/msg/duration.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

namespace manipulation_curobo::curobo::io
{
namespace
{

bool allFinite(const std::vector<double> & values)
{
  return std::all_of(values.begin(), values.end(), [](const double value) {
             return std::isfinite(value);
  });
}

bool readVector(
  const YAML::Node & point,
  const char * field,
  const std::size_t expected_size,
  const bool required,
  std::vector<double> & values,
  std::string & error)
{
  const YAML::Node value = point[field];
  if (!value) {
    if (required) {
      error = std::string{"trajectory point misses required field: "} + field;
      return false;
    }
    values.clear();
    return true;
  }
  if (!value.IsSequence()) {
    error = std::string{"trajectory point field is not a sequence: "} + field;
    return false;
  }
  values = value.as<std::vector<double>>();
  if (values.size() != expected_size || !allFinite(values)) {
    error = std::string{"trajectory point field has invalid size or values: "} + field;
    return false;
  }
  return true;
}

bool toDuration(
  const std::size_t point_index,
  const double dt,
  std::int64_t & previous_nanoseconds,
  builtin_interfaces::msg::Duration & duration,
  std::string & error)
{
  constexpr std::int64_t kNanosecondsPerSecond = 1'000'000'000LL;
  const long double seconds = static_cast<long double>(point_index) * dt;
  const long double nanoseconds = seconds * kNanosecondsPerSecond;
  const auto max_nanoseconds =
    static_cast<long double>(std::numeric_limits<std::int32_t>::max()) *
    kNanosecondsPerSecond + (kNanosecondsPerSecond - 1);
  if (!std::isfinite(seconds) || nanoseconds < 0.0L || nanoseconds > max_nanoseconds) {
    error = "trajectory time_from_start is outside builtin Duration range";
    return false;
  }

  const auto total_nanoseconds = static_cast<std::int64_t>(std::llround(nanoseconds));
  if (point_index > 0U && total_nanoseconds <= previous_nanoseconds) {
    error = "trajectory timestamps are not strictly increasing at nanosecond precision";
    return false;
  }
  previous_nanoseconds = total_nanoseconds;
  duration.sec = static_cast<std::int32_t>(total_nanoseconds / kNanosecondsPerSecond);
  duration.nanosec = static_cast<std::uint32_t>(total_nanoseconds % kNanosecondsPerSecond);
  return true;
}

bool validateJointNames(
  const std::vector<std::string> & response_names,
  const moveit::core::JointModelGroup & joint_model_group,
  std::string & error)
{
  const auto & expected_names = joint_model_group.getVariableNames();
  const std::set<std::string> actual(response_names.begin(), response_names.end());
  const std::set<std::string> expected(expected_names.begin(), expected_names.end());
  if (response_names.empty() || actual.size() != response_names.size() || actual != expected) {
    error = "trajectory joint_names must match the planning group exactly";
    return false;
  }
  return true;
}

}  // namespace

BridgeResultCode loadPlanningResponse(
  const std::filesystem::path & response_path,
  const moveit::core::RobotModelConstPtr & model,
  const moveit::core::JointModelGroup & joint_model_group,
  const moveit::core::RobotState & resolved_start_state,
  const planning_scene::PlanningSceneConstPtr & scene,
  const double start_point_tolerance,
  robot_trajectory::RobotTrajectory & trajectory,
  double & planning_time,
  std::string & error)
{
  planning_time = 0.0;
  trajectory.clear();
  if (!model || !scene || response_path.empty() ||
    resolved_start_state.getRobotModel() != model ||
    scene->getRobotModel() != model || !resolved_start_state.satisfiesBounds() ||
    !std::isfinite(start_point_tolerance) || start_point_tolerance < 0.0)
  {
    error = "response context is incomplete";
    return BridgeResultCode::kInvalidResponse;
  }

  try {
    const YAML::Node root = YAML::LoadFile(response_path.string());
    if (!root.IsMap()) {
      error = "bridge response root must be a mapping";
      return BridgeResultCode::kInvalidResponse;
    }
    planning_time = root["planning_time"] ? root["planning_time"].as<double>() : 0.0;
    if (!std::isfinite(planning_time) || planning_time < 0.0) {
      error = "bridge response contains invalid planning_time";
      return BridgeResultCode::kInvalidResponse;
    }
    if (!root["success"].as<bool>(false)) {
      error = root["message"] ? root["message"].as<std::string>() :
        "cuRobo reported an unsuccessful plan";
      const std::string result_code = root["error_code"] ?
        root["error_code"].as<std::string>() : "planning_failed";
      if (result_code == "planning_failed") {
        return BridgeResultCode::kPlanningFailed;
      }
      if (result_code == "timed_out") {
        return BridgeResultCode::kTimedOut;
      }
      if (result_code == "invalid_request") {
        return BridgeResultCode::kInvalidRequest;
      }
      if (result_code == "unsupported_scene") {
        return BridgeResultCode::kUnsupportedScene;
      }
      if (result_code == "internal_error") {
        return BridgeResultCode::kInternalError;
      }
      error = "bridge response contains an unknown error_code: " + result_code;
      return BridgeResultCode::kInvalidResponse;
    }

    const YAML::Node trajectory_node = root["trajectory"];
    if (!trajectory_node.IsMap() || !trajectory_node["dt"] ||
      !trajectory_node["joint_names"] || !trajectory_node["points"])
    {
      error = "successful response does not contain a complete trajectory";
      return BridgeResultCode::kInvalidResponse;
    }
    const double dt = trajectory_node["dt"].as<double>();
    if (!std::isfinite(dt) || dt <= 0.0) {
      error = "trajectory dt must be finite and positive";
      return BridgeResultCode::kInvalidResponse;
    }

    const auto joint_names =
      trajectory_node["joint_names"].as<std::vector<std::string>>();
    if (!validateJointNames(joint_names, joint_model_group, error)) {
      return BridgeResultCode::kInvalidResponse;
    }
    const YAML::Node points = trajectory_node["points"];
    if (!points.IsSequence() || points.size() == 0U) {
      error = "successful trajectory must contain at least one point";
      return BridgeResultCode::kInvalidResponse;
    }

    trajectory_msgs::msg::JointTrajectory trajectory_message;
    trajectory_message.joint_names = joint_names;
    trajectory_message.points.reserve(points.size());
    std::int64_t previous_nanoseconds = -1;
    for (std::size_t index = 0; index < points.size(); ++index) {
      const YAML::Node point = points[index];
      if (!point.IsMap()) {
        error = "trajectory point must be a mapping";
        return BridgeResultCode::kInvalidResponse;
      }
      trajectory_msgs::msg::JointTrajectoryPoint message;
      if (!readVector(point, "positions", joint_names.size(), true, message.positions, error) ||
        !readVector(point, "velocities", joint_names.size(), false, message.velocities, error) ||
        !readVector(
          point, "accelerations", joint_names.size(), false, message.accelerations, error) ||
        !toDuration(index, dt, previous_nanoseconds, message.time_from_start, error))
      {
        return BridgeResultCode::kInvalidResponse;
      }
      trajectory_message.points.push_back(std::move(message));
    }

    const auto & first_positions = trajectory_message.points.front().positions;
    for (std::size_t index = 0; index < joint_names.size(); ++index) {
      const double requested_start =
        resolved_start_state.getVariablePosition(joint_names[index]);
      if (std::abs(first_positions[index] - requested_start) > start_point_tolerance) {
        error = "trajectory first point differs from the resolved requested start state";
        return BridgeResultCode::kInvalidResponse;
      }
    }

    for (std::size_t index = 0; index < trajectory_message.points.size(); ++index) {
      moveit::core::RobotState waypoint(resolved_start_state);
      waypoint.setVariablePositions(
        joint_names, trajectory_message.points[index].positions);
      waypoint.update();
      if (!waypoint.satisfiesBounds(&joint_model_group)) {
        error = "trajectory waypoint violates planning-group joint bounds at index " +
          std::to_string(index);
        return BridgeResultCode::kInvalidResponse;
      }
      if (!scene->isStateValid(waypoint, joint_model_group.getName(), false)) {
        error = "trajectory waypoint is invalid in the immutable PlanningScene at index " +
          std::to_string(index);
        return BridgeResultCode::kInvalidResponse;
      }
    }

    trajectory.setRobotTrajectoryMsg(resolved_start_state, trajectory_message);
    return BridgeResultCode::kSuccess;
  } catch (const YAML::Exception & exception) {
    error = std::string{"invalid bridge response YAML: "} + exception.what();
  } catch (const std::exception & exception) {
    error = std::string{"cannot construct MoveIt trajectory: "} + exception.what();
  }
  return BridgeResultCode::kInvalidResponse;
}

}  // namespace manipulation_curobo::curobo::io
