// Responsibility: Validates joint and multi-DOF first waypoints against one robot state.

#include "trajectory_state_validation.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace manipulation::moveit::detail
{
namespace
{

bool finite_transform(const geometry_msgs::msg::Transform & transform) noexcept
{
  return std::isfinite(transform.translation.x) &&
         std::isfinite(transform.translation.y) &&
         std::isfinite(transform.translation.z) &&
         std::isfinite(transform.rotation.x) &&
         std::isfinite(transform.rotation.y) &&
         std::isfinite(transform.rotation.z) &&
         std::isfinite(transform.rotation.w);
}

TrajectoryStartStateStatus compare_transform(
  const geometry_msgs::msg::Transform & planned,
  const geometry_msgs::msg::Transform & observed,
  const double tolerance) noexcept
{
  if (!finite_transform(planned) || !finite_transform(observed)) {
    return TrajectoryStartStateStatus::kInvalid;
  }
  const double planned_norm = std::sqrt(
    planned.rotation.x * planned.rotation.x +
    planned.rotation.y * planned.rotation.y +
    planned.rotation.z * planned.rotation.z +
    planned.rotation.w * planned.rotation.w);
  const double observed_norm = std::sqrt(
    observed.rotation.x * observed.rotation.x +
    observed.rotation.y * observed.rotation.y +
    observed.rotation.z * observed.rotation.z +
    observed.rotation.w * observed.rotation.w);
  constexpr double kUnitQuaternionTolerance = 1.0e-6;
  if (std::abs(planned_norm - 1.0) > kUnitQuaternionTolerance ||
    std::abs(observed_norm - 1.0) > kUnitQuaternionTolerance)
  {
    return TrajectoryStartStateStatus::kInvalid;
  }

  const double translation_delta = std::max({
        std::abs(planned.translation.x - observed.translation.x),
        std::abs(planned.translation.y - observed.translation.y),
        std::abs(planned.translation.z - observed.translation.z)});
  const double quaternion_dot = std::abs(
    (planned.rotation.x * observed.rotation.x +
    planned.rotation.y * observed.rotation.y +
    planned.rotation.z * observed.rotation.z +
    planned.rotation.w * observed.rotation.w) /
    (planned_norm * observed_norm));
  const double angular_delta = 2.0 * std::acos(std::min(1.0, quaternion_dot));
  return translation_delta <= tolerance && angular_delta <= tolerance ?
         TrajectoryStartStateStatus::kMatch :
         TrajectoryStartStateStatus::kMismatch;
}

TrajectoryStartStateCheck check_joint_start(
  const moveit_msgs::msg::RobotState & authoritative_state,
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  const double tolerance)
{
  if (trajectory.points.empty()) {
    return {TrajectoryStartStateStatus::kMatch, {}};
  }
  if (trajectory.joint_names.empty() ||
    trajectory.points.front().positions.size() != trajectory.joint_names.size())
  {
    return {
      TrajectoryStartStateStatus::kInvalid,
      "planned joint trajectory has no complete first waypoint"};
  }
  const auto & state = authoritative_state.joint_state;
  if (state.name.empty() || state.name.size() != state.position.size()) {
    return {
      TrajectoryStartStateStatus::kInvalid,
      "authoritative snapshot has an incomplete joint start state"};
  }

  std::unordered_map<std::string, double> observed_positions;
  observed_positions.reserve(state.name.size());
  for (std::size_t index = 0U; index < state.name.size(); ++index) {
    if (state.name[index].empty() || !std::isfinite(state.position[index]) ||
      !observed_positions.emplace(state.name[index], state.position[index]).second)
    {
      return {
        TrajectoryStartStateStatus::kInvalid,
        "authoritative snapshot has invalid or duplicate joint state entries"};
    }
  }

  std::unordered_set<std::string> planned_names;
  for (std::size_t index = 0U; index < trajectory.joint_names.size(); ++index) {
    const auto & name = trajectory.joint_names[index];
    const double planned_position = trajectory.points.front().positions[index];
    const auto observed = observed_positions.find(name);
    if (name.empty() || !planned_names.insert(name).second ||
      observed == observed_positions.cend() || !std::isfinite(planned_position))
    {
      return {
        TrajectoryStartStateStatus::kInvalid,
        "planned trajectory start cannot be correlated with the authoritative snapshot"};
    }
    if (std::abs(planned_position - observed->second) > tolerance) {
      return {
        TrajectoryStartStateStatus::kMismatch,
        "planned trajectory first waypoint differs from the authoritative snapshot"};
    }
  }
  return {TrajectoryStartStateStatus::kMatch, {}};
}

TrajectoryStartStateCheck check_multi_dof_start(
  const moveit_msgs::msg::RobotState & authoritative_state,
  const trajectory_msgs::msg::MultiDOFJointTrajectory & trajectory,
  const double tolerance)
{
  if (trajectory.points.empty()) {
    return {TrajectoryStartStateStatus::kMatch, {}};
  }
  const auto & state = authoritative_state.multi_dof_joint_state;
  if (trajectory.joint_names.empty() ||
    trajectory.points.front().transforms.size() != trajectory.joint_names.size() ||
    state.joint_names.empty() || state.joint_names.size() != state.transforms.size())
  {
    return {
      TrajectoryStartStateStatus::kInvalid,
      "planned multi-DOF trajectory has no complete authoritative start state"};
  }

  std::unordered_map<std::string, geometry_msgs::msg::Transform> observed_transforms;
  observed_transforms.reserve(state.joint_names.size());
  for (std::size_t index = 0U; index < state.joint_names.size(); ++index) {
    if (state.joint_names[index].empty() || !finite_transform(state.transforms[index]) ||
      !observed_transforms.emplace(state.joint_names[index], state.transforms[index]).second)
    {
      return {
        TrajectoryStartStateStatus::kInvalid,
        "authoritative snapshot has invalid or duplicate multi-DOF entries"};
    }
  }

  std::unordered_set<std::string> planned_names;
  for (std::size_t index = 0U; index < trajectory.joint_names.size(); ++index) {
    const auto & name = trajectory.joint_names[index];
    const auto observed = observed_transforms.find(name);
    if (name.empty() || !planned_names.insert(name).second ||
      observed == observed_transforms.cend())
    {
      return {
        TrajectoryStartStateStatus::kInvalid,
        "planned multi-DOF start cannot be correlated with the authoritative snapshot"};
    }
    const auto status = compare_transform(
      trajectory.points.front().transforms[index], observed->second, tolerance);
    if (status != TrajectoryStartStateStatus::kMatch) {
      return {
        status,
        status == TrajectoryStartStateStatus::kInvalid ?
        "planned multi-DOF start contains an invalid transform" :
        "planned multi-DOF first waypoint differs from the authoritative snapshot"};
    }
  }
  return {TrajectoryStartStateStatus::kMatch, {}};
}

}  // namespace

TrajectoryStartStateCheck check_trajectory_start_state(
  const moveit_msgs::msg::RobotState & authoritative_state,
  const moveit_msgs::msg::RobotTrajectory & trajectory,
  const double tolerance)
{
  if (!std::isfinite(tolerance) || tolerance < 0.0) {
    return {
      TrajectoryStartStateStatus::kInvalid,
      "trajectory start-state tolerance must be finite and non-negative"};
  }
  if (trajectory.joint_trajectory.points.empty() &&
    trajectory.multi_dof_joint_trajectory.points.empty())
  {
    return {
      TrajectoryStartStateStatus::kInvalid,
      "planning backend returned success with an empty trajectory"};
  }
  const auto joint = check_joint_start(
    authoritative_state, trajectory.joint_trajectory, tolerance);
  if (joint.status != TrajectoryStartStateStatus::kMatch) {
    return joint;
  }
  return check_multi_dof_start(
    authoritative_state, trajectory.multi_dof_joint_trajectory, tolerance);
}

}  // namespace manipulation::moveit::detail
