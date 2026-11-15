// Responsibility: Resolve transfer target geometry without lifecycle state or side effects.

#include "detail/transfer_target_resolution.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace manipulation::workflow::detail
{
namespace
{

double quaternion_dot(
  const geometry_msgs::msg::Quaternion & lhs,
  const geometry_msgs::msg::Quaternion & rhs)
{
  return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z + lhs.w * rhs.w;
}

geometry_msgs::msg::Quaternion multiply_quaternions(
  const geometry_msgs::msg::Quaternion & lhs,
  const geometry_msgs::msg::Quaternion & rhs)
{
  geometry_msgs::msg::Quaternion result;
  result.x = lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y;
  result.y = lhs.w * rhs.y - lhs.x * rhs.z + lhs.y * rhs.w + lhs.z * rhs.x;
  result.z = lhs.w * rhs.z + lhs.x * rhs.y - lhs.y * rhs.x + lhs.z * rhs.w;
  result.w = lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z;
  return result;
}

geometry_msgs::msg::Quaternion align_quaternion_sign(
  geometry_msgs::msg::Quaternion orientation,
  const geometry_msgs::msg::Quaternion & reference)
{
  if (quaternion_dot(orientation, reference) < 0.0) {
    orientation.x = -orientation.x;
    orientation.y = -orientation.y;
    orientation.z = -orientation.z;
    orientation.w = -orientation.w;
  }
  return orientation;
}

geometry_msgs::msg::PoseStamped nearest_equivalent_target(
  geometry_msgs::msg::PoseStamped target,
  const geometry_msgs::msg::Quaternion & reference,
  const ObjectProfile & profile)
{
  if (profile.orientation_symmetries().empty()) {
    return target;
  }

  auto best = align_quaternion_sign(target.pose.orientation, reference);
  auto best_score = std::abs(quaternion_dot(best, reference));
  for (const auto & local_symmetry : profile.orientation_symmetries()) {
    auto candidate = multiply_quaternions(target.pose.orientation, local_symmetry);
    const auto score = std::abs(quaternion_dot(candidate, reference));
    if (score > best_score) {
      best = align_quaternion_sign(std::move(candidate), reference);
      best_score = score;
    }
  }
  target.pose.orientation = std::move(best);
  return target;
}

}  // namespace

geometry_msgs::msg::PoseStamped resolve_transfer_target(
  const manipulation_msgs::msg::TransferObjectStepV2 & step,
  const ObjectObservation & observation,
  const ObjectProfile & profile,
  const std::unordered_map<std::string, geometry_msgs::msg::PoseStamped> & initial_sources)
{
  using Step = manipulation_msgs::msg::TransferObjectStepV2;
  switch (step.target_mode) {
    case Step::TARGET_ABSOLUTE_POSE:
      return step.target_object_pose;
    case Step::TARGET_TRANSLATION_FROM_CURRENT_SOURCE: {
        auto target = observation.pose();
        target.pose.position.x += step.target_translation.vector.x;
        target.pose.position.y += step.target_translation.vector.y;
        target.pose.position.z += step.target_translation.vector.z;
        return target;
      }
    case Step::TARGET_INITIAL_SOURCE: {
        const auto found = initial_sources.find(step.object_id);
        if (found == initial_sources.cend()) {
          throw std::invalid_argument(
                  "validated initial-source target has no prior successful observation");
        }
        return nearest_equivalent_target(
          found->second, observation.pose().pose.orientation, profile);
      }
    default:
      throw std::invalid_argument("validated V2 step contains an unknown target mode");
  }
}

}  // namespace manipulation::workflow::detail
