// Responsibility: Canonicalizes world and attached collision objects before hashing or applying
//   diffs.

#include "scene_state_model_internal.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace manipulation::scene::detail
{
namespace
{

double canonical_scalar(const double value)
{
  constexpr double kResolution = 1e-12;
  if (std::abs(value) > 1e6) {
    return value;
  }
  const double rounded = std::round(value / kResolution) * kResolution;
  return rounded == 0.0 ? 0.0 : rounded;
}

bool pose_is_empty(const geometry_msgs::msg::Pose & pose)
{
  return pose.position.x == 0.0 && pose.position.y == 0.0 &&
         pose.position.z == 0.0 && pose.orientation.x == 0.0 &&
         pose.orientation.y == 0.0 && pose.orientation.z == 0.0 &&
         pose.orientation.w == 0.0;
}

geometry_msgs::msg::Pose identity_pose()
{
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  return pose;
}

bool normalize_pose(geometry_msgs::msg::Pose & pose)
{
  const auto & position = pose.position;
  auto & orientation = pose.orientation;
  if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
    !std::isfinite(position.z) || !std::isfinite(orientation.x) ||
    !std::isfinite(orientation.y) || !std::isfinite(orientation.z) ||
    !std::isfinite(orientation.w))
  {
    return false;
  }

  const double norm = std::sqrt(
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w);
  if (!std::isfinite(norm) || norm == 0.0) {
    return false;
  }
  orientation.x /= norm;
  orientation.y /= norm;
  orientation.z /= norm;
  orientation.w /= norm;

  // q and -q represent the same rotation. A stable sign avoids false digest
  // mismatches when equivalent quaternions cross different serializers.
  const bool negate = orientation.w < 0.0 ||
    (orientation.w == 0.0 && orientation.x < 0.0) ||
    (orientation.w == 0.0 && orientation.x == 0.0 && orientation.y < 0.0) ||
    (orientation.w == 0.0 && orientation.x == 0.0 && orientation.y == 0.0 &&
    orientation.z < 0.0);
  if (negate) {
    orientation.x = -orientation.x;
    orientation.y = -orientation.y;
    orientation.z = -orientation.z;
    orientation.w = -orientation.w;
  }
  pose.position.x = canonical_scalar(pose.position.x);
  pose.position.y = canonical_scalar(pose.position.y);
  pose.position.z = canonical_scalar(pose.position.z);
  orientation.x = canonical_scalar(orientation.x);
  orientation.y = canonical_scalar(orientation.y);
  orientation.z = canonical_scalar(orientation.z);
  orientation.w = canonical_scalar(orientation.w);
  return true;
}

template<typename ValueT>
bool sort_paired_by_name(
  std::vector<std::string> & names,
  std::vector<ValueT> & values)
{
  if (names.size() != values.size()) {
    return false;
  }
  std::vector<std::pair<std::string, ValueT>> pairs;
  pairs.reserve(names.size());
  for (std::size_t index = 0; index < names.size(); ++index) {
    if (names[index].empty()) {
      return false;
    }
    pairs.emplace_back(std::move(names[index]), std::move(values[index]));
  }
  std::sort(pairs.begin(), pairs.end(), [](const auto & left, const auto & right) {
      return left.first < right.first;
  });
  for (std::size_t index = 1; index < pairs.size(); ++index) {
    if (pairs[index - 1].first == pairs[index].first) {
      return false;
    }
  }
  names.clear();
  values.clear();
  names.reserve(pairs.size());
  values.reserve(pairs.size());
  for (auto & [name, value] : pairs) {
    names.push_back(std::move(name));
    values.push_back(std::move(value));
  }
  return true;
}

}  // namespace

bool type_is_empty(const object_recognition_msgs::msg::ObjectType & type)
{
  return type.db.empty() && type.key.empty();
}

bool normalize_collision_object(CollisionObject & object, std::string & error)
{
  const auto shape_count =
    object.primitives.size() + object.meshes.size() + object.planes.size();
  if (shape_count == 0U) {
    error = "collision object '" + object.id + "' must contain explicit geometry";
    return false;
  }
  if (object.primitives.size() != object.primitive_poses.size() ||
    object.meshes.size() != object.mesh_poses.size() ||
    object.planes.size() != object.plane_poses.size())
  {
    error = "collision object '" + object.id + "' must provide one pose per shape";
    return false;
  }

  auto normalize_poses = [](auto & poses) {
      return std::all_of(poses.begin(), poses.end(), [](auto & pose) {
                 return normalize_pose(pose);
      });
    };
  if (!normalize_poses(object.primitive_poses) ||
    !normalize_poses(object.mesh_poses) || !normalize_poses(object.plane_poses))
  {
    error = "collision object '" + object.id + "' contains an invalid shape pose";
    return false;
  }

  if (pose_is_empty(object.pose)) {
    if (shape_count != 1U) {
      error = "collision object '" + object.id +
        "' must provide an object pose when it has multiple shapes";
      return false;
    }
    auto move_single_shape_pose = [&object](auto & poses) {
        if (!poses.empty()) {
          object.pose = poses.front();
          poses.front() = identity_pose();
          return true;
        }
        return false;
      };
    if (!move_single_shape_pose(object.primitive_poses) &&
      !move_single_shape_pose(object.mesh_poses))
    {
      (void)move_single_shape_pose(object.plane_poses);
    }
  } else if (!normalize_pose(object.pose)) {
    error = "collision object '" + object.id + "' contains an invalid object pose";
    return false;
  }

  if (!sort_paired_by_name(object.subframe_names, object.subframe_poses)) {
    error = "collision object '" + object.id + "' has invalid subframes";
    return false;
  }
  if (!normalize_poses(object.subframe_poses)) {
    error = "collision object '" + object.id + "' contains an invalid subframe pose";
    return false;
  }

  object.header.stamp = builtin_interfaces::msg::Time();
  object.operation = CollisionObject::ADD;
  return true;
}

bool normalize_attached_object(AttachedObject & attached, std::string & error)
{
  if (attached.link_name.empty() || attached.object.header.frame_id != attached.link_name) {
    error = "attached object '" + attached.object.id +
      "' must provide geometry in its attached link frame";
    return false;
  }
  if (!normalize_collision_object(attached.object, error)) {
    return false;
  }
  std::sort(attached.touch_links.begin(), attached.touch_links.end());
  attached.touch_links.erase(
    std::unique(attached.touch_links.begin(), attached.touch_links.end()),
    attached.touch_links.end());
  attached.detach_posture.header.stamp = builtin_interfaces::msg::Time();
  return true;
}

}  // namespace manipulation::scene::detail
