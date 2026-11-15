// Responsibility: Rejects scene diffs that violate ownership, lease or canonical mutation
//   constraints.

#include <algorithm>
#include <map>
#include <set>
#include <string>
#include <vector>

#include <moveit_msgs/msg/collision_object.hpp>

#include "manipulation/scene/coordinator_utils.hpp"
#include "manipulation/scene/resource_key.hpp"
#include "manipulation/validation/frame_id.hpp"

namespace manipulation::scene
{
namespace
{

SceneDiffValidationResult invalid(std::string message)
{
  return {false, {}, std::move(message)};
}

bool has_unsupported_robot_state(const moveit_msgs::msg::PlanningScene & scene)
{
  const auto & joint_state = scene.robot_state.joint_state;
  const auto & multi_dof = scene.robot_state.multi_dof_joint_state;
  return !joint_state.name.empty() || !joint_state.position.empty() ||
         !joint_state.velocity.empty() || !joint_state.effort.empty() ||
         !multi_dof.joint_names.empty() || !multi_dof.transforms.empty() ||
         !multi_dof.twist.empty() || !multi_dof.wrench.empty();
}

bool has_octomap_update(const moveit_msgs::msg::PlanningScene & scene)
{
  const auto & octomap = scene.world.octomap.octomap;
  return octomap.binary || !octomap.id.empty() || octomap.resolution != 0.0 ||
         !octomap.data.empty();
}

bool is_supported_operation(const std::uint8_t operation)
{
  using CollisionObject = moveit_msgs::msg::CollisionObject;
  return operation == CollisionObject::ADD || operation == CollisionObject::REMOVE;
}

bool has_required_frame(const moveit_msgs::msg::CollisionObject & object)
{
  return object.operation == moveit_msgs::msg::CollisionObject::REMOVE ||
         manipulation::validation::is_canonical_relative_frame_id(
    object.header.frame_id);
}

bool has_explicit_geometry(const moveit_msgs::msg::CollisionObject & object)
{
  return !object.primitives.empty() || !object.meshes.empty() || !object.planes.empty();
}

bool insert_unique(std::set<std::string> & resources, std::string resource)
{
  return resources.insert(std::move(resource)).second;
}

}  // namespace

SceneDiffValidationResult validate_scene_diff_resources(
  const moveit_msgs::msg::PlanningScene & scene_diff,
  const std::vector<std::string> & declared_resource_keys)
{
  if (!scene_diff.is_diff) {
    return invalid("scene_diff.is_diff must be true");
  }
  if (!scene_diff.name.empty() || !scene_diff.robot_model_name.empty() ||
    !scene_diff.fixed_frame_transforms.empty() || !scene_diff.link_padding.empty() ||
    !scene_diff.link_scale.empty() || !scene_diff.object_colors.empty() ||
    has_octomap_update(scene_diff) || has_unsupported_robot_state(scene_diff))
  {
    return invalid("scene diff contains a mutation outside the leased resource model");
  }
  if (!scene_diff.robot_state.attached_collision_objects.empty() &&
    !scene_diff.robot_state.is_diff)
  {
    return invalid("robot_state.is_diff must be true for attached-object changes");
  }

  std::set<std::string> derived_resources;
  std::map<std::string, std::uint8_t> world_operations;
  for (const auto & object : scene_diff.world.collision_objects) {
    const auto resource = make_world_resource_key(object.id);
    if (!resource || !is_supported_operation(object.operation) || !has_required_frame(object)) {
      return invalid("world collision object has an invalid ID, operation or frame");
    }
    if (!insert_unique(derived_resources, *resource)) {
      return invalid("scene diff modifies the same world object more than once");
    }
    if (object.operation == moveit_msgs::msg::CollisionObject::ADD &&
      !has_explicit_geometry(object))
    {
      return invalid("world ADD requires explicit geometry");
    }
    world_operations.emplace(object.id, object.operation);
  }

  for (const auto & attached : scene_diff.robot_state.attached_collision_objects) {
    const auto resource = make_attached_resource_key(attached.object.id);
    if (!resource || !is_supported_operation(attached.object.operation) ||
      !has_required_frame(attached.object))
    {
      return invalid("attached collision object has an invalid ID, operation or frame");
    }
    if (!insert_unique(derived_resources, *resource)) {
      return invalid("scene diff modifies the same attached object more than once");
    }
    const auto world_operation = world_operations.find(attached.object.id);
    if (attached.object.operation == moveit_msgs::msg::CollisionObject::ADD) {
      if (!has_explicit_geometry(attached.object) || attached.link_name.empty() ||
        attached.object.header.frame_id != attached.link_name)
      {
        return invalid("attached ADD requires explicit geometry in its link frame");
      }
      if (world_operation != world_operations.end()) {
        return invalid("attached ADD cannot be combined with a world mutation for the same ID");
      }
    } else if (world_operation == world_operations.end()) {
      return invalid("attached REMOVE requires a paired world ADD or REMOVE");
    }
    // Attaching can consume a world object, while detaching can make it world
    // state again. Uniform dual ownership closes both implicit transitions.
    const auto world_resource = make_world_resource_key(attached.object.id);
    derived_resources.insert(*world_resource);
  }

  const auto & acm = scene_diff.allowed_collision_matrix;
  if (!acm.default_entry_names.empty() || !acm.default_entry_values.empty()) {
    return invalid("ACM default entries are not supported by pair-scoped leases");
  }
  if (acm.entry_names.empty() != acm.entry_values.empty()) {
    return invalid("ACM names and rows must either both be empty or both be present");
  }
  if (!acm.entry_names.empty()) {
    const auto size = acm.entry_names.size();
    if (size < 2U || acm.entry_values.size() != size) {
      return invalid("ACM update must be a square matrix containing at least one pair");
    }
    std::set<std::string> unique_names(acm.entry_names.begin(), acm.entry_names.end());
    if (unique_names.size() != size) {
      return invalid("ACM entry names must be unique");
    }
    for (const auto & row : acm.entry_values) {
      if (row.enabled.size() != size) {
        return invalid("ACM update must be square");
      }
    }
    for (std::size_t first = 0; first < size; ++first) {
      for (std::size_t second = first + 1; second < size; ++second) {
        if (acm.entry_values[first].enabled[second] !=
          acm.entry_values[second].enabled[first])
        {
          return invalid("ACM update must be symmetric");
        }
        const auto resource = make_acm_resource_key(
          acm.entry_names[first], acm.entry_names[second]);
        if (!resource || !insert_unique(derived_resources, *resource)) {
          return invalid("ACM contains an invalid or duplicate pair");
        }
      }
    }
  }

  std::set<std::string> declared_resources;
  for (const auto & key : declared_resource_keys) {
    const auto parsed = parse_resource_key(key);
    if (!parsed || !declared_resources.insert(parsed->canonical).second) {
      return invalid("declared resource keys must be canonical and unique");
    }
  }
  if (derived_resources.empty()) {
    return invalid("scene diff does not contain a supported mutation");
  }
  if (declared_resources != derived_resources) {
    return invalid("declared resource keys do not exactly cover scene mutations");
  }

  return {
    true,
    std::vector<std::string>(derived_resources.begin(), derived_resources.end()),
    {}};
}

}  // namespace manipulation::scene
