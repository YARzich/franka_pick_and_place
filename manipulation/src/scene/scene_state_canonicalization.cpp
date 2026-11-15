// Responsibility: Produces deterministic PlanningScene ordering and byte-independent semantic
//   digests.

#include <algorithm>
#include <set>
#include <stdexcept>
#include <string>

#include "manipulation/scene/coordinator_utils.hpp"
#include "scene_state_model_internal.hpp"

namespace manipulation::scene
{

moveit_msgs::msg::PlanningScene canonical_authoritative_scene(
  const moveit_msgs::msg::PlanningScene & scene)
{
  moveit_msgs::msg::PlanningScene canonical;
  canonical.world.collision_objects = scene.world.collision_objects;
  canonical.robot_state.attached_collision_objects =
    scene.robot_state.attached_collision_objects;

  std::set<std::string> world_ids;
  for (auto & object : canonical.world.collision_objects) {
    std::string error;
    if (object.id.empty() || !world_ids.insert(object.id).second ||
      !detail::normalize_collision_object(object, error))
    {
      throw std::runtime_error(
              error.empty() ? "authoritative world contains duplicate or empty IDs" : error);
    }
  }
  std::sort(
    canonical.world.collision_objects.begin(),
    canonical.world.collision_objects.end(),
    [](const auto & left, const auto & right) {return left.id < right.id;});

  std::set<std::string> attached_ids;
  for (auto & attached : canonical.robot_state.attached_collision_objects) {
    std::string error;
    if (attached.object.id.empty() || !attached_ids.insert(attached.object.id).second ||
      !detail::normalize_attached_object(attached, error))
    {
      throw std::runtime_error(
              error.empty() ? "authoritative attached state has duplicate or empty IDs" : error);
    }
  }
  std::sort(
    canonical.robot_state.attached_collision_objects.begin(),
    canonical.robot_state.attached_collision_objects.end(),
    [](const auto & left, const auto & right) {
      return left.object.id < right.object.id;
    });

  canonical.allowed_collision_matrix =
    detail::AcmState::from_message(scene.allowed_collision_matrix).to_message();
  return canonical;
}

}  // namespace manipulation::scene
