// Responsibility: Validates complete staged objects and builds exact lease-scoped scene diffs.

#include "manipulation/scene/transfer_object_scene.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "manipulation/scene/collision_object_validation.hpp"
#include "manipulation/scene/resource_key.hpp"
#include "manipulation/validation/rigid_pose.hpp"
#include "moveit_msgs/msg/allowed_collision_entry.hpp"

namespace manipulation::scene
{
TransferObjectSceneResources make_transfer_object_scene_resources(
  const std::string & object_id,
  const std::vector<std::string> & touch_links)
{
  const auto world = make_world_resource_key(object_id);
  const auto attached = make_attached_resource_key(object_id);
  if (!world || !attached || touch_links.empty()) {
    throw std::invalid_argument(
            "object_id and at least one touch link must form canonical resources");
  }

  std::vector<std::string> contacts;
  contacts.reserve(touch_links.size());
  for (const auto & link : touch_links) {
    const auto contact = make_acm_resource_key(object_id, link);
    if (!contact) {
      throw std::invalid_argument("touch link cannot form a canonical ACM resource");
    }
    contacts.push_back(*contact);
  }
  std::sort(contacts.begin(), contacts.end());
  if (std::adjacent_find(contacts.begin(), contacts.end()) != contacts.end()) {
    throw std::invalid_argument("touch links must be unique");
  }

  TransferObjectSceneResources resources;
  resources.world = *world;
  resources.attached = *attached;
  resources.allowed_collision_pairs = std::move(contacts);
  resources.all = {resources.world, resources.attached};
  resources.all.insert(
    resources.all.end(), resources.allowed_collision_pairs.begin(),
    resources.allowed_collision_pairs.end());
  std::sort(resources.all.begin(), resources.all.end());
  return resources;
}

bool validate_staged_collision_object(
  const moveit_msgs::msg::CollisionObject & object,
  const std::string & planning_frame,
  std::string & reason)
{
  reason.clear();
  const auto structural_validation = validate_explicit_collision_object(object);
  if (!structural_validation.valid) {
    reason = structural_validation.message;
    return false;
  }
  if (!manipulation::validation::is_canonical_relative_frame_id(planning_frame) ||
    object.header.frame_id != planning_frame)
  {
    reason = "object must already use the configured planning frame";
    return false;
  }
  if (!manipulation::validation::is_normalized_nonzero_stamp(object.header.stamp)) {
    reason = "object observation stamp must be normalized and non-zero";
    return false;
  }
  return true;
}

moveit_msgs::msg::PlanningScene make_staged_world_add_diff(
  const moveit_msgs::msg::CollisionObject & object,
  const std::string & planning_frame)
{
  std::string reason;
  if (!validate_staged_collision_object(object, planning_frame, reason)) {
    throw std::invalid_argument("invalid staged object: " + reason);
  }
  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;
  scene.world.collision_objects.push_back(object);
  return scene;
}

moveit_msgs::msg::PlanningScene make_staged_contact_allow_diff(
  const std::string & object_id,
  const std::string & touch_link)
{
  if (!make_acm_resource_key(object_id, touch_link)) {
    throw std::invalid_argument("invalid staged object ACM pair");
  }
  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;
  auto & matrix = scene.allowed_collision_matrix;
  matrix.entry_names = {object_id, touch_link};
  matrix.entry_values.resize(2U);
  matrix.entry_values[0].enabled = {false, true};
  matrix.entry_values[1].enabled = {true, false};
  return scene;
}

moveit_msgs::msg::PlanningScene make_staged_world_remove_diff(
  const std::string & object_id,
  const std::string & planning_frame)
{
  if (!make_world_resource_key(object_id) ||
    !manipulation::validation::is_canonical_relative_frame_id(planning_frame))
  {
    throw std::invalid_argument("invalid staged object removal identity or frame");
  }
  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;
  moveit_msgs::msg::CollisionObject object;
  object.id = object_id;
  object.header.frame_id = planning_frame;
  object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
  scene.world.collision_objects.push_back(std::move(object));
  return scene;
}

}  // namespace manipulation::scene
