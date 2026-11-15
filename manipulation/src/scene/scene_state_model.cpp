// Responsibility: Applies validated diffs to the coordinator's canonical in-memory scene model.

#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>

#include "manipulation/scene/coordinator_utils.hpp"
#include "scene_state_model_internal.hpp"

namespace manipulation::scene
{
namespace
{

using detail::AcmState;
using detail::AttachedObject;
using detail::CollisionObject;
using detail::PlanningScene;

template<typename ObjectT, typename IdFunctionT>
std::map<std::string, ObjectT> index_unique_objects(
  std::vector<ObjectT> objects,
  IdFunctionT id_function,
  const std::string & domain)
{
  std::map<std::string, ObjectT> indexed;
  for (auto & object : objects) {
    const auto id = id_function(object);
    if (id.empty() || !indexed.emplace(id, std::move(object)).second) {
      throw std::runtime_error("authoritative " + domain + " contains duplicate or empty IDs");
    }
  }
  return indexed;
}

PlanningScene build_scene(
  const std::map<std::string, CollisionObject> & world,
  const std::map<std::string, AttachedObject> & attached,
  const AcmState & acm)
{
  PlanningScene scene;
  for (const auto & [id, object] : world) {
    (void)id;
    scene.world.collision_objects.push_back(object);
  }
  for (const auto & [id, object] : attached) {
    (void)id;
    scene.robot_state.attached_collision_objects.push_back(object);
  }
  scene.allowed_collision_matrix = acm.to_message();
  return scene;
}

SceneEvolutionResult invalid_evolution(std::string message)
{
  SceneEvolutionResult result;
  result.message = std::move(message);
  return result;
}

}  // namespace

SceneEvolutionResult evolve_authoritative_scene(
  const PlanningScene & authoritative_scene,
  const PlanningScene & requested_diff,
  const std::string & planning_frame)
{
  if (planning_frame.empty()) {
    return invalid_evolution("planning_frame must not be empty");
  }

  PlanningScene baseline;
  AcmState acm;
  try {
    baseline = canonical_authoritative_scene(authoritative_scene);
    acm = AcmState::from_message(baseline.allowed_collision_matrix);
  } catch (const std::exception & error) {
    return invalid_evolution(
      "authoritative scene cannot be modeled safely: " + std::string(error.what()));
  }

  auto world = index_unique_objects<CollisionObject>(
    baseline.world.collision_objects,
    [](const auto & object) {return object.id;}, "world");
  auto attached = index_unique_objects<AttachedObject>(
    baseline.robot_state.attached_collision_objects,
    [](const auto & object) {return object.object.id;}, "attached state");

  SceneEvolutionResult result;
  result.backend_diff = requested_diff;
  std::map<std::string, std::size_t> world_request_indices;
  for (std::size_t index = 0; index < requested_diff.world.collision_objects.size(); ++index) {
    world_request_indices.emplace(
      requested_diff.world.collision_objects[index].id, index);
  }

  std::map<std::string, object_recognition_msgs::msg::ObjectType> detached_types;
  for (std::size_t index = 0;
    index < requested_diff.robot_state.attached_collision_objects.size(); ++index)
  {
    const auto & requested = requested_diff.robot_state.attached_collision_objects[index];
    const auto id = requested.object.id;
    auto & backend_object =
      result.backend_diff.robot_state.attached_collision_objects[index];
    if (requested.object.operation == CollisionObject::ADD) {
      if (world_request_indices.count(id) != 0U) {
        return invalid_evolution(
          "attached ADD for '" + id + "' cannot be combined with a world mutation");
      }
      auto normalized = requested;
      std::string error;
      if (!detail::normalize_attached_object(normalized, error)) {
        return invalid_evolution(error);
      }
      if (detail::type_is_empty(normalized.object.type)) {
        const auto old_attached = attached.find(id);
        const auto old_world = world.find(id);
        if (old_attached != attached.end()) {
          normalized.object.type = old_attached->second.object.type;
        } else if (old_world != world.end()) {
          normalized.object.type = old_world->second.type;
        }
      }
      world.erase(id);
      attached[id] = normalized;
      backend_object = std::move(normalized);
    } else if (requested.object.operation == CollisionObject::REMOVE) {
      const auto current = attached.find(id);
      if (current == attached.end()) {
        return invalid_evolution("cannot detach unknown object '" + id + "'");
      }
      if (!requested.link_name.empty() && requested.link_name != current->second.link_name) {
        return invalid_evolution(
          "detach link for '" + id + "' differs from the authoritative link");
      }
      if (world_request_indices.count(id) == 0U) {
        return invalid_evolution(
          "attached REMOVE for '" + id +
          "' requires an explicit world ADD or REMOVE in the same diff");
      }
      detached_types[id] = current->second.object.type;
      attached.erase(current);
      backend_object.object.header.stamp = builtin_interfaces::msg::Time();
    } else {
      return invalid_evolution("attached objects support only ADD and REMOVE");
    }
  }

  if (!requested_diff.allowed_collision_matrix.entry_names.empty()) {
    const auto & update = requested_diff.allowed_collision_matrix;
    for (std::size_t first = 0; first < update.entry_names.size(); ++first) {
      for (std::size_t second = first + 1U; second < update.entry_names.size(); ++second) {
        acm.set_pair(
          update.entry_names[first], update.entry_names[second],
          update.entry_values[first].enabled[second]);
      }
    }
    result.backend_diff.allowed_collision_matrix = acm.to_message();
    if (result.backend_diff.allowed_collision_matrix.entry_names.empty()) {
      return invalid_evolution(
        "MoveIt cannot represent clearing the complete ACM through a diff message");
    }
  }

  for (std::size_t index = 0; index < requested_diff.world.collision_objects.size(); ++index) {
    const auto & requested = requested_diff.world.collision_objects[index];
    const auto id = requested.id;
    auto & backend_object = result.backend_diff.world.collision_objects[index];
    if (requested.operation == CollisionObject::ADD) {
      if (attached.count(id) != 0U) {
        return invalid_evolution(
          "world ADD for '" + id + "' would duplicate an attached object");
      }
      if (requested.header.frame_id != planning_frame) {
        return invalid_evolution(
          "world ADD for '" + id + "' must use planning_frame '" +
          planning_frame + "'");
      }
      auto normalized = requested;
      std::string error;
      if (!detail::normalize_collision_object(normalized, error)) {
        return invalid_evolution(error);
      }
      if (detail::type_is_empty(normalized.type)) {
        const auto old_world = world.find(id);
        const auto detached_type = detached_types.find(id);
        if (old_world != world.end()) {
          normalized.type = old_world->second.type;
        } else if (detached_type != detached_types.end()) {
          normalized.type = detached_type->second;
        }
      }
      world[id] = normalized;
      backend_object = std::move(normalized);
    } else if (requested.operation == CollisionObject::REMOVE) {
      const bool detached_transient = detached_types.count(id) != 0U;
      if (world.erase(id) == 0U && !detached_transient) {
        return invalid_evolution("cannot remove unknown world object '" + id + "'");
      }
      acm.remove_entries(id);
      backend_object.header.stamp = builtin_interfaces::msg::Time();
    } else {
      return invalid_evolution("world objects support only ADD and REMOVE");
    }
  }

  result.expected_scene = build_scene(world, attached, acm);
  try {
    result.expected_scene = canonical_authoritative_scene(result.expected_scene);
  } catch (const std::exception & error) {
    return invalid_evolution(
      "expected scene cannot be canonicalized: " + std::string(error.what()));
  }
  result.valid = true;
  result.message = "scene transition is deterministic";
  return result;
}

}  // namespace manipulation::scene
