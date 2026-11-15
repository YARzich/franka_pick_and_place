// Responsibility: Canonicalizes requested scene diffs into MoveIt's authoritative planning frame.

#include "planning_scene_coordinator_node.hpp"

#include <exception>
#include <optional>
#include <string>
#include <utility>

#include <tf2/exceptions.hpp>

#include "manipulation/scene/scene_frame_transform.hpp"

namespace manipulation::scene
{
namespace
{

bool requires_world_frame_transform(
  const moveit_msgs::msg::PlanningScene & scene,
  const std::string & source_frame,
  const std::string & target_frame)
{
  if (source_frame == target_frame) {
    return false;
  }
  for (const auto & object : scene.world.collision_objects) {
    if (object.operation == moveit_msgs::msg::CollisionObject::ADD &&
      object.header.frame_id == source_frame)
    {
      return true;
    }
  }
  return false;
}

}  // namespace

RegistryStatus PlanningSceneCoordinatorNode::validate_scene_dependencies(
  const moveit_msgs::msg::PlanningScene & scene_diff) const
{
  const auto snapshot = registry_->snapshot();
  if (!requires_world_frame_transform(
      scene_diff, planning_frame_, backend_planning_frame_) ||
    world_transform_ready())
  {
    return {
      true, RegistryError::kNone, snapshot.revision,
      "PlanningScene dependencies are ready"};
  }
  return {
    false, RegistryError::kDependencyUnavailable, snapshot.revision,
    "required world-object frame transform is unavailable before dispatch"};
}

std::optional<BackendOperationResult>
PlanningSceneCoordinatorNode::express_transition_in_backend_frame(
  SceneEvolutionResult & transition)
{
  if (!requires_world_frame_transform(
      transition.backend_diff, planning_frame_, backend_planning_frame_))
  {
    return std::nullopt;
  }

  geometry_msgs::msg::TransformStamped frame_transform;
  try {
    frame_transform = tf_buffer_->lookupTransform(
      backend_planning_frame_, planning_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException & error) {
    return BackendOperationResult{
      false,
      "required PlanningScene frame transform is unavailable: " +
      std::string(error.what()),
      RegistryError::kDependencyUnavailable};
  }

  auto backend_diff = transform_world_objects_to_backend_frame(
    transition.backend_diff, planning_frame_, backend_planning_frame_,
    frame_transform);
  auto expected_scene = transform_world_objects_to_backend_frame(
    transition.expected_scene, planning_frame_, backend_planning_frame_,
    frame_transform);
  if (!backend_diff.valid || !expected_scene.valid) {
    const std::string message = !backend_diff.valid ?
      backend_diff.message : expected_scene.message;
    return make_backend_operation_result(BackendCallOutcome::kInternalError, message);
  }

  transition.backend_diff = std::move(backend_diff.scene);
  try {
    transition.expected_scene = canonical_authoritative_scene(expected_scene.scene);
  } catch (const std::exception & error) {
    return make_backend_operation_result(
      BackendCallOutcome::kInternalError,
      "transformed expected PlanningScene is invalid: " + std::string(error.what()));
  }
  return std::nullopt;
}

}  // namespace manipulation::scene
