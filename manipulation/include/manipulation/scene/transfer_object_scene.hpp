// Responsibility: Declares robot-neutral resource, geometry, and diff helpers for staged
//   TransferObject scene lifecycles.

#ifndef MANIPULATION__SCENE__TRANSFER_OBJECT_SCENE_HPP_
#define MANIPULATION__SCENE__TRANSFER_OBJECT_SCENE_HPP_

#include <string>
#include <vector>

#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_msgs/msg/planning_scene.hpp"

namespace manipulation::scene
{

/// Canonical resources held for the complete staged-object lifecycle.
struct TransferObjectSceneResources
{
  std::string world;
  std::string attached;
  std::vector<std::string> allowed_collision_pairs;
  std::vector<std::string> all;
};

/**
 * Build world, attached, and object-to-touch-link ACM resource keys.
 *
 * Touch links must be non-empty and unique. The returned vectors are sorted so
 * lease requests and validation proofs have deterministic payloads.
 * `std::invalid_argument` is thrown for any non-canonical component.
 */
[[nodiscard]] TransferObjectSceneResources make_transfer_object_scene_resources(
  const std::string & object_id,
  const std::vector<std::string> & touch_links);

/**
 * Validate a complete perception-supplied world CollisionObject.
 *
 * The object must be an ADD in `planning_frame`, carry a normalized non-zero
 * observation stamp, explicit valid geometry, one pose per shape, finite rigid
 * poses, and valid subframes. The helper performs no TF lookup and never fills
 * missing geometry or timestamps.
 */
[[nodiscard]] bool validate_staged_collision_object(
  const moveit_msgs::msg::CollisionObject & object,
  const std::string & planning_frame,
  std::string & reason);

/// Wrap a validated complete object in a coordinator-compatible world ADD diff.
[[nodiscard]] moveit_msgs::msg::PlanningScene make_staged_world_add_diff(
  const moveit_msgs::msg::CollisionObject & object,
  const std::string & planning_frame);

/// Build one pair-scoped ACM allow diff for the staged object and a touch link.
[[nodiscard]] moveit_msgs::msg::PlanningScene make_staged_contact_allow_diff(
  const std::string & object_id,
  const std::string & touch_link);

/**
 * Remove one confirmed world object.
 *
 * Coordinator scene evolution also removes every ACM entry naming this object;
 * callers therefore must own and prove all corresponding ACM resources before
 * applying this diff.
 */
[[nodiscard]] moveit_msgs::msg::PlanningScene make_staged_world_remove_diff(
  const std::string & object_id,
  const std::string & planning_frame);

}  // namespace manipulation::scene

#endif  // MANIPULATION__SCENE__TRANSFER_OBJECT_SCENE_HPP_
