// Responsibility: Declares robot-neutral structural validation for complete PlanningScene objects.

#ifndef MANIPULATION__SCENE__COLLISION_OBJECT_VALIDATION_HPP_
#define MANIPULATION__SCENE__COLLISION_OBJECT_VALIDATION_HPP_

#include <string>

#include "moveit_msgs/msg/collision_object.hpp"

namespace manipulation::scene
{

/// Result of validating one complete world-object ADD request.
struct CollisionObjectValidationResult
{
  bool valid{false};
  std::string message;
};

/**
 * Validate an explicit CollisionObject without ROS calls or data mutation.
 *
 * The object must describe a complete `ADD`: canonical identity and relative
 * frame, normalized stamp (the zero readback sentinel is allowed), explicit
 * non-degenerate geometry, one valid rigid pose per shape, and unique valid
 * subframes. Geometry-free `REMOVE` and incremental `APPEND`/`MOVE` messages
 * intentionally belong to separate mutation-specific contracts.
 */
[[nodiscard]] CollisionObjectValidationResult validate_explicit_collision_object(
  const moveit_msgs::msg::CollisionObject & object);

}  // namespace manipulation::scene

#endif  // MANIPULATION__SCENE__COLLISION_OBJECT_VALIDATION_HPP_
