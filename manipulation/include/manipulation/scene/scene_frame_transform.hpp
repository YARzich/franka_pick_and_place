// Responsibility: Transforms world and attached collision-object diffs into the planning frame.

#ifndef MANIPULATION__SCENE__SCENE_FRAME_TRANSFORM_HPP_
#define MANIPULATION__SCENE__SCENE_FRAME_TRANSFORM_HPP_

#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>

namespace manipulation::scene
{

/// Result of converting world-object poses to a backend-owned frame.
struct WorldObjectTransformResult
{
  bool valid{false};
  moveit_msgs::msg::PlanningScene scene;
  std::string message;
};

/**
 * Transform complete world-object poses while preserving local shape poses.
 *
 * `transform` must express `child_frame_id` in `header.frame_id`, matching the
 * convention returned by `tf2_ros::Buffer::lookupTransform(target, source)`.
 * ADD objects already in the target frame are left unchanged; ADD objects in
 * any third frame fail closed. REMOVE carries only identity and is untouched.
 * Attached objects are deliberately outside this helper because MoveIt stores
 * them in their attachment-link frame.
 */
WorldObjectTransformResult transform_world_objects_to_backend_frame(
  const moveit_msgs::msg::PlanningScene & scene,
  const std::string & source_frame,
  const std::string & target_frame,
  const geometry_msgs::msg::TransformStamped & transform);

}  // namespace manipulation::scene

#endif  // MANIPULATION__SCENE__SCENE_FRAME_TRANSFORM_HPP_
