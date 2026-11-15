// Responsibility: Applies timestamped TF transforms to every frame-bearing element of a scene diff.

#include "manipulation/scene/scene_frame_transform.hpp"

#include <exception>
#include <string>
#include <utility>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "manipulation/validation/frame_id.hpp"

namespace manipulation::scene
{
namespace
{

WorldObjectTransformResult invalid_result(std::string message)
{
  WorldObjectTransformResult result;
  result.message = std::move(message);
  return result;
}

}  // namespace

WorldObjectTransformResult transform_world_objects_to_backend_frame(
  const moveit_msgs::msg::PlanningScene & scene,
  const std::string & source_frame,
  const std::string & target_frame,
  const geometry_msgs::msg::TransformStamped & transform)
{
  if (!manipulation::validation::is_canonical_relative_frame_id(target_frame) ||
    !manipulation::validation::is_canonical_relative_frame_id(source_frame) ||
    target_frame == source_frame)
  {
    return invalid_result(
      "world-object transform requires distinct REP-compliant source and target frames");
  }
  if (transform.header.frame_id != target_frame ||
    transform.child_frame_id != source_frame)
  {
    return invalid_result(
      "world-object transform direction does not match requested source and target frames");
  }

  auto transformed_scene = scene;
  for (auto & object : transformed_scene.world.collision_objects) {
    if (object.operation == moveit_msgs::msg::CollisionObject::REMOVE ||
      object.header.frame_id == target_frame)
    {
      continue;
    }
    if (object.header.frame_id != source_frame) {
      return invalid_result(
        "world object '" + object.id + "' uses unexpected frame '" +
        object.header.frame_id + "'; expected '" + source_frame + "' or '" +
        target_frame + "'");
    }

    try {
      geometry_msgs::msg::Pose transformed_pose;
      tf2::doTransform(object.pose, transformed_pose, transform);
      object.pose = std::move(transformed_pose);
      object.header.frame_id = target_frame;
    } catch (const std::exception & error) {
      return invalid_result(
        "failed to transform world object '" + object.id + "': " + error.what());
    }
  }

  WorldObjectTransformResult result;
  result.valid = true;
  result.scene = std::move(transformed_scene);
  result.message = "world objects expressed in backend frame";
  return result;
}

}  // namespace manipulation::scene
