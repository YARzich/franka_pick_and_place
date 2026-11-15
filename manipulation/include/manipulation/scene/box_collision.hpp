// Responsibility: Defines canonical construction and validation of box-shaped scene objects.

#ifndef MANIPULATION__SCENE__BOX_COLLISION_HPP_
#define MANIPULATION__SCENE__BOX_COLLISION_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include "manipulation/validation/frame_id.hpp"

namespace manipulation::scene
{

/**
 * Inputs for one axis-aligned world collision box.
 *
 * `position` is the box centre expressed in `frame_id`; dimensions and position
 * are `[x, y, z]` in metres. The box has identity orientation in that frame.
 */
struct BoxCollisionSpec
{
  std::string object_id;
  std::string frame_id;
  std::array<double, 3> dimensions{};
  std::array<double, 3> position{};
};

/**
 * Convert a ROS parameter vector to a fixed three-axis value.
 *
 * @throws std::invalid_argument when the vector does not contain exactly
 * three finite values.
 */
inline std::array<double, 3> parse_vector3(
  const std::vector<double> & values,
  const std::string & parameter_name)
{
  if (values.size() != 3U) {
    throw std::invalid_argument(
            parameter_name + " must contain exactly three values");
  }
  if (!std::all_of(values.begin(), values.end(), [](const double value) {
      return std::isfinite(value);
    }))
  {
    throw std::invalid_argument(parameter_name + " must contain finite values");
  }
  return {values[0], values[1], values[2]};
}

/// Return whether IDs, dimensions and coordinates form a valid box.
inline bool is_valid(const BoxCollisionSpec & spec)
{
  if (spec.object_id.empty() ||
    !manipulation::validation::is_canonical_relative_frame_id(spec.frame_id))
  {
    return false;
  }
  for (const double dimension : spec.dimensions) {
    if (!std::isfinite(dimension) || dimension <= 0.0) {
      return false;
    }
  }
  return std::all_of(
    spec.position.begin(), spec.position.end(),
    [](const double coordinate) {return std::isfinite(coordinate);});
}

/**
 * Build a MoveIt world-object ADD mutation from a validated box.
 *
 * Geometry is stored in the object's top-level pose while the primitive pose is
 * identity, avoiding two competing transforms for the same physical box.
 *
 * @throws std::invalid_argument when the specification is invalid.
 */
inline moveit_msgs::msg::CollisionObject make_box_collision_object(
  const BoxCollisionSpec & spec)
{
  if (!is_valid(spec)) {
    throw std::invalid_argument("Invalid box collision specification");
  }

  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = spec.frame_id;
  collision_object.id = spec.object_id;
  collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;
  collision_object.pose.position.x = spec.position[0];
  collision_object.pose.position.y = spec.position[1];
  collision_object.pose.position.z = spec.position[2];
  collision_object.pose.orientation.w = 1.0;

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions.assign(spec.dimensions.begin(), spec.dimensions.end());

  geometry_msgs::msg::Pose primitive_pose;
  primitive_pose.orientation.w = 1.0;
  collision_object.primitives.push_back(std::move(primitive));
  collision_object.primitive_poses.push_back(std::move(primitive_pose));
  return collision_object;
}

/// Build the minimal PlanningScene diff containing one world-object ADD.
inline moveit_msgs::msg::PlanningScene make_planning_scene_diff(
  const BoxCollisionSpec & spec)
{
  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;
  scene.world.collision_objects.push_back(make_box_collision_object(spec));
  return scene;
}

}  // namespace manipulation::scene

#endif  // MANIPULATION__SCENE__BOX_COLLISION_HPP_
