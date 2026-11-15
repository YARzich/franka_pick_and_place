// Responsibility: Shares internal canonicalization and mutation primitives across scene model
//   units.

#ifndef SCENE__SCENE_STATE_MODEL_INTERNAL_HPP_
#define SCENE__SCENE_STATE_MODEL_INTERNAL_HPP_

#include <map>
#include <string>
#include <utility>

#include <moveit_msgs/msg/allowed_collision_matrix.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <object_recognition_msgs/msg/object_type.hpp>

namespace manipulation::scene::detail
{

using AttachedObject = moveit_msgs::msg::AttachedCollisionObject;
using CollisionObject = moveit_msgs::msg::CollisionObject;
using PlanningScene = moveit_msgs::msg::PlanningScene;

bool type_is_empty(const object_recognition_msgs::msg::ObjectType & type);

bool normalize_collision_object(CollisionObject & object, std::string & error);

bool normalize_attached_object(AttachedObject & attached, std::string & error);

/**
 * Canonical semantic ACM representation matching MoveIt's message constructor.
 *
 * The model owns normalized defaults and only pair values that differ from
 * those defaults. Conversion validates a square symmetric matrix and rebuilds
 * deterministic sorted output; no PlanningScene or ROS side effect occurs.
 */
class AcmState
{
public:
  static AcmState from_message(
    const moveit_msgs::msg::AllowedCollisionMatrix & matrix);

  void set_pair(const std::string & first, const std::string & second, bool value);

  void remove_entries(const std::string & name);

  moveit_msgs::msg::AllowedCollisionMatrix to_message() const;

private:
  static std::pair<std::string, std::string> ordered_pair(
    const std::string & first,
    const std::string & second);

  bool pair_default(const std::string & first, const std::string & second) const;

  std::map<std::string, bool> defaults_;
  std::map<std::pair<std::string, std::string>, bool> entries_;
};

}  // namespace manipulation::scene::detail

#endif  // SCENE__SCENE_STATE_MODEL_INTERNAL_HPP_
