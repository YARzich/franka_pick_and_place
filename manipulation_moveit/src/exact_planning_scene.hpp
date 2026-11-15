// Responsibility: Construct one request-owned MoveIt scene without inheriting backend state.

#ifndef EXACT_PLANNING_SCENE_HPP_
#define EXACT_PLANNING_SCENE_HPP_

#include <memory>
#include <string>

#include "moveit/planning_scene/planning_scene.hpp"
#include "moveit/robot_model/robot_model.hpp"
#include "moveit_msgs/msg/planning_scene.hpp"

namespace manipulation::moveit::detail
{

[[nodiscard]] planning_scene::PlanningScenePtr make_exact_planning_scene(
  const ::moveit::core::RobotModelConstPtr & robot_model,
  const moveit_msgs::msg::PlanningScene & snapshot,
  std::string & error);

}  // namespace manipulation::moveit::detail

#endif  // EXACT_PLANNING_SCENE_HPP_
