// Responsibility: Construct one request-owned MoveIt scene without inheriting backend state.

#include "exact_planning_scene.hpp"

#include <exception>
#include <memory>
#include <string>

namespace manipulation::moveit::detail
{

planning_scene::PlanningScenePtr make_exact_planning_scene(
  const ::moveit::core::RobotModelConstPtr & robot_model,
  const moveit_msgs::msg::PlanningScene & snapshot,
  std::string & error)
{
  error.clear();
  if (!robot_model) {
    error = "exact PlanningScene construction requires a RobotModel";
    return {};
  }
  if (snapshot.is_diff || snapshot.robot_state.is_diff) {
    error = "exact PlanningScene input must be a complete non-diff snapshot";
    return {};
  }
  if (!snapshot.robot_model_name.empty() &&
    snapshot.robot_model_name != robot_model->getName())
  {
    error = "PlanningScene robot model does not match the active MoveGroup model";
    return {};
  }

  try {
    auto scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
    if (!scene->setPlanningSceneMsg(snapshot)) {
      error = "MoveIt rejected the complete PlanningScene snapshot";
      return {};
    }
    return scene;
  } catch (const std::exception & exception) {
    error = std::string{"cannot construct exact PlanningScene: "} + exception.what();
  } catch (...) {
    error = "cannot construct exact PlanningScene: unknown exception";
  }
  return {};
}

}  // namespace manipulation::moveit::detail
