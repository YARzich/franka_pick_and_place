// Responsibility: Declares the MoveIt PlannerManager adapter that admits requests to cuRobo
//   workers.

#ifndef MANIPULATION_CUROBO__CUROBO_PLANNER_MANAGER_HPP_
#define MANIPULATION_CUROBO__CUROBO_PLANNER_MANAGER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <moveit/planning_interface/planning_interface.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>

#include "manipulation_curobo/planning_admission.hpp"

namespace manipulation_curobo
{

/**
 * MoveIt plugin facade for the `manipulation_curobo/CuRoboPlanner` ID.
 *
 * The manager owns immutable backend configuration and one shared admission
 * gate after initialize(). Each request receives a separate context, while the
 * gate bounds how many contexts may enter the external GPU backend at once.
 */
class CuRoboPlannerManager final : public planning_interface::PlannerManager
{
public:
  /** Construct an uninitialized plugin manager. */
  CuRoboPlannerManager();

  /** Resolve parameters and prove backend readiness without MotionGen warmup. */
  bool initialize(
    const moveit::core::RobotModelConstPtr & model,
    const rclcpp::Node::SharedPtr & node,
    const std::string & parameter_namespace) override;

  /** Create one isolated context that shares this manager's admission gate. */
  planning_interface::PlanningContextPtr getPlanningContext(
    const planning_scene::PlanningSceneConstPtr & planning_scene,
    const planning_interface::MotionPlanRequest & request,
    moveit_msgs::msg::MoveItErrorCodes & error_code) const override;

  /** Return whether the request group and single goal set are structurally supported. */
  bool canServiceRequest(
    const planning_interface::MotionPlanRequest & request) const override;

  /** Append the stable `curobo` algorithm identifier. */
  void getPlanningAlgorithms(std::vector<std::string> & algorithms) const override;

  /** Return a human-readable backend description. */
  std::string getDescription() const override;

private:
  moveit::core::RobotModelConstPtr robot_model_;
  rclcpp::Logger logger_;
  rclcpp::Node::WeakPtr node_;
  std::string python_executable_;
  std::string planner_script_;
  std::string motion_gen_config_;
  std::string workspace_frame_;
  std::string curobo_ee_link_;
  std::shared_ptr<curobo::PlanningAdmissionGate> admission_gate_;
};

}  // namespace manipulation_curobo

#endif  // MANIPULATION_CUROBO__CUROBO_PLANNER_MANAGER_HPP_
