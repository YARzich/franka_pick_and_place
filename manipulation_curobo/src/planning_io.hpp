// Responsibility: Defines the internal data contract for cuRobo request serialization and result
//   parsing.

#ifndef PLANNING_IO_HPP_
#define PLANNING_IO_HPP_

#include <array>
#include <filesystem>
#include <string>
#include <vector>

#include <moveit/planning_interface/planning_interface.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_model/joint_model_group.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>

namespace manipulation_curobo::curobo::io
{

/** Canonical goal data accepted by the file bridge. */
struct GoalData
{
  enum class Type
  {
    kJoint,
    kPose,
  };

  Type type{Type::kJoint};
  std::vector<double> joint_positions;
  std::array<double, 3> position{};
  std::array<double, 4> orientation_xyzw{};
  std::string frame_id;
  std::string link_name;
};

/** Typed terminal state written by the private one-shot bridge. */
enum class BridgeResultCode
{
  kSuccess,
  kPlanningFailed,
  kTimedOut,
  kInvalidRequest,
  kUnsupportedScene,
  kInternalError,
  kInvalidResponse,
};

/** Resolve and validate the complete start state used by both planners. */
[[nodiscard]] bool extractStartState(
  const planning_interface::MotionPlanRequest & request,
  const moveit::core::JointModelGroup & joint_model_group,
  const planning_scene::PlanningSceneConstPtr & scene,
  moveit::core::RobotState & resolved_state,
  std::vector<double> & positions,
  std::string & error);

/** Convert one unambiguous MoveIt joint or pose goal to the bridge contract. */
[[nodiscard]] bool extractGoal(
  const planning_interface::MotionPlanRequest & request,
  const moveit::core::RobotModelConstPtr & robot_model,
  const moveit::core::JointModelGroup & joint_model_group,
  const std::string & curobo_ee_link,
  const std::string & workspace_frame,
  GoalData & goal,
  std::string & error);

/** Serialize one validated planning request to a private temporary file. */
[[nodiscard]] BridgeResultCode writePlanningRequest(
  const planning_interface::MotionPlanRequest & request,
  const planning_scene::PlanningSceneConstPtr & scene,
  const moveit::core::RobotState & resolved_start_state,
  const std::vector<double> & start_positions,
  const std::vector<std::string> & joint_names,
  const GoalData & goal,
  const std::string & motion_gen_config,
  const std::string & workspace_frame,
  const std::filesystem::path & path,
  std::string & error);

/** Parse and validate bridge output before exposing it to MoveIt. */
[[nodiscard]] BridgeResultCode loadPlanningResponse(
  const std::filesystem::path & response_path,
  const moveit::core::RobotModelConstPtr & model,
  const moveit::core::JointModelGroup & joint_model_group,
  const moveit::core::RobotState & resolved_start_state,
  const planning_scene::PlanningSceneConstPtr & scene,
  double start_point_tolerance,
  robot_trajectory::RobotTrajectory & trajectory,
  double & planning_time,
  std::string & error);

}  // namespace manipulation_curobo::curobo::io

#endif  // PLANNING_IO_HPP_
