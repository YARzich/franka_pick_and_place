// Responsibility: Extracts supported joint or pose goals and start states from MoveIt constraints.

#include "planning_io.hpp"

#include <Eigen/Geometry>

#include <cmath>
#include <map>
#include <string>
#include <vector>

#include <moveit/robot_state/conversions.hpp>
#include <moveit/robot_state/robot_state.hpp>

#include "planning_validation.hpp"

namespace manipulation_curobo::curobo::io
{
namespace
{

bool extractJointGoal(
  const moveit_msgs::msg::Constraints & constraints,
  const moveit::core::JointModelGroup & joint_model_group,
  GoalData & goal,
  std::string & error)
{
  if (constraints.joint_constraints.empty()) {
    return false;
  }
  if (!constraints.position_constraints.empty() ||
    !constraints.orientation_constraints.empty())
  {
    error = "mixed joint and pose constraints are not supported";
    return false;
  }

  std::map<std::string, double> constraint_map;
  for (const auto & constraint : constraints.joint_constraints) {
    if (constraint.joint_name.empty() || !std::isfinite(constraint.position)) {
      error = "joint goal contains an empty name or non-finite position";
      return false;
    }
    if (!constraint_map.emplace(constraint.joint_name, constraint.position).second) {
      error = "joint goal contains a duplicate joint constraint";
      return false;
    }
  }

  const auto & joint_names = joint_model_group.getVariableNames();
  if (constraint_map.size() != joint_names.size()) {
    error = "joint goal must constrain every planning-group variable exactly once";
    return false;
  }

  goal = GoalData{};
  goal.type = GoalData::Type::kJoint;
  goal.joint_positions.reserve(joint_names.size());
  for (const auto & joint_name : joint_names) {
    const auto constraint = constraint_map.find(joint_name);
    if (constraint == constraint_map.end()) {
      error = "joint goal does not match the requested planning group";
      return false;
    }
    goal.joint_positions.push_back(constraint->second);
  }
  return true;
}

bool extractPoseGoal(
  const planning_interface::MotionPlanRequest & request,
  const moveit::core::RobotModelConstPtr & robot_model,
  const std::string & curobo_ee_link,
  const std::string & workspace_frame,
  GoalData & goal,
  std::string & error)
{
  const auto & constraints = request.goal_constraints.front();
  if (constraints.position_constraints.empty() &&
    constraints.orientation_constraints.empty())
  {
    error = "goal contains neither complete joint constraints nor pose constraints";
    return false;
  }
  if (!constraints.joint_constraints.empty() ||
    constraints.position_constraints.size() != 1U ||
    constraints.orientation_constraints.size() != 1U)
  {
    error = "pose goal must contain exactly one position and one orientation constraint";
    return false;
  }

  const auto & position_constraint = constraints.position_constraints.front();
  const auto & orientation_constraint = constraints.orientation_constraints.front();
  if (position_constraint.constraint_region.primitive_poses.size() != 1U) {
    error = "pose goal must contain exactly one primitive pose";
    return false;
  }
  if (!position_constraint.link_name.empty() &&
    !orientation_constraint.link_name.empty() &&
    position_constraint.link_name != orientation_constraint.link_name)
  {
    error = "position and orientation constraints target different links";
    return false;
  }

  const std::string requested_link = position_constraint.link_name.empty() ?
    orientation_constraint.link_name : position_constraint.link_name;
  if (requested_link.empty() || curobo_ee_link.empty() || workspace_frame.empty()) {
    error = "pose goal link, cuRobo end-effector link and workspace frame must be explicit";
    return false;
  }
  if (!position_constraint.header.frame_id.empty() &&
    !orientation_constraint.header.frame_id.empty() &&
    position_constraint.header.frame_id != orientation_constraint.header.frame_id)
  {
    error = "position and orientation constraints use different frames";
    return false;
  }

  const auto & offset = position_constraint.target_point_offset;
  if (!std::isfinite(offset.x) || !std::isfinite(offset.y) || !std::isfinite(offset.z) ||
    std::abs(offset.x) > 1.0e-12 || std::abs(offset.y) > 1.0e-12 ||
    std::abs(offset.z) > 1.0e-12)
  {
    error = "non-zero target-point offsets are not supported by the bridge";
    return false;
  }

  const auto & pose = position_constraint.constraint_region.primitive_poses.front();
  const auto & orientation = orientation_constraint.orientation;
  const std::vector<double> pose_values{
    pose.position.x, pose.position.y, pose.position.z,
    orientation.x, orientation.y, orientation.z, orientation.w};
  if (!detail::allFinite(pose_values)) {
    error = "pose goal contains NaN or Inf";
    return false;
  }

  Eigen::Quaterniond requested_orientation(
    orientation.w, orientation.x, orientation.y, orientation.z);
  if (requested_orientation.norm() <= 1.0e-12) {
    error = "pose goal contains a zero quaternion";
    return false;
  }

  const auto * requested_link_model = robot_model->getLinkModel(requested_link);
  const auto * curobo_link_model = robot_model->getLinkModel(curobo_ee_link);
  if (!requested_link_model || !curobo_link_model ||
    moveit::core::RobotModel::getRigidlyConnectedParentLinkModel(requested_link_model) !=
    moveit::core::RobotModel::getRigidlyConnectedParentLinkModel(curobo_link_model))
  {
    error = "requested pose link is not rigidly connected to the configured cuRobo link";
    return false;
  }

  Eigen::Isometry3d requested_goal_pose = Eigen::Isometry3d::Identity();
  requested_goal_pose.translation() =
    Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  requested_goal_pose.linear() = requested_orientation.normalized().toRotationMatrix();

  // Both targets stay in the explicit request frame. This fixed RobotModel
  // transform changes only the rigidly attached target link; it is not a TF lookup.
  moveit::core::RobotState reference_state(robot_model);
  reference_state.setToDefaultValues();
  reference_state.update();
  const Eigen::Isometry3d requested_to_curobo =
    reference_state.getGlobalLinkTransform(requested_link).inverse() *
    reference_state.getGlobalLinkTransform(curobo_ee_link);
  const Eigen::Isometry3d curobo_goal_pose = requested_goal_pose * requested_to_curobo;
  const Eigen::Quaterniond curobo_orientation(curobo_goal_pose.rotation());

  goal = GoalData{};
  goal.type = GoalData::Type::kPose;
  goal.position = {
    curobo_goal_pose.translation().x(),
    curobo_goal_pose.translation().y(),
    curobo_goal_pose.translation().z()};
  goal.orientation_xyzw = {
    curobo_orientation.x(), curobo_orientation.y(),
    curobo_orientation.z(), curobo_orientation.w()};
  goal.frame_id = position_constraint.header.frame_id;
  if (goal.frame_id.empty()) {
    goal.frame_id = orientation_constraint.header.frame_id;
  }
  if (goal.frame_id.empty()) {
    goal.frame_id = workspace_frame;
  }
  if (goal.frame_id != workspace_frame) {
    error = "pose goal frame_id must match the configured cuRobo workspace frame";
    return false;
  }
  goal.link_name = curobo_ee_link;
  return true;
}

}  // namespace

bool extractStartState(
  const planning_interface::MotionPlanRequest & request,
  const moveit::core::JointModelGroup & joint_model_group,
  const planning_scene::PlanningSceneConstPtr & scene,
  moveit::core::RobotState & resolved_state,
  std::vector<double> & positions,
  std::string & error)
{
  if (!scene) {
    error = "PlanningScene is unavailable";
    return false;
  }
  resolved_state = scene->getCurrentState();
  const bool has_start_override =
    !request.start_state.joint_state.name.empty() ||
    !request.start_state.multi_dof_joint_state.joint_names.empty();
  if (has_start_override &&
    !moveit::core::robotStateMsgToRobotState(
      scene->getTransforms(), request.start_state, resolved_state))
  {
    error = "MoveIt start_state cannot be converted to RobotState";
    return false;
  }
  resolved_state.update();
  resolved_state.copyJointGroupPositions(&joint_model_group, positions);
  if (positions.size() != joint_model_group.getVariableCount() ||
    !detail::allFinite(positions) || !resolved_state.satisfiesBounds())
  {
    error = "resolved start state is incomplete, non-finite or outside joint bounds";
    return false;
  }
  return true;
}

bool extractGoal(
  const planning_interface::MotionPlanRequest & request,
  const moveit::core::RobotModelConstPtr & robot_model,
  const moveit::core::JointModelGroup & joint_model_group,
  const std::string & curobo_ee_link,
  const std::string & workspace_frame,
  GoalData & goal,
  std::string & error)
{
  if (!robot_model) {
    error = "RobotModel is unavailable";
    return false;
  }
  if (request.goal_constraints.size() != 1U) {
    error = "the reference bridge accepts exactly one goal constraint set";
    return false;
  }
  if (extractJointGoal(request.goal_constraints.front(), joint_model_group, goal, error)) {
    return true;
  }
  if (!error.empty()) {
    return false;
  }
  return extractPoseGoal(
    request, robot_model, curobo_ee_link, workspace_frame, goal, error);
}

}  // namespace manipulation_curobo::curobo::io
