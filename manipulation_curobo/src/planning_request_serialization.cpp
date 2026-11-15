// Responsibility: Serialize one immutable MoveIt request and complete collision-scene snapshot.

#include "planning_io.hpp"

#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include "planning_validation.hpp"

namespace manipulation_curobo::curobo::io
{
namespace
{

constexpr int kPrivateSchemaVersion = 2;
constexpr double kQuaternionEpsilon = 1.0e-12;

bool poseToEigen(
  const geometry_msgs::msg::Pose & pose, Eigen::Isometry3d & transform,
  std::string & error)
{
  const std::vector<double> values{
    pose.position.x, pose.position.y, pose.position.z,
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w};
  if (!detail::allFinite(values)) {
    error = "scene pose contains NaN or Inf";
    return false;
  }
  Eigen::Quaterniond orientation(
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  if (orientation.norm() <= kQuaternionEpsilon) {
    error = "scene pose contains a zero quaternion";
    return false;
  }
  transform = Eigen::Isometry3d::Identity();
  transform.translation() =
    Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  transform.linear() = orientation.normalized().toRotationMatrix();
  return true;
}

YAML::Node poseNode(const Eigen::Isometry3d & transform)
{
  const Eigen::Quaterniond orientation(transform.rotation());
  YAML::Node node;
  node.push_back(transform.translation().x());
  node.push_back(transform.translation().y());
  node.push_back(transform.translation().z());
  node.push_back(orientation.x());
  node.push_back(orientation.y());
  node.push_back(orientation.z());
  node.push_back(orientation.w());
  return node;
}

std::string partName(
  const std::string & object_id, const char * kind, const std::size_t index)
{
  std::ostringstream stream;
  stream << object_id << "__" << kind << "_" << std::setw(3) << std::setfill('0') << index;
  return stream.str();
}

bool positiveDimensions(
  const shape_msgs::msg::SolidPrimitive & primitive, const std::size_t count)
{
  return primitive.dimensions.size() == count &&
         std::all_of(
    primitive.dimensions.begin(), primitive.dimensions.end(), [](const double value) {
      return std::isfinite(value) && value > 0.0;
    });
}

bool serializePrimitive(
  const shape_msgs::msg::SolidPrimitive & primitive,
  const Eigen::Isometry3d & planning_pose,
  const std::string & object_id, const std::size_t index,
  YAML::Node & part, std::string & error)
{
  part["name"] = partName(object_id, "primitive", index);
  part["owner_id"] = object_id;
  part["pose"] = poseNode(planning_pose);
  switch (primitive.type) {
    case shape_msgs::msg::SolidPrimitive::BOX:
      if (!positiveDimensions(primitive, 3U)) {
        error = "box dimensions must contain three finite positive values";
        return false;
      }
      part["kind"] = "cuboid";
      part["dims"] = std::vector<double>(
        primitive.dimensions.begin(), primitive.dimensions.end());
      return true;
    case shape_msgs::msg::SolidPrimitive::SPHERE:
      if (!positiveDimensions(primitive, 1U)) {
        error = "sphere radius must be finite and positive";
        return false;
      }
      part["kind"] = "sphere";
      part["radius"] = primitive.dimensions[shape_msgs::msg::SolidPrimitive::SPHERE_RADIUS];
      return true;
    case shape_msgs::msg::SolidPrimitive::CYLINDER:
      if (!positiveDimensions(primitive, 2U)) {
        error = "cylinder dimensions must contain finite positive height and radius";
        return false;
      }
      part["kind"] = "cylinder";
      part["height"] =
        primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT];
      part["radius"] =
        primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS];
      return true;
    case shape_msgs::msg::SolidPrimitive::CONE:
      if (!positiveDimensions(primitive, 2U)) {
        error = "cone dimensions must contain finite positive height and radius";
        return false;
      }
      // cuRobo has no native cone. The bridge converts this exact height/radius
      // contract into a deterministic conservative 32-sided mesh envelope.
      part["kind"] = "cone";
      part["height"] = primitive.dimensions[shape_msgs::msg::SolidPrimitive::CONE_HEIGHT];
      part["radius"] = primitive.dimensions[shape_msgs::msg::SolidPrimitive::CONE_RADIUS];
      return true;
    default:
      error = "PlanningScene contains an unsupported solid primitive type";
      return false;
  }
}

bool serializeMesh(
  const shape_msgs::msg::Mesh & mesh,
  const Eigen::Isometry3d & planning_pose,
  const std::string & object_id, const std::size_t index,
  YAML::Node & part, std::string & error)
{
  if (mesh.vertices.empty() || mesh.triangles.empty()) {
    error = "mesh must contain vertices and triangles";
    return false;
  }
  YAML::Node vertices;
  for (const auto & vertex : mesh.vertices) {
    const std::vector<double> coordinates{vertex.x, vertex.y, vertex.z};
    if (!detail::allFinite(coordinates)) {
      error = "mesh contains a non-finite vertex";
      return false;
    }
    vertices.push_back(coordinates);
  }
  YAML::Node faces;
  for (const auto & triangle : mesh.triangles) {
    YAML::Node face;
    for (const auto vertex_index : triangle.vertex_indices) {
      if (vertex_index >= mesh.vertices.size()) {
        error = "mesh triangle references a missing vertex";
        return false;
      }
      face.push_back(vertex_index);
    }
    faces.push_back(face);
  }
  part["name"] = partName(object_id, "mesh", index);
  part["owner_id"] = object_id;
  part["kind"] = "mesh";
  part["pose"] = poseNode(planning_pose);
  part["vertices"] = vertices;
  part["faces"] = faces;
  return true;
}

bool serializeCollisionObject(
  const moveit_msgs::msg::CollisionObject & object,
  const planning_scene::PlanningSceneConstPtr & scene,
  const moveit::core::RobotState & resolved_state,
  const std::string & fallback_frame,
  YAML::Node & node, std::set<std::string> & referenced_frames,
  std::string & error)
{
  if (object.id.empty()) {
    error = "collision object id must not be empty";
    return false;
  }
  if (object.operation != moveit_msgs::msg::CollisionObject::ADD) {
    error = "an immutable PlanningScene snapshot may contain only ADD collision objects";
    return false;
  }
  if (!object.planes.empty()) {
    error = "cuRobo backend does not support infinite PlanningScene planes";
    return false;
  }
  if (object.primitives.size() != object.primitive_poses.size() ||
    object.meshes.size() != object.mesh_poses.size() ||
    object.subframe_names.size() != object.subframe_poses.size())
  {
    error = "collision object geometry and pose arrays have different lengths";
    return false;
  }
  if (object.primitives.empty() && object.meshes.empty()) {
    error = "collision object has no supported geometry";
    return false;
  }

  const std::string source_frame = object.header.frame_id.empty() ?
    fallback_frame : object.header.frame_id;
  if (source_frame.empty() || !scene->knowsFrameTransform(resolved_state, source_frame)) {
    error = "collision object source frame is unknown to the immutable scene: " + source_frame;
    return false;
  }
  referenced_frames.insert(source_frame);

  Eigen::Isometry3d object_pose;
  if (!poseToEigen(object.pose, object_pose, error)) {
    return false;
  }
  const Eigen::Isometry3d planning_object_pose =
    scene->getFrameTransform(resolved_state, source_frame) * object_pose;

  node["id"] = object.id;
  node["source_frame"] = source_frame;
  node["object_pose"] = poseNode(planning_object_pose);
  YAML::Node parts;
  for (std::size_t index = 0; index < object.primitives.size(); ++index) {
    Eigen::Isometry3d part_pose;
    if (!poseToEigen(object.primitive_poses[index], part_pose, error)) {
      return false;
    }
    YAML::Node part;
    if (!serializePrimitive(
        object.primitives[index], planning_object_pose * part_pose,
        object.id, index, part, error))
    {
      return false;
    }
    parts.push_back(part);
  }
  for (std::size_t index = 0; index < object.meshes.size(); ++index) {
    Eigen::Isometry3d part_pose;
    if (!poseToEigen(object.mesh_poses[index], part_pose, error)) {
      return false;
    }
    YAML::Node part;
    if (!serializeMesh(
        object.meshes[index], planning_object_pose * part_pose,
        object.id, index, part, error))
    {
      return false;
    }
    parts.push_back(part);
  }
  node["parts"] = parts;

  YAML::Node subframes;
  for (std::size_t index = 0; index < object.subframe_names.size(); ++index) {
    Eigen::Isometry3d subframe_pose;
    if (object.subframe_names[index].empty() ||
      !poseToEigen(object.subframe_poses[index], subframe_pose, error))
    {
      error = error.empty() ? "collision object subframe name must not be empty" : error;
      return false;
    }
    YAML::Node subframe;
    subframe["name"] = object.id + "/" + object.subframe_names[index];
    subframe["pose"] = poseNode(planning_object_pose * subframe_pose);
    subframes.push_back(subframe);
  }
  node["subframes"] = subframes;
  return true;
}

bool serializeAllowedCollisionMatrix(
  const moveit_msgs::msg::AllowedCollisionMatrix & matrix,
  YAML::Node & node, std::string & error)
{
  if (matrix.entry_names.size() != matrix.entry_values.size() ||
    matrix.default_entry_names.size() != matrix.default_entry_values.size())
  {
    error = "AllowedCollisionMatrix names and value arrays have different lengths";
    return false;
  }
  std::set<std::string> names;
  for (std::size_t row = 0; row < matrix.entry_names.size(); ++row) {
    if (matrix.entry_names[row].empty() ||
      !names.insert(matrix.entry_names[row]).second ||
      matrix.entry_values[row].enabled.size() != matrix.entry_names.size())
    {
      error = "AllowedCollisionMatrix must be square with unique non-empty names";
      return false;
    }
  }
  node["entry_names"] = matrix.entry_names;
  YAML::Node entries;
  for (const auto & row : matrix.entry_values) {
    YAML::Node values;
    for (const bool enabled : row.enabled) {
      values.push_back(enabled);
    }
    entries.push_back(values);
  }
  node["entries"] = entries;
  node["default_entry_names"] = matrix.default_entry_names;
  YAML::Node defaults;
  for (const bool enabled : matrix.default_entry_values) {
    defaults.push_back(enabled);
  }
  node["default_entries"] = defaults;
  return true;
}

bool serializeScene(
  const planning_scene::PlanningSceneConstPtr & scene,
  const moveit::core::RobotState & resolved_state,
  YAML::Node & node, std::string & error)
{
  if (!scene) {
    error = "PlanningScene is unavailable";
    return false;
  }
  moveit_msgs::msg::PlanningScene snapshot;
  scene->getPlanningSceneMsg(snapshot);
  if (snapshot.is_diff || snapshot.robot_state.is_diff) {
    error = "cuRobo requires a complete non-diff PlanningScene snapshot";
    return false;
  }
  if (!snapshot.world.octomap.octomap.data.empty()) {
    error = "cuRobo file bridge does not support PlanningScene octomaps";
    return false;
  }

  node["name"] = snapshot.name;
  node["robot_model_name"] = snapshot.robot_model_name;
  node["planning_frame"] = scene->getPlanningFrame();
  node["is_diff"] = false;
  node["robot_state"]["is_diff"] = false;
  moveit_msgs::msg::RobotState resolved_state_message;
  moveit::core::robotStateToRobotStateMsg(
    resolved_state, resolved_state_message, false);
  node["robot_state"]["joint_names"] = resolved_state_message.joint_state.name;
  node["robot_state"]["joint_positions"] = resolved_state_message.joint_state.position;
  node["robot_state"]["joint_velocities"] = resolved_state_message.joint_state.velocity;
  node["robot_state"]["joint_effort"] = resolved_state_message.joint_state.effort;

  std::set<std::string> referenced_frames{scene->getPlanningFrame()};
  std::vector<moveit_msgs::msg::CollisionObject> world_objects =
    snapshot.world.collision_objects;
  std::sort(
    world_objects.begin(), world_objects.end(),
    [](const auto & left, const auto & right) {return left.id < right.id;});
  std::set<std::string> object_ids;
  YAML::Node world;
  for (const auto & object : world_objects) {
    if (!object_ids.insert(object.id).second) {
      error = "PlanningScene contains duplicate collision object ids";
      return false;
    }
    YAML::Node object_node;
    if (!serializeCollisionObject(
        object, scene, resolved_state, scene->getPlanningFrame(), object_node,
        referenced_frames, error))
    {
      return false;
    }
    world.push_back(object_node);
  }
  node["world_objects"] = world;

  auto attached_objects = snapshot.robot_state.attached_collision_objects;
  std::sort(
    attached_objects.begin(), attached_objects.end(),
    [](const auto & left, const auto & right) {return left.object.id < right.object.id;});
  YAML::Node attached;
  for (const auto & attached_object : attached_objects) {
    if (attached_object.link_name.empty() ||
      !object_ids.insert(attached_object.object.id).second)
    {
      error = "attached collision object needs a unique id and explicit link_name";
      return false;
    }
    if (!std::isfinite(attached_object.weight) || attached_object.weight < 0.0) {
      error = "attached collision object weight must be finite and non-negative";
      return false;
    }
    referenced_frames.insert(attached_object.link_name);
    YAML::Node attached_node;
    attached_node["link_name"] = attached_object.link_name;
    attached_node["touch_links"] = attached_object.touch_links;
    attached_node["weight"] = attached_object.weight;
    YAML::Node object_node;
    if (!serializeCollisionObject(
        attached_object.object, scene, resolved_state, attached_object.link_name,
        object_node, referenced_frames, error))
    {
      return false;
    }
    attached_node["object"] = object_node;
    attached.push_back(attached_node);
  }
  node["attached_objects"] = attached;

  YAML::Node matrix;
  if (!serializeAllowedCollisionMatrix(snapshot.allowed_collision_matrix, matrix, error)) {
    return false;
  }
  node["allowed_collision_matrix"] = matrix;

  for (const auto & transform : scene->getTransforms().getAllTransforms()) {
    referenced_frames.insert(transform.first);
  }
  if (scene->getRobotModel()) {
    for (const auto & link_name : scene->getRobotModel()->getLinkModelNames()) {
      referenced_frames.insert(link_name);
    }
  }
  YAML::Node transforms;
  for (const auto & frame : referenced_frames) {
    if (frame.empty() || !scene->knowsFrameTransform(resolved_state, frame)) {
      error = "PlanningScene cannot resolve referenced frame: " + frame;
      return false;
    }
    YAML::Node transform;
    transform["parent_frame"] = scene->getPlanningFrame();
    transform["child_frame"] = frame;
    transform["pose"] = poseNode(scene->getFrameTransform(resolved_state, frame));
    transforms.push_back(transform);
  }
  node["frame_transforms"] = transforms;
  return true;
}

}  // namespace

BridgeResultCode writePlanningRequest(
  const planning_interface::MotionPlanRequest & request,
  const planning_scene::PlanningSceneConstPtr & scene,
  const moveit::core::RobotState & resolved_start_state,
  const std::vector<double> & start_positions,
  const std::vector<std::string> & joint_names,
  const GoalData & goal,
  const std::string & motion_gen_config,
  const std::string & workspace_frame,
  const std::filesystem::path & path,
  std::string & error)
{
  if (joint_names.empty() || start_positions.size() != joint_names.size() ||
    !detail::allFinite(start_positions) || motion_gen_config.empty() ||
    workspace_frame.empty() || path.empty())
  {
    error = "request metadata or start state is incomplete";
    return BridgeResultCode::kInvalidRequest;
  }
  const std::set<std::string> unique_joint_names(joint_names.begin(), joint_names.end());
  if (unique_joint_names.size() != joint_names.size() || unique_joint_names.count("") != 0U) {
    error = "planning-group joint names must be non-empty and unique";
    return BridgeResultCode::kInvalidRequest;
  }
  if (!std::isfinite(request.allowed_planning_time) || request.allowed_planning_time < 0.0 ||
    !detail::validScale(request.max_velocity_scaling_factor) ||
    !detail::validScale(request.max_acceleration_scaling_factor))
  {
    error = "planning timeout or scaling factor is outside its valid range";
    return BridgeResultCode::kInvalidRequest;
  }
  if (goal.type == GoalData::Type::kJoint &&
    (goal.joint_positions.size() != joint_names.size() ||
    !detail::allFinite(goal.joint_positions)))
  {
    error = "joint goal does not match the serialized planning group";
    return BridgeResultCode::kInvalidRequest;
  }
  if (goal.type == GoalData::Type::kPose && goal.frame_id != workspace_frame) {
    error = "pose goal frame must match the configured cuRobo workspace frame";
    return BridgeResultCode::kInvalidRequest;
  }
  if (!scene || scene->getPlanningFrame().empty() ||
    scene->getPlanningFrame() != workspace_frame)
  {
    error = "PlanningScene planning frame must match the configured cuRobo workspace frame";
    return BridgeResultCode::kUnsupportedScene;
  }
  if (resolved_start_state.getRobotModel() != scene->getRobotModel()) {
    error = "resolved start state does not belong to the PlanningScene RobotModel";
    return BridgeResultCode::kInvalidRequest;
  }

  YAML::Node root;
  root["schema_version"] = kPrivateSchemaVersion;
  root["motion_gen_config_file"] = motion_gen_config;
  root["workspace_frame"] = workspace_frame;
  root["joint_names"] = joint_names;
  root["start_positions"] = start_positions;
  root["plan_timeout"] = request.allowed_planning_time > 0.0 ?
    request.allowed_planning_time : 5.0;
  root["max_attempts"] = std::max(1, request.num_planning_attempts);
  if (request.max_velocity_scaling_factor > 0.0) {
    root["velocity_scale"] = request.max_velocity_scaling_factor;
  }
  if (request.max_acceleration_scaling_factor > 0.0) {
    root["acceleration_scale"] = request.max_acceleration_scaling_factor;
  }

  YAML::Node goal_node;
  if (goal.type == GoalData::Type::kJoint) {
    goal_node["type"] = "joint";
    goal_node["joint_positions"] = goal.joint_positions;
  } else {
    goal_node["type"] = "pose";
    goal_node["frame_id"] = goal.frame_id;
    goal_node["link_name"] = goal.link_name;
    goal_node["position"] = std::vector<double>(goal.position.begin(), goal.position.end());
    goal_node["orientation"] =
      std::vector<double>(goal.orientation_xyzw.begin(), goal.orientation_xyzw.end());
  }
  root["goal"] = goal_node;

  YAML::Node scene_node;
  if (!serializeScene(scene, resolved_start_state, scene_node, error)) {
    return BridgeResultCode::kUnsupportedScene;
  }
  root["scene"] = scene_node;

  std::ofstream stream(path, std::ios::out | std::ios::trunc);
  if (!stream.is_open()) {
    error = "cannot open the private request file";
    return BridgeResultCode::kInternalError;
  }
  stream << root;
  stream.flush();
  if (!stream.good()) {
    error = "cannot finish writing the private request file";
    return BridgeResultCode::kInternalError;
  }
  return BridgeResultCode::kSuccess;
}

}  // namespace manipulation_curobo::curobo::io
