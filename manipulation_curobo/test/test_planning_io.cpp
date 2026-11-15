// Responsibility: Verifies cuRobo request validation and YAML serialization of planning inputs.

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <moveit/utils/robot_model_test_utils.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <shape_msgs/msg/mesh_triangle.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include "manipulation_curobo/temporary_file.hpp"
#include "planning_io.hpp"

namespace
{

using manipulation_curobo::curobo::TemporaryFile;
namespace planning_io = manipulation_curobo::curobo::io;

class RosEnvironment final : public testing::Environment
{
public:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
      owns_context_ = true;
    }
  }

  void TearDown() override
  {
    if (owns_context_ && rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

private:
  bool owns_context_{false};
};

[[maybe_unused]] const auto * const ros_environment =
  testing::AddGlobalTestEnvironment(new RosEnvironment);

struct PandaFixture
{
  PandaFixture()
  : model(moveit::core::loadTestingRobotModel("panda")),
    group(model ? model->getJointModelGroup("panda_arm") : nullptr),
    start_positions{0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785}
  {
    if (!group || start_positions.size() != group->getVariableCount()) {
      start_positions.clear();
    }
  }

  moveit::core::RobotModelPtr model;
  const moveit::core::JointModelGroup * group;
  std::vector<double> start_positions;
};

void writeResponse(
  const std::filesystem::path & path,
  const std::vector<std::string> & joint_names,
  const double dt,
  const std::size_t position_count,
  const std::vector<double> & first_positions)
{
  YAML::Node root;
  root["success"] = true;
  root["planning_time"] = 0.125;
  root["trajectory"]["dt"] = dt;
  root["trajectory"]["joint_names"] = joint_names;
  for (std::size_t index = 0; index < 2U; ++index) {
    YAML::Node point;
    std::vector<double> positions(position_count, 0.0);
    for (std::size_t variable = 0;
      variable < std::min(position_count, first_positions.size()); ++variable)
    {
      positions[variable] = first_positions[variable] + 0.001 * index;
    }
    point["positions"] = positions;
    point["velocities"] = std::vector<double>(position_count, 0.0);
    point["accelerations"] = std::vector<double>(position_count, 0.0);
    root["trajectory"]["points"].push_back(point);
  }
  std::ofstream stream(path, std::ios::out | std::ios::trunc);
  ASSERT_TRUE(stream.is_open());
  stream << root;
  ASSERT_TRUE(stream.good());
}

planning_scene::PlanningScenePtr makeScene(
  const PandaFixture & fixture, const double finger_position = 0.0)
{
  auto scene = std::make_shared<planning_scene::PlanningScene>(fixture.model);
  auto & state = scene->getCurrentStateNonConst();
  state.setToDefaultValues();
  state.setJointGroupPositions(fixture.group, fixture.start_positions);
  state.setVariablePosition("panda_finger_joint1", finger_position);
  state.setVariablePosition("panda_finger_joint2", finger_position);
  state.update();
  return scene;
}

TEST(PlanningRequest, AcceptsCompleteJointGoalAndRejectsDuplicates)
{
  PandaFixture fixture;
  ASSERT_NE(fixture.model, nullptr);
  ASSERT_NE(fixture.group, nullptr);

  planning_interface::MotionPlanRequest request;
  request.goal_constraints.resize(1U);
  for (const auto & name : fixture.group->getVariableNames()) {
    moveit_msgs::msg::JointConstraint constraint;
    constraint.joint_name = name;
    constraint.position = 0.1;
    request.goal_constraints.front().joint_constraints.push_back(constraint);
  }

  planning_io::GoalData goal;
  std::string error;
  EXPECT_TRUE(planning_io::extractGoal(
      request, fixture.model, *fixture.group, "panda_hand", "world", goal, error));
  EXPECT_EQ(goal.type, planning_io::GoalData::Type::kJoint);
  EXPECT_EQ(goal.joint_positions.size(), fixture.group->getVariableCount());

  request.goal_constraints.front().joint_constraints.push_back(
    request.goal_constraints.front().joint_constraints.front());
  error.clear();
  EXPECT_FALSE(planning_io::extractGoal(
      request, fixture.model, *fixture.group, "panda_hand", "world", goal, error));
  EXPECT_FALSE(error.empty());
}

TEST(PlanningResponse, LoadsOnlyACompleteGroupTrajectory)
{
  PandaFixture fixture;
  ASSERT_NE(fixture.model, nullptr);
  ASSERT_NE(fixture.group, nullptr);
  TemporaryFile response_file{"curobo_response_test"};
  writeResponse(
    response_file.path(), fixture.group->getVariableNames(), 0.01,
    fixture.group->getVariableCount(), fixture.start_positions);

  const auto scene = makeScene(fixture);
  const moveit::core::RobotState resolved_start_state = scene->getCurrentState();
  robot_trajectory::RobotTrajectory trajectory(fixture.model, fixture.group->getName());
  double planning_time = 0.0;
  std::string error;
  EXPECT_EQ(planning_io::loadPlanningResponse(
      response_file.path(), fixture.model, *fixture.group, resolved_start_state,
      scene, 1.0e-4, trajectory, planning_time, error),
    planning_io::BridgeResultCode::kSuccess);
  EXPECT_EQ(trajectory.getWayPointCount(), 2U);
  EXPECT_DOUBLE_EQ(planning_time, 0.125);
}

TEST(PlanningResponse, PreservesResolvedNonGroupJoints)
{
  PandaFixture fixture;
  ASSERT_NE(fixture.model, nullptr);
  ASSERT_NE(fixture.group, nullptr);
  TemporaryFile response_file{"curobo_full_start_state"};
  writeResponse(
    response_file.path(), fixture.group->getVariableNames(), 0.01,
    fixture.group->getVariableCount(), fixture.start_positions);

  constexpr double kFingerPosition = 0.02;
  const auto scene = makeScene(fixture, kFingerPosition);
  const moveit::core::RobotState resolved_start_state = scene->getCurrentState();
  robot_trajectory::RobotTrajectory trajectory(fixture.model, fixture.group->getName());
  double planning_time = 0.0;
  std::string error;

  ASSERT_EQ(
    planning_io::loadPlanningResponse(
      response_file.path(), fixture.model, *fixture.group, resolved_start_state,
      scene, 1.0e-4, trajectory, planning_time, error),
    planning_io::BridgeResultCode::kSuccess) << error;
  ASSERT_EQ(trajectory.getWayPointCount(), 2U);
  EXPECT_DOUBLE_EQ(
    trajectory.getWayPoint(0).getVariablePosition("panda_finger_joint1"),
    kFingerPosition);
  EXPECT_DOUBLE_EQ(
    trajectory.getWayPoint(1).getVariablePosition("panda_finger_joint2"),
    kFingerPosition);
}

TEST(PlanningResponse, RejectsFirstPointOutsideExplicitTolerance)
{
  PandaFixture fixture;
  ASSERT_NE(fixture.model, nullptr);
  ASSERT_NE(fixture.group, nullptr);
  TemporaryFile response_file{"curobo_start_mismatch"};
  auto mismatched_start = fixture.start_positions;
  mismatched_start.front() += 2.0e-4;
  writeResponse(
    response_file.path(), fixture.group->getVariableNames(), 0.01,
    fixture.group->getVariableCount(), mismatched_start);

  const auto scene = makeScene(fixture);
  const moveit::core::RobotState resolved_start_state = scene->getCurrentState();
  robot_trajectory::RobotTrajectory trajectory(fixture.model, fixture.group->getName());
  double planning_time = 0.0;
  std::string error;

  EXPECT_EQ(
    planning_io::loadPlanningResponse(
      response_file.path(), fixture.model, *fixture.group, resolved_start_state,
      scene, 1.0e-4, trajectory, planning_time, error),
    planning_io::BridgeResultCode::kInvalidResponse);
  EXPECT_NE(error.find("first point"), std::string::npos);
}

TEST(PlanningResponse, RejectsOutOfBoundsAndSceneInvalidWaypoints)
{
  PandaFixture fixture;
  ASSERT_NE(fixture.model, nullptr);
  ASSERT_NE(fixture.group, nullptr);
  TemporaryFile response_file{"curobo_invalid_waypoint"};
  writeResponse(
    response_file.path(), fixture.group->getVariableNames(), 0.01,
    fixture.group->getVariableCount(), fixture.start_positions);
  YAML::Node response = YAML::LoadFile(response_file.path().string());
  response["trajectory"]["points"][1]["positions"][0] = 100.0;
  {
    std::ofstream stream(response_file.path(), std::ios::out | std::ios::trunc);
    ASSERT_TRUE(stream.is_open());
    stream << response;
  }

  auto scene = makeScene(fixture);
  moveit::core::RobotState resolved_start_state = scene->getCurrentState();
  robot_trajectory::RobotTrajectory trajectory(fixture.model, fixture.group->getName());
  double planning_time = 0.0;
  std::string error;
  EXPECT_EQ(
    planning_io::loadPlanningResponse(
      response_file.path(), fixture.model, *fixture.group, resolved_start_state,
      scene, 1.0e-4, trajectory, planning_time, error),
    planning_io::BridgeResultCode::kInvalidResponse);
  EXPECT_NE(error.find("joint bounds"), std::string::npos);

  writeResponse(
    response_file.path(), fixture.group->getVariableNames(), 0.01,
    fixture.group->getVariableCount(), fixture.start_positions);
  moveit_msgs::msg::CollisionObject blocker;
  blocker.header.frame_id = scene->getPlanningFrame();
  blocker.id = "blocking_world";
  blocker.operation = moveit_msgs::msg::CollisionObject::ADD;
  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {10.0, 10.0, 10.0};
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  blocker.primitives.push_back(box);
  blocker.primitive_poses.push_back(pose);
  ASSERT_TRUE(scene->processCollisionObjectMsg(blocker));
  ASSERT_FALSE(scene->isStateValid(
      resolved_start_state, fixture.group->getName(), false));
  error.clear();
  EXPECT_EQ(
    planning_io::loadPlanningResponse(
      response_file.path(), fixture.model, *fixture.group, resolved_start_state,
      scene, 1.0e-4, trajectory, planning_time, error),
    planning_io::BridgeResultCode::kInvalidResponse);
  EXPECT_NE(error.find("immutable PlanningScene"), std::string::npos);
}

TEST(PlanningResponse, RejectsWrongPointWidthAndNonPositiveStep)
{
  PandaFixture fixture;
  ASSERT_NE(fixture.model, nullptr);
  ASSERT_NE(fixture.group, nullptr);
  TemporaryFile response_file{"curobo_invalid_response"};

  const auto scene = makeScene(fixture);
  const moveit::core::RobotState resolved_start_state = scene->getCurrentState();
  robot_trajectory::RobotTrajectory trajectory(fixture.model, fixture.group->getName());
  double planning_time = 0.0;
  std::string error;
  writeResponse(
    response_file.path(), fixture.group->getVariableNames(), 0.01,
    fixture.group->getVariableCount() - 1U, fixture.start_positions);
  EXPECT_NE(planning_io::loadPlanningResponse(
      response_file.path(), fixture.model, *fixture.group, resolved_start_state,
      scene, 1.0e-4, trajectory, planning_time, error),
    planning_io::BridgeResultCode::kSuccess);
  EXPECT_FALSE(error.empty());

  error.clear();
  writeResponse(
    response_file.path(), fixture.group->getVariableNames(), 0.0,
    fixture.group->getVariableCount(), fixture.start_positions);
  EXPECT_NE(planning_io::loadPlanningResponse(
      response_file.path(), fixture.model, *fixture.group, resolved_start_state,
      scene, 1.0e-4, trajectory, planning_time, error),
    planning_io::BridgeResultCode::kSuccess);
  EXPECT_FALSE(error.empty());
}

TEST(PlanningResponse, RejectsDuplicateOrForeignJointNames)
{
  PandaFixture fixture;
  ASSERT_NE(fixture.model, nullptr);
  ASSERT_NE(fixture.group, nullptr);
  TemporaryFile response_file{"curobo_joint_names"};
  auto names = fixture.group->getVariableNames();
  names.back() = names.front();
  writeResponse(
    response_file.path(), names, 0.01, names.size(), fixture.start_positions);

  const auto scene = makeScene(fixture);
  const moveit::core::RobotState resolved_start_state = scene->getCurrentState();
  robot_trajectory::RobotTrajectory trajectory(fixture.model, fixture.group->getName());
  double planning_time = 0.0;
  std::string error;
  EXPECT_NE(planning_io::loadPlanningResponse(
      response_file.path(), fixture.model, *fixture.group, resolved_start_state,
      scene, 1.0e-4, trajectory, planning_time, error),
    planning_io::BridgeResultCode::kSuccess);
  EXPECT_FALSE(error.empty());
}

TEST(PlanningResponse, PreservesTypedBridgeFailure)
{
  PandaFixture fixture;
  ASSERT_NE(fixture.model, nullptr);
  ASSERT_NE(fixture.group, nullptr);
  TemporaryFile response_file{"curobo_typed_response"};
  YAML::Node root;
  root["success"] = false;
  root["error_code"] = "unsupported_scene";
  root["message"] = "partial ACM is unsupported";
  root["planning_time"] = 0.0;
  std::ofstream stream(response_file.path(), std::ios::out | std::ios::trunc);
  ASSERT_TRUE(stream.is_open());
  stream << root;
  stream.close();

  const auto scene = makeScene(fixture);
  const moveit::core::RobotState resolved_start_state = scene->getCurrentState();
  robot_trajectory::RobotTrajectory trajectory(fixture.model, fixture.group->getName());
  double planning_time = 0.0;
  std::string error;
  EXPECT_EQ(
    planning_io::loadPlanningResponse(
      response_file.path(), fixture.model, *fixture.group, resolved_start_state,
      scene, 1.0e-4, trajectory, planning_time, error),
    planning_io::BridgeResultCode::kUnsupportedScene);
  EXPECT_EQ(error, "partial ACM is unsupported");
}

TEST(PlanningResponse, PreservesTypedPlanningBudgetExhaustion)
{
  PandaFixture fixture;
  ASSERT_NE(fixture.model, nullptr);
  ASSERT_NE(fixture.group, nullptr);
  TemporaryFile response_file{"curobo_timeout_response"};
  YAML::Node root;
  root["success"] = false;
  root["error_code"] = "timed_out";
  root["message"] = "planning budget exhausted";
  root["planning_time"] = 5.0;
  std::ofstream stream(response_file.path(), std::ios::out | std::ios::trunc);
  ASSERT_TRUE(stream.is_open());
  stream << root;
  stream.close();

  const auto scene = makeScene(fixture);
  const moveit::core::RobotState resolved_start_state = scene->getCurrentState();
  robot_trajectory::RobotTrajectory trajectory(fixture.model, fixture.group->getName());
  double planning_time = 0.0;
  std::string error;
  EXPECT_EQ(
    planning_io::loadPlanningResponse(
      response_file.path(), fixture.model, *fixture.group, resolved_start_state,
      scene, 1.0e-4, trajectory, planning_time, error),
    planning_io::BridgeResultCode::kTimedOut);
  EXPECT_EQ(error, "planning budget exhausted");
}

TEST(PlanningRequest, RejectsWorkspaceMismatchForJointGoal)
{
  PandaFixture fixture;
  ASSERT_NE(fixture.model, nullptr);
  ASSERT_NE(fixture.group, nullptr);
  const auto scene = makeScene(fixture);
  planning_interface::MotionPlanRequest request;
  planning_io::GoalData goal;
  goal.type = planning_io::GoalData::Type::kJoint;
  goal.joint_positions = fixture.start_positions;
  TemporaryFile request_file{"curobo_workspace_mismatch"};
  std::string error;

  EXPECT_EQ(
    planning_io::writePlanningRequest(
      request, scene, scene->getCurrentState(), fixture.start_positions,
      fixture.group->getVariableNames(), goal, "/tmp/motion_gen.yaml",
      "different_workspace", request_file.path(), error),
    planning_io::BridgeResultCode::kUnsupportedScene);
  EXPECT_NE(error.find("planning frame"), std::string::npos);
}

TEST(PlanningRequest, SerializesCompleteImmutableSceneDeterministically)
{
  PandaFixture fixture;
  ASSERT_NE(fixture.model, nullptr);
  ASSERT_NE(fixture.group, nullptr);
  auto scene = std::make_shared<planning_scene::PlanningScene>(fixture.model);
  scene->getCurrentStateNonConst().setToDefaultValues();
  scene->getCurrentStateNonConst().setJointGroupPositions(
    fixture.group, fixture.start_positions);
  scene->getCurrentStateNonConst().update();
  scene->getTransformsNonConst().setTransform(
    Eigen::Translation3d(0.1, 0.2, 0.3) * Eigen::Isometry3d::Identity(),
    "fixture_frame");

  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = "fixture_frame";
  object.pose.orientation.w = 1.0;
  object.id = "compound";
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {0.4, 0.2, 0.1};
  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  object.primitives.push_back(box);
  object.primitive_poses.push_back(box_pose);
  shape_msgs::msg::SolidPrimitive cylinder;
  cylinder.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  cylinder.dimensions = {0.3, 0.05};
  geometry_msgs::msg::Pose cylinder_pose;
  cylinder_pose.position.x = 0.3;
  cylinder_pose.orientation.w = 1.0;
  object.primitives.push_back(cylinder);
  object.primitive_poses.push_back(cylinder_pose);
  shape_msgs::msg::Mesh mesh;
  mesh.vertices.resize(3U);
  mesh.vertices[1].x = 0.1;
  mesh.vertices[2].y = 0.1;
  shape_msgs::msg::MeshTriangle triangle;
  triangle.vertex_indices = {0U, 1U, 2U};
  mesh.triangles.push_back(triangle);
  geometry_msgs::msg::Pose mesh_pose;
  mesh_pose.position.y = 0.2;
  mesh_pose.orientation.w = 1.0;
  object.meshes.push_back(mesh);
  object.mesh_poses.push_back(mesh_pose);
  ASSERT_TRUE(scene->processCollisionObjectMsg(object));

  moveit_msgs::msg::AttachedCollisionObject attached;
  attached.link_name = "panda_hand";
  attached.touch_links = {"panda_hand", "panda_leftfinger", "panda_rightfinger"};
  attached.object.header.frame_id = "panda_hand";
  attached.object.pose.orientation.w = 1.0;
  attached.object.id = "payload";
  attached.object.operation = moveit_msgs::msg::CollisionObject::ADD;
  shape_msgs::msg::SolidPrimitive payload;
  payload.type = shape_msgs::msg::SolidPrimitive::SPHERE;
  payload.dimensions = {0.03};
  geometry_msgs::msg::Pose payload_pose;
  payload_pose.orientation.w = 1.0;
  attached.object.primitives.push_back(payload);
  attached.object.primitive_poses.push_back(payload_pose);
  ASSERT_TRUE(scene->processAttachedCollisionObjectMsg(attached));
  scene->getAllowedCollisionMatrixNonConst().setEntry(
    "compound", "panda_link0", true);

  planning_interface::MotionPlanRequest request;
  request.allowed_planning_time = 1.5;
  planning_io::GoalData goal;
  goal.type = planning_io::GoalData::Type::kJoint;
  goal.joint_positions.assign(fixture.group->getVariableCount(), 0.1);
  TemporaryFile request_file{"curobo_scene_request"};
  std::string error;
  ASSERT_EQ(
    planning_io::writePlanningRequest(
      request, scene, scene->getCurrentState(), fixture.start_positions,
      fixture.group->getVariableNames(), goal, "/tmp/motion_gen.yaml",
      scene->getPlanningFrame(), request_file.path(), error),
    planning_io::BridgeResultCode::kSuccess) << error;

  const YAML::Node root = YAML::LoadFile(request_file.path().string());
  EXPECT_EQ(root["schema_version"].as<int>(), 2);
  EXPECT_EQ(
    root["workspace_frame"].as<std::string>(), scene->getPlanningFrame());
  EXPECT_FALSE(root["scene"]["is_diff"].as<bool>());
  EXPECT_FALSE(root["scene"]["robot_state"]["is_diff"].as<bool>());
  ASSERT_EQ(root["scene"]["world_objects"].size(), 1U);
  EXPECT_EQ(root["scene"]["world_objects"][0]["id"].as<std::string>(), "compound");
  ASSERT_EQ(root["scene"]["world_objects"][0]["parts"].size(), 3U);
  EXPECT_EQ(
    root["scene"]["world_objects"][0]["parts"][0]["name"].as<std::string>(),
    "compound__primitive_000");
  EXPECT_NEAR(
    root["scene"]["world_objects"][0]["parts"][0]["pose"][0].as<double>(),
    0.1, 1.0e-12);
  EXPECT_NEAR(
    root["scene"]["world_objects"][0]["parts"][0]["pose"][1].as<double>(),
    0.2, 1.0e-12);
  EXPECT_EQ(root["scene"]["attached_objects"].size(), 1U);
  EXPECT_EQ(
    root["scene"]["attached_objects"][0]["object"]["id"].as<std::string>(),
    "payload");

  bool found_fixture_transform = false;
  for (const auto & transform : root["scene"]["frame_transforms"]) {
    found_fixture_transform = found_fixture_transform ||
      transform["child_frame"].as<std::string>() == "fixture_frame";
  }
  EXPECT_TRUE(found_fixture_transform);
  EXPECT_FALSE(root["scene"]["allowed_collision_matrix"]["entry_names"].IsNull());
  EXPECT_GT(
    root["scene"]["robot_state"]["joint_names"].size(),
    fixture.group->getVariableCount());
}

}  // namespace
