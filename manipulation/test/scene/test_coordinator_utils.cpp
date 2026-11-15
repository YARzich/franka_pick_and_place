// Responsibility: Verifies coordinator request validation and canonical scene resource derivation.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <manipulation_msgs/msg/planning_scene_operation_result.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <moveit_msgs/msg/allowed_collision_entry.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <shape_msgs/msg/plane.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include "manipulation/scene/coordinator_utils.hpp"
#include "manipulation/scene/robot_description_frame.hpp"
#include "manipulation/scene/scene_frame_transform.hpp"

namespace
{

using namespace std::chrono_literals;
using manipulation::scene::BackendCallOutcome;
using manipulation::scene::RegistryError;

moveit_msgs::msg::PlanningScene world_object_diff(const std::string & object_id = "cube")
{
  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;
  moveit_msgs::msg::CollisionObject object;
  object.id = object_id;
  object.header.frame_id = "robot_base";
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  object.pose.orientation.w = 1.0;
  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {0.05, 0.05, 0.05};
  object.primitives.push_back(box);
  geometry_msgs::msg::Pose shape_pose;
  shape_pose.orientation.w = 1.0;
  object.primitive_poses.push_back(shape_pose);
  scene.world.collision_objects.push_back(std::move(object));
  return scene;
}

TEST(CoordinatorDuration, ValidatesAndRoundTripsPositiveTtl)
{
  builtin_interfaces::msg::Duration input;
  input.sec = 2;
  input.nanosec = 250'000'000U;
  const auto parsed = manipulation::scene::parse_positive_steady_duration(input);
  ASSERT_TRUE(parsed.valid);
  EXPECT_EQ(parsed.value, 2250ms);

  const auto output = manipulation::scene::to_duration_message(parsed.value);
  ASSERT_TRUE(output);
  EXPECT_EQ(output->sec, input.sec);
  EXPECT_EQ(output->nanosec, input.nanosec);
}

TEST(CoordinatorDuration, RejectsMalformedOrUnrepresentableTtl)
{
  builtin_interfaces::msg::Duration duration;
  EXPECT_FALSE(manipulation::scene::parse_positive_steady_duration(duration).valid);

  duration.sec = -1;
  EXPECT_FALSE(manipulation::scene::parse_positive_steady_duration(duration).valid);

  duration.sec = 1;
  duration.nanosec = 1'000'000'000U;
  EXPECT_FALSE(manipulation::scene::parse_positive_steady_duration(duration).valid);

  EXPECT_FALSE(manipulation::scene::to_duration_message(0s));
  EXPECT_FALSE(manipulation::scene::to_duration_message(
      std::chrono::steady_clock::duration::max()));
}

TEST(CoordinatorContract, MapsEveryRegistryErrorExplicitly)
{
  using Result = manipulation_msgs::msg::PlanningSceneOperationResult;
  const std::array<std::pair<RegistryError, std::uint16_t>, 18> cases{{
    {RegistryError::kNone, Result::NONE},
    {RegistryError::kInvalidRequest, Result::INVALID_REQUEST},
    {RegistryError::kResourceConflict, Result::RESOURCE_CONFLICT},
    {RegistryError::kLeaseNotFound, Result::LEASE_NOT_FOUND},
    {RegistryError::kLeaseExpired, Result::LEASE_EXPIRED},
    {RegistryError::kNotOwner, Result::NOT_OWNER},
    {RegistryError::kRevisionMismatch, Result::REVISION_MISMATCH},
    {RegistryError::kBackendFailed, Result::BACKEND_FAILED},
    {RegistryError::kTimeout, Result::TIMEOUT},
    {RegistryError::kSessionMismatch, Result::SESSION_MISMATCH},
    {RegistryError::kIdempotencyConflict, Result::IDEMPOTENCY_CONFLICT},
    {RegistryError::kOperationInProgress, Result::OPERATION_IN_PROGRESS},
    {RegistryError::kOperationTicketInvalid, Result::OPERATION_TICKET_INVALID},
    {RegistryError::kOperationCapacityExceeded, Result::OPERATION_CAPACITY_EXCEEDED},
    {RegistryError::kReconciliationRequired, Result::RECONCILIATION_REQUIRED},
    {RegistryError::kDependencyUnavailable, Result::DEPENDENCY_UNAVAILABLE},
    {RegistryError::kResourceQuarantined, Result::RESOURCE_QUARANTINED},
    {RegistryError::kInternalError, Result::INTERNAL_ERROR},
  }};

  for (const auto & [registry_error, interface_error] : cases) {
    EXPECT_EQ(manipulation::scene::to_interface_error(registry_error), interface_error);
  }
  EXPECT_EQ(
    manipulation::scene::to_interface_error(static_cast<RegistryError>(999U)),
    Result::INTERNAL_ERROR);
}

TEST(CoordinatorContract, ConvertsStatusAndStateWithoutNarrowing)
{
  const auto result = manipulation::scene::to_interface_result(
    {false, RegistryError::kTimeout, 17, "timeout"});
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.error_code, result.TIMEOUT);
  EXPECT_EQ(result.revision, 17U);
  EXPECT_EQ(result.message, "timeout");

  const auto state = manipulation::scene::to_interface_state(
    {"session", 17, 2, 3, 8, false, true, 4, {"world:cube"}});
  ASSERT_TRUE(state);
  EXPECT_EQ(state->session_id, "session");
  EXPECT_EQ(state->revision, 17U);
  EXPECT_EQ(state->active_lease_count, 2U);
  EXPECT_EQ(state->operation_record_count, 3U);
  EXPECT_EQ(state->operation_record_capacity, 8U);
  EXPECT_FALSE(state->operation_capacity_exhausted);
  EXPECT_TRUE(state->operation_in_progress);
  EXPECT_EQ(state->expired_lease_count, 4U);
  EXPECT_EQ(state->quarantined_resource_count, 1U);
  EXPECT_EQ(state->quarantined_resource_keys, std::vector<std::string>{"world:cube"});

  auto oversized = manipulation::scene::RegistrySnapshot{};
  oversized.active_lease_count =
    static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max()) + 1U;
  EXPECT_FALSE(manipulation::scene::to_interface_state(oversized));
}

TEST(CoordinatorContract, MapsBoundedBackendOutcomes)
{
  const auto success = manipulation::scene::make_backend_operation_result(
    BackendCallOutcome::kSuccess, "ok");
  EXPECT_TRUE(success.success);
  EXPECT_EQ(success.failure_error, RegistryError::kNone);

  EXPECT_EQ(
    manipulation::scene::make_backend_operation_result(
      BackendCallOutcome::kUnavailable, "unavailable").failure_error,
    RegistryError::kBackendFailed);
  EXPECT_EQ(
    manipulation::scene::make_backend_operation_result(
      BackendCallOutcome::kRejected, "rejected").failure_error,
    RegistryError::kBackendFailed);
  EXPECT_EQ(
    manipulation::scene::make_backend_operation_result(
      BackendCallOutcome::kTimeout, "timeout").failure_error,
    RegistryError::kTimeout);
  EXPECT_EQ(
    manipulation::scene::make_backend_operation_result(
      BackendCallOutcome::kInternalError, "internal").failure_error,
    RegistryError::kInternalError);
}

TEST(CoordinatorDigest, CoversFullRequestAndCanonicalizesResourceSetOrder)
{
  manipulation_msgs::srv::ApplyPlanningSceneDiff::Request request;
  request.lease_id = "session/lease/1";
  request.operation_id = "operation-1";
  request.expected_revision = 4;
  request.resource_keys = {"world:cube", "acm:a|b"};
  request.scene_diff = world_object_diff();

  const auto original = manipulation::scene::digest_apply_request(request);
  EXPECT_EQ(original.size(), 64U);
  EXPECT_EQ(original, manipulation::scene::digest_apply_request(request));

  std::swap(request.resource_keys[0], request.resource_keys[1]);
  EXPECT_EQ(original, manipulation::scene::digest_apply_request(request));

  ++request.expected_revision;
  EXPECT_NE(original, manipulation::scene::digest_apply_request(request));
  --request.expected_revision;
  request.scene_diff.world.collision_objects.front().id = "other";
  EXPECT_NE(original, manipulation::scene::digest_apply_request(request));
}

TEST(CoordinatorDigest, CanonicalizesAuthoritativeOwnedDomains)
{
  auto first = world_object_diff("first");
  auto second_object = world_object_diff("second").world.collision_objects.front();
  first.world.collision_objects.push_back(second_object);
  first.world.collision_objects.front().header.stamp.sec = 12;

  auto reordered = first;
  std::reverse(
    reordered.world.collision_objects.begin(),
    reordered.world.collision_objects.end());
  reordered.world.collision_objects.back().header.stamp.sec = 99;

  const auto digest = manipulation::scene::digest_authoritative_scene(first);
  EXPECT_EQ(digest.size(), 64U);
  EXPECT_EQ(digest, manipulation::scene::digest_authoritative_scene(reordered));

  reordered.world.collision_objects.front().id = "changed";
  EXPECT_NE(digest, manipulation::scene::digest_authoritative_scene(reordered));
}

TEST(SceneFrameTransform, ConvertsOnlyWorldObjectBasePoses)
{
  auto scene = world_object_diff("cube");
  scene.world.collision_objects.reserve(2U);
  auto & object = scene.world.collision_objects.front();
  object.pose.position.x = 0.2;
  object.primitive_poses.front().position.y = 0.03;
  object.meshes.emplace_back();
  geometry_msgs::msg::Pose mesh_pose;
  mesh_pose.position.x = 0.04;
  mesh_pose.orientation.w = 1.0;
  object.mesh_poses.push_back(mesh_pose);
  object.planes.emplace_back();
  object.planes.front().coef = {0.0, 0.0, 1.0, 0.0};
  geometry_msgs::msg::Pose plane_pose;
  plane_pose.position.z = 0.05;
  plane_pose.orientation.w = 1.0;
  object.plane_poses.push_back(plane_pose);
  object.subframe_names.push_back("grasp_origin");
  geometry_msgs::msg::Pose subframe_pose;
  subframe_pose.position.y = 0.06;
  subframe_pose.orientation.w = 1.0;
  object.subframe_poses.push_back(subframe_pose);

  moveit_msgs::msg::AttachedCollisionObject attached;
  attached.link_name = "tool_mount";
  attached.object = object;
  attached.object.header.frame_id = attached.link_name;
  scene.robot_state.attached_collision_objects.push_back(attached);
  moveit_msgs::msg::CollisionObject removed;
  removed.id = "removed";
  removed.header.frame_id = "map";
  removed.operation = moveit_msgs::msg::CollisionObject::REMOVE;
  scene.world.collision_objects.push_back(removed);

  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "world";
  transform.child_frame_id = "robot_base";
  transform.transform.translation.x = 1.0;
  transform.transform.translation.y = 2.0;
  transform.transform.translation.z = 3.0;
  const double half_sqrt = std::sqrt(0.5);
  transform.transform.rotation.z = half_sqrt;
  transform.transform.rotation.w = half_sqrt;

  const auto result =
    manipulation::scene::transform_world_objects_to_backend_frame(
    scene, "robot_base", "world", transform);
  ASSERT_TRUE(result.valid) << result.message;
  ASSERT_EQ(result.scene.world.collision_objects.size(), 2U);
  const auto & transformed = result.scene.world.collision_objects.front();
  EXPECT_EQ(transformed.header.frame_id, "world");
  EXPECT_NEAR(transformed.pose.position.x, 1.0, 1e-12);
  EXPECT_NEAR(transformed.pose.position.y, 2.2, 1e-12);
  EXPECT_NEAR(transformed.pose.position.z, 3.0, 1e-12);
  EXPECT_NEAR(transformed.pose.orientation.z, half_sqrt, 1e-12);
  EXPECT_NEAR(transformed.pose.orientation.w, half_sqrt, 1e-12);
  ASSERT_EQ(transformed.primitive_poses.size(), 1U);
  EXPECT_DOUBLE_EQ(transformed.primitive_poses.front().position.y, 0.03);
  EXPECT_EQ(transformed.mesh_poses, object.mesh_poses);
  EXPECT_EQ(transformed.plane_poses, object.plane_poses);
  EXPECT_EQ(transformed.subframe_poses, object.subframe_poses);
  EXPECT_EQ(
    result.scene.robot_state.attached_collision_objects,
    scene.robot_state.attached_collision_objects);
  ASSERT_EQ(result.scene.world.collision_objects.size(), 2U);
  EXPECT_EQ(result.scene.world.collision_objects[1], removed);

  // The helper is pure: caller-owned input remains in the public frame.
  EXPECT_EQ(scene.world.collision_objects.front().header.frame_id, "robot_base");
}

TEST(SceneFrameTransform, PromotesLegacySingleShapePoseBeforeTransform)
{
  auto request = world_object_diff();
  auto & requested_object = request.world.collision_objects.front();
  requested_object.pose = geometry_msgs::msg::Pose{};
  requested_object.pose.orientation.w = 0.0;
  requested_object.primitive_poses.front().position.x = 0.25;
  requested_object.header.stamp.sec = 42;

  moveit_msgs::msg::PlanningScene baseline;
  const auto transition = manipulation::scene::evolve_authoritative_scene(
    baseline, request, "robot_base");
  ASSERT_TRUE(transition.valid) << transition.message;
  ASSERT_EQ(transition.expected_scene.world.collision_objects.size(), 1U);
  const auto & promoted = transition.expected_scene.world.collision_objects.front();
  EXPECT_DOUBLE_EQ(promoted.pose.position.x, 0.25);
  EXPECT_DOUBLE_EQ(promoted.primitive_poses.front().position.x, 0.0);
  EXPECT_EQ(promoted.header.stamp.sec, 0);
  EXPECT_EQ(promoted.header.stamp.nanosec, 0U);

  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "world";
  transform.child_frame_id = "robot_base";
  transform.transform.translation.z = 1.0;
  transform.transform.rotation.w = 1.0;
  const auto transformed =
    manipulation::scene::transform_world_objects_to_backend_frame(
    transition.expected_scene, "robot_base", "world", transform);
  ASSERT_TRUE(transformed.valid) << transformed.message;
  const auto & backend_object = transformed.scene.world.collision_objects.front();
  EXPECT_EQ(backend_object.header.frame_id, "world");
  EXPECT_DOUBLE_EQ(backend_object.pose.position.x, 0.25);
  EXPECT_DOUBLE_EQ(backend_object.pose.position.z, 1.0);
  EXPECT_EQ(
    manipulation::scene::digest_authoritative_scene(transformed.scene),
    manipulation::scene::digest_authoritative_scene(
      manipulation::scene::canonical_authoritative_scene(transformed.scene)));
}

TEST(RobotDescriptionFrame, ParsesFixedRootAndRejectsInvalidUrdf)
{
  const auto parsed = manipulation::scene::parse_fixed_root_frame(
    "<robot name='test_robot'>"
    "<link name='world'/><link name='robot_base'/>"
    "<joint name='root' type='fixed'><parent link='world'/>"
    "<child link='robot_base'/></joint></robot>");
  ASSERT_TRUE(parsed.valid) << parsed.message;
  EXPECT_EQ(parsed.frame_id, "world");

  EXPECT_FALSE(manipulation::scene::parse_fixed_root_frame("").valid);
  EXPECT_FALSE(
    manipulation::scene::parse_fixed_root_frame("<robot name='broken'>").valid);
}

TEST(SceneFrameTransform, PreservesBackendObjectsAndRejectsThirdFrames)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "world";
  transform.child_frame_id = "robot_base";
  transform.transform.rotation.w = 1.0;

  auto backend_scene = world_object_diff();
  backend_scene.world.collision_objects.front().header.frame_id = "world";
  const auto unchanged =
    manipulation::scene::transform_world_objects_to_backend_frame(
    backend_scene, "robot_base", "world", transform);
  ASSERT_TRUE(unchanged.valid) << unchanged.message;
  EXPECT_EQ(
    manipulation::scene::digest_authoritative_scene(backend_scene),
    manipulation::scene::digest_authoritative_scene(unchanged.scene));

  auto unexpected = world_object_diff();
  unexpected.world.collision_objects.front().header.frame_id = "map";
  const auto rejected =
    manipulation::scene::transform_world_objects_to_backend_frame(
    unexpected, "robot_base", "world", transform);
  EXPECT_FALSE(rejected.valid);
  EXPECT_NE(rejected.message.find("unexpected frame"), std::string::npos);

  std::swap(transform.header.frame_id, transform.child_frame_id);
  EXPECT_FALSE(
    manipulation::scene::transform_world_objects_to_backend_frame(
      backend_scene, "robot_base", "world", transform).valid);
}

TEST(CoordinatorDigest, CanonicalizesAllowedCollisionMatrixOrdering)
{
  moveit_msgs::msg::PlanningScene first;
  first.allowed_collision_matrix.entry_names = {"b", "a"};
  first.allowed_collision_matrix.entry_values.resize(2);
  first.allowed_collision_matrix.entry_values[0].enabled = {false, true};
  first.allowed_collision_matrix.entry_values[1].enabled = {true, false};
  first.allowed_collision_matrix.default_entry_names = {"z", "y"};
  first.allowed_collision_matrix.default_entry_values = {true, false};

  moveit_msgs::msg::PlanningScene reordered;
  reordered.allowed_collision_matrix.entry_names = {"a", "b"};
  reordered.allowed_collision_matrix.entry_values.resize(2);
  reordered.allowed_collision_matrix.entry_values[0].enabled = {false, true};
  reordered.allowed_collision_matrix.entry_values[1].enabled = {true, false};
  reordered.allowed_collision_matrix.default_entry_names = {"y", "z"};
  reordered.allowed_collision_matrix.default_entry_values = {false, true};

  EXPECT_EQ(
    manipulation::scene::digest_authoritative_scene(first),
    manipulation::scene::digest_authoritative_scene(reordered));
}

TEST(CoordinatorDigest, GeneratesVersionFourSessionUuid)
{
  const auto session_id = manipulation::scene::generate_session_id();
  ASSERT_EQ(session_id.size(), 36U);
  EXPECT_EQ(session_id[8], '-');
  EXPECT_EQ(session_id[13], '-');
  EXPECT_EQ(session_id[18], '-');
  EXPECT_EQ(session_id[23], '-');
  EXPECT_EQ(session_id[14], '4');
  EXPECT_NE(std::string("89ab").find(session_id[19]), std::string::npos);
}

TEST(SceneDiffValidation, AcceptsExactlyCoveredWorldObjectMutation)
{
  const auto result = manipulation::scene::validate_scene_diff_resources(
    world_object_diff(), {"world:cube"});
  ASSERT_TRUE(result.valid) << result.message;
  ASSERT_EQ(result.canonical_resource_keys.size(), 1U);
  EXPECT_EQ(result.canonical_resource_keys.front(), "world:cube");
}

TEST(SceneDiffValidation, RejectsMissingExtraOrDuplicateCoverage)
{
  const auto scene = world_object_diff();
  EXPECT_FALSE(manipulation::scene::validate_scene_diff_resources(scene, {}).valid);
  EXPECT_FALSE(manipulation::scene::validate_scene_diff_resources(
      scene, {"world:cube", "attached:cube"}).valid);
  EXPECT_FALSE(manipulation::scene::validate_scene_diff_resources(
      scene, {"world:cube", "world:cube"}).valid);

  auto duplicate = scene;
  duplicate.world.collision_objects.push_back(
    duplicate.world.collision_objects.front());
  EXPECT_FALSE(manipulation::scene::validate_scene_diff_resources(
      duplicate, {"world:cube"}).valid);
}

TEST(SceneDiffValidation, RequiresAttachedObjectDiffAndLease)
{
  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;
  moveit_msgs::msg::AttachedCollisionObject attached;
  attached.object.id = "cube";
  attached.object.header.frame_id = "robot_base";
  attached.object.operation = moveit_msgs::msg::CollisionObject::ADD;
  attached.link_name = "robot_base";
  attached.object = world_object_diff().world.collision_objects.front();
  attached.object.header.frame_id = attached.link_name;
  scene.robot_state.attached_collision_objects.push_back(attached);

  EXPECT_FALSE(manipulation::scene::validate_scene_diff_resources(
      scene, {"attached:cube"}).valid);
  scene.robot_state.is_diff = true;
  EXPECT_FALSE(manipulation::scene::validate_scene_diff_resources(
      scene, {"attached:cube"}).valid);
  EXPECT_TRUE(manipulation::scene::validate_scene_diff_resources(
      scene, {"attached:cube", "world:cube"}).valid);
}

TEST(SceneDiffValidation, RequiresFrameForNonRemoveObjects)
{
  auto world = world_object_diff();
  world.world.collision_objects.front().header.frame_id.clear();
  EXPECT_FALSE(manipulation::scene::validate_scene_diff_resources(
      world, {"world:cube"}).valid);

  world.world.collision_objects.front().header.frame_id = "/robot_base";
  EXPECT_FALSE(manipulation::scene::validate_scene_diff_resources(
      world, {"world:cube"}).valid);
  world.world.collision_objects.front().header.frame_id = "bad frame";
  EXPECT_FALSE(manipulation::scene::validate_scene_diff_resources(
      world, {"world:cube"}).valid);

  world.world.collision_objects.front().operation =
    moveit_msgs::msg::CollisionObject::REMOVE;
  EXPECT_TRUE(manipulation::scene::validate_scene_diff_resources(
      world, {"world:cube"}).valid);

  moveit_msgs::msg::PlanningScene attached;
  attached.is_diff = true;
  attached.robot_state.is_diff = true;
  moveit_msgs::msg::AttachedCollisionObject object;
  object.object.id = "cube";
  object.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
  attached.robot_state.attached_collision_objects.push_back(object);
  EXPECT_FALSE(manipulation::scene::validate_scene_diff_resources(
      attached, {"attached:cube", "world:cube"}).valid);
  moveit_msgs::msg::CollisionObject world_remove;
  world_remove.id = "cube";
  world_remove.operation = moveit_msgs::msg::CollisionObject::REMOVE;
  attached.world.collision_objects.push_back(world_remove);
  EXPECT_TRUE(manipulation::scene::validate_scene_diff_resources(
      attached, {"attached:cube", "world:cube"}).valid);
}

TEST(SceneDiffValidation, ValidatesSquareSymmetricAcmAndEveryPair)
{
  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;
  scene.allowed_collision_matrix.entry_names = {"link_b", "link_a"};
  moveit_msgs::msg::AllowedCollisionEntry first;
  first.enabled = {true, true};
  moveit_msgs::msg::AllowedCollisionEntry second;
  second.enabled = {true, true};
  scene.allowed_collision_matrix.entry_values = {first, second};

  EXPECT_TRUE(manipulation::scene::validate_scene_diff_resources(
      scene, {"acm:link_a|link_b"}).valid);

  scene.allowed_collision_matrix.entry_values[1].enabled[0] = false;
  EXPECT_FALSE(manipulation::scene::validate_scene_diff_resources(
      scene, {"acm:link_a|link_b"}).valid);

  scene.allowed_collision_matrix.entry_values[1].enabled[0] = true;
  scene.allowed_collision_matrix.default_entry_names = {"link_a"};
  scene.allowed_collision_matrix.default_entry_values = {true};
  EXPECT_FALSE(manipulation::scene::validate_scene_diff_resources(
      scene, {"acm:link_a|link_b"}).valid);
}

TEST(SceneDiffValidation, RejectsGlobalOrNonDiffSceneMutations)
{
  auto scene = world_object_diff();
  scene.is_diff = false;
  EXPECT_FALSE(manipulation::scene::validate_scene_diff_resources(
      scene, {"world:cube"}).valid);

  scene.is_diff = true;
  scene.robot_model_name = "test_robot";
  EXPECT_FALSE(manipulation::scene::validate_scene_diff_resources(
      scene, {"world:cube"}).valid);

  scene.robot_model_name.clear();
  scene.world.octomap.octomap.id = "OcTree";
  EXPECT_FALSE(manipulation::scene::validate_scene_diff_resources(
      scene, {"world:cube"}).valid);
}

TEST(SceneStateModel, EvolvesExplicitWorldAddAndRejectsUnknownRemove)
{
  moveit_msgs::msg::PlanningScene baseline;
  const auto add = world_object_diff();
  const auto transition = manipulation::scene::evolve_authoritative_scene(
    baseline, add, "robot_base");
  ASSERT_TRUE(transition.valid) << transition.message;
  ASSERT_EQ(transition.expected_scene.world.collision_objects.size(), 1U);
  EXPECT_EQ(transition.expected_scene.world.collision_objects.front().id, "cube");
  EXPECT_EQ(
    manipulation::scene::digest_authoritative_scene(transition.expected_scene),
    manipulation::scene::digest_authoritative_scene(transition.backend_diff));

  moveit_msgs::msg::PlanningScene remove;
  remove.is_diff = true;
  moveit_msgs::msg::CollisionObject object;
  object.id = "missing";
  object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
  remove.world.collision_objects.push_back(object);
  EXPECT_FALSE(manipulation::scene::evolve_authoritative_scene(
      baseline, remove, "robot_base").valid);

  auto append = add;
  append.world.collision_objects.front().operation =
    moveit_msgs::msg::CollisionObject::APPEND;
  EXPECT_FALSE(manipulation::scene::validate_scene_diff_resources(
      append, {"world:cube"}).valid);
}

TEST(SceneStateModel, ExpandsPairScopedAcmWithoutDroppingUnrelatedPairs)
{
  moveit_msgs::msg::PlanningScene baseline;
  baseline.allowed_collision_matrix.entry_names = {"a", "b", "c"};
  baseline.allowed_collision_matrix.entry_values.resize(3);
  baseline.allowed_collision_matrix.entry_values[0].enabled = {false, true, false};
  baseline.allowed_collision_matrix.entry_values[1].enabled = {true, false, true};
  baseline.allowed_collision_matrix.entry_values[2].enabled = {false, true, false};

  moveit_msgs::msg::PlanningScene update;
  update.is_diff = true;
  update.allowed_collision_matrix.entry_names = {"c", "a"};
  update.allowed_collision_matrix.entry_values.resize(2);
  update.allowed_collision_matrix.entry_values[0].enabled = {false, true};
  update.allowed_collision_matrix.entry_values[1].enabled = {true, false};

  const auto transition = manipulation::scene::evolve_authoritative_scene(
    baseline, update, "robot_base");
  ASSERT_TRUE(transition.valid) << transition.message;
  const auto & expected = transition.expected_scene.allowed_collision_matrix;
  ASSERT_EQ(expected.entry_names, std::vector<std::string>({"a", "b", "c"}));
  EXPECT_TRUE(expected.entry_values[0].enabled[1]);
  EXPECT_TRUE(expected.entry_values[0].enabled[2]);
  EXPECT_TRUE(expected.entry_values[1].enabled[2]);
  EXPECT_EQ(
    transition.backend_diff.allowed_collision_matrix.entry_names,
    expected.entry_names);
}

TEST(SceneStateModel, ModelsExplicitAttachAndDeterministicDetach)
{
  auto baseline = world_object_diff();
  baseline.is_diff = false;

  moveit_msgs::msg::PlanningScene attach;
  attach.is_diff = true;
  attach.robot_state.is_diff = true;
  moveit_msgs::msg::AttachedCollisionObject attached;
  attached.link_name = "tool_mount";
  attached.object = baseline.world.collision_objects.front();
  attached.object.header.frame_id = attached.link_name;
  attach.robot_state.attached_collision_objects.push_back(attached);

  const auto attached_transition = manipulation::scene::evolve_authoritative_scene(
    baseline, attach, "robot_base");
  ASSERT_TRUE(attached_transition.valid) << attached_transition.message;
  EXPECT_TRUE(attached_transition.expected_scene.world.collision_objects.empty());
  ASSERT_EQ(
    attached_transition.expected_scene.robot_state.attached_collision_objects.size(), 1U);

  moveit_msgs::msg::PlanningScene detach;
  detach.is_diff = true;
  detach.robot_state.is_diff = true;
  moveit_msgs::msg::AttachedCollisionObject remove_attached;
  remove_attached.object.id = "cube";
  remove_attached.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
  detach.robot_state.attached_collision_objects.push_back(remove_attached);
  detach.world.collision_objects.push_back(
    baseline.world.collision_objects.front());

  const auto detached_transition = manipulation::scene::evolve_authoritative_scene(
    attached_transition.expected_scene, detach, "robot_base");
  ASSERT_TRUE(detached_transition.valid) << detached_transition.message;
  EXPECT_TRUE(
    detached_transition.expected_scene.robot_state.attached_collision_objects.empty());
  ASSERT_EQ(detached_transition.expected_scene.world.collision_objects.size(), 1U);
  EXPECT_EQ(detached_transition.expected_scene.world.collision_objects.front().id, "cube");
}

}  // namespace
