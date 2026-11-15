// Responsibility: Verifies pure V2 scene resource, request, bracket, wait, and revision logic.

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <set>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_msgs/msg/planning_scene_components.hpp"
#include "shape_msgs/msg/mesh_triangle.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"

#include "manipulation/workflow/object_catalog.hpp"
#include "manipulation/workflow/object_geometry_conversion.hpp"

#include "v2_scene_adapter_support.hpp"

namespace manipulation::moveit::detail
{
namespace
{

using CollisionObject = moveit_msgs::msg::CollisionObject;
using Components = moveit_msgs::msg::PlanningSceneComponents;
using Instance = manipulation::workflow::ObjectInstance;
using OperationResult = manipulation_msgs::msg::PlanningSceneOperationResult;
using GateResult = manipulation::workflow::TaskOperationGateResult;

class SceneDispatchTestContext final
  : public manipulation::workflow::TransferExecutionContext
{
public:
  explicit SceneDispatchTestContext(
    const GateResult goal_result,
    const GateResult maintenance_result = GateResult::kExecuted)
  : goal_result_(goal_result), maintenance_result_(maintenance_result)
  {
  }

  manipulation::workflow::TransferExecutionControlResult checkpoint() override
  {
    return manipulation::workflow::TransferExecutionControlResult::kContinue;
  }

  [[nodiscard]] bool stop_requested() const noexcept override
  {
    return false;
  }

  manipulation::workflow::TransferExecutionControlResult wait_until(TimePoint) override
  {
    return manipulation::workflow::TransferExecutionControlResult::kContinue;
  }

  GateResult run_operation(const std::function<void()> & operation) override
  {
    ++goal_gate_calls;
    if (goal_result_ == GateResult::kExecuted) {
      operation();
    }
    return goal_result_;
  }

  GateResult run_maintenance_operation(
    const std::function<void()> & operation) override
  {
    ++maintenance_gate_calls;
    if (maintenance_result_ == GateResult::kExecuted) {
      operation();
    }
    return maintenance_result_;
  }

  void report_feedback(manipulation::task::TaskStateSnapshot) override {}

  std::size_t goal_gate_calls{0U};
  std::size_t maintenance_gate_calls{0U};

private:
  GateResult goal_result_;
  GateResult maintenance_result_;
};

class RecordingMeshLoader final
  : public manipulation::workflow::MeshResourceLoader
{
public:
  shape_msgs::msg::Mesh load(
    const std::string_view resource_uri,
    const std::array<double, 3> & scale) const override
  {
    ++calls;
    last_uri = std::string{resource_uri};
    last_scale = scale;
    shape_msgs::msg::Mesh mesh;
    mesh.vertices.resize(3U);
    mesh.vertices[1].x = 1.0;
    mesh.vertices[2].y = 1.0;
    shape_msgs::msg::MeshTriangle triangle;
    triangle.vertex_indices = {0U, 1U, 2U};
    mesh.triangles.push_back(triangle);
    return mesh;
  }

  mutable std::size_t calls{0U};
  mutable std::string last_uri;
  mutable std::array<double, 3> last_scale{};
};

geometry_msgs::msg::Pose identity_pose()
{
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  return pose;
}

CollisionObject compound_object(
  const std::string & id = "part",
  const std::string & frame = "world")
{
  CollisionObject object;
  object.id = id;
  object.header.frame_id = frame;
  object.header.stamp.sec = 1;
  object.pose = identity_pose();
  object.operation = CollisionObject::ADD;

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions = {0.1, 0.2, 0.3};
  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(identity_pose());

  shape_msgs::msg::Mesh mesh;
  geometry_msgs::msg::Point first;
  geometry_msgs::msg::Point second;
  geometry_msgs::msg::Point third;
  second.x = 1.0;
  third.y = 1.0;
  mesh.vertices = {first, second, third};
  shape_msgs::msg::MeshTriangle triangle;
  triangle.vertex_indices = {0U, 1U, 2U};
  mesh.triangles.push_back(triangle);
  object.meshes.push_back(mesh);
  object.mesh_poses.push_back(identity_pose());
  return object;
}

StableCoordinatorState stable_state(
  const std::string & session = "session-a",
  const std::uint64_t revision = 7U)
{
  manipulation_msgs::msg::PlanningSceneCoordinatorState state;
  state.session_id = session;
  state.revision = revision;
  state.backend_service_ready = true;
  OperationResult result;
  result.success = true;
  result.error_code = OperationResult::NONE;
  result.revision = revision;
  return validate_coordinator_state(state, result);
}

TEST(V2SceneAdapterSupport, AcquiresEveryObjectAndRobotContactResource)
{
  const std::vector<Instance> instances{
    Instance::create("part-a", "box"),
    Instance::create("part-b", "mesh")};
  const auto keys = make_goal_scene_resource_keys(
    instances, {"panda_hand", "panda_leftfinger"});
  const std::set<std::string> actual(keys.cbegin(), keys.cend());

  EXPECT_EQ(actual.size(), 8U);
  EXPECT_EQ(actual.count("world:part-a"), 1U);
  EXPECT_EQ(actual.count("attached:part-a"), 1U);
  EXPECT_EQ(actual.count("acm:panda_hand|part-a"), 1U);
  EXPECT_EQ(actual.count("acm:panda_leftfinger|part-a"), 1U);
  EXPECT_EQ(actual.count("world:part-b"), 1U);
  EXPECT_EQ(actual.count("attached:part-b"), 1U);
  EXPECT_TRUE(std::is_sorted(keys.cbegin(), keys.cend()));
}

TEST(V2SceneAdapterSupport, RejectsDuplicateGoalInstances)
{
  const std::vector<Instance> duplicate{
    Instance::create("part", "box"),
    Instance::create("part", "box")};
  EXPECT_THROW(
    make_goal_scene_resource_keys(duplicate, {"panda_hand"}),
    std::invalid_argument);
}

TEST(V2SceneAdapterSupport, RequestsEveryFullPlanningSceneComponent)
{
  const auto request = make_full_planning_scene_request();
  const std::uint32_t expected =
    Components::SCENE_SETTINGS |
    Components::ROBOT_STATE |
    Components::ROBOT_STATE_ATTACHED_OBJECTS |
    Components::WORLD_OBJECT_NAMES |
    Components::WORLD_OBJECT_GEOMETRY |
    Components::OCTOMAP |
    Components::TRANSFORMS |
    Components::ALLOWED_COLLISION_MATRIX |
    Components::LINK_PADDING_AND_SCALING |
    Components::OBJECT_COLORS;
  ASSERT_NE(request, nullptr);
  EXPECT_EQ(request->components.components, expected);
}

TEST(V2SceneAdapterSupport, AcceptsOnlyStableCompleteSceneBracket)
{
  moveit_msgs::msg::PlanningScene scene;
  auto before = stable_state();
  auto after = stable_state();
  EXPECT_EQ(
    check_full_scene_bracket(before, scene, after, "session-a", 7U).status,
    FullSceneBracketStatus::kAccepted);

  scene.robot_state.is_diff = true;
  EXPECT_EQ(
    check_full_scene_bracket(before, scene, after, "session-a", 7U).status,
    FullSceneBracketStatus::kInvalid);
  scene.robot_state.is_diff = false;
  scene.is_diff = true;
  EXPECT_EQ(
    check_full_scene_bracket(before, scene, after, "session-a", 7U).status,
    FullSceneBracketStatus::kInvalid);
}

TEST(V2SceneAdapterSupport, RetriesRevisionRaceAndRejectsSessionReplacement)
{
  moveit_msgs::msg::PlanningScene scene;
  EXPECT_EQ(
    check_full_scene_bracket(
      stable_state("session-a", 7U), scene,
      stable_state("session-a", 8U), "session-a", 7U).status,
    FullSceneBracketStatus::kRetry);
  EXPECT_EQ(
    check_full_scene_bracket(
      stable_state("session-a", 7U), scene,
      stable_state("session-b", 7U), "session-a", 7U).status,
    FullSceneBracketStatus::kConflict);
}

TEST(V2SceneAdapterSupport, BuildsCatalogCompoundStageRequestThroughCoordinator)
{
  const auto built = make_stage_scene_request(
    "lease-a", "operation-a", 7U, compound_object());
  ASSERT_TRUE(built.valid) << built.detail;
  ASSERT_NE(built.request, nullptr);
  EXPECT_EQ(built.request->lease_id, "lease-a");
  EXPECT_EQ(built.request->expected_revision, 7U);
  EXPECT_EQ(built.request->resource_keys, std::vector<std::string>{"world:part"});
  ASSERT_EQ(built.request->scene_diff.world.collision_objects.size(), 1U);
  const auto & staged = built.request->scene_diff.world.collision_objects.front();
  EXPECT_EQ(staged.primitives.size(), 1U);
  EXPECT_EQ(staged.meshes.size(), 1U);
}

TEST(V2SceneAdapterSupport, ConvertsCatalogPrimitiveAndMeshPartsBeforeStaging)
{
  using manipulation::workflow::MeshGeometry;
  using manipulation::workflow::ObjectGeometry;
  using manipulation::workflow::ObjectGeometryPart;
  using manipulation::workflow::ObjectProfile;
  using manipulation::workflow::PrimitiveGeometry;
  using manipulation::workflow::PrimitiveGeometryType;

  const auto instance = Instance::create("catalog-part", "compound-profile");
  std::vector<ObjectGeometryPart> parts;
  parts.emplace_back(PrimitiveGeometry::create(
      PrimitiveGeometryType::kBox, {0.1, 0.2, 0.3}, identity_pose()));
  parts.emplace_back(MeshGeometry::create(
      "package://example/part.stl", {2.0, 3.0, 4.0}, identity_pose()));
  const auto profile = ObjectProfile::create(
    "compound-profile", ObjectGeometry::compound(std::move(parts)));
  geometry_msgs::msg::PoseStamped observation;
  observation.header.frame_id = "world";
  observation.header.stamp.sec = 1;
  observation.pose = identity_pose();
  RecordingMeshLoader loader;

  const auto converted = manipulation::workflow::convert_object_geometry(
    instance, profile, observation, loader);
  EXPECT_EQ(converted.id, "catalog-part");
  EXPECT_EQ(converted.primitives.size(), 1U);
  EXPECT_EQ(converted.meshes.size(), 1U);
  EXPECT_EQ(loader.calls, 1U);
  EXPECT_EQ(loader.last_uri, "package://example/part.stl");
  EXPECT_EQ(loader.last_scale, (std::array<double, 3>{2.0, 3.0, 4.0}));

  const auto built = make_stage_scene_request(
    "lease-a", "catalog-stage", 9U, converted);
  ASSERT_TRUE(built.valid) << built.detail;
  EXPECT_EQ(
    built.request->scene_diff.world.collision_objects.front().meshes.size(), 1U);
}

TEST(V2SceneAdapterSupport, BuildsExactAttachAndDetachCoordinatorDiffs)
{
  auto link_object = compound_object("part", "panda_hand");
  const auto attached = make_attach_scene_request(
    "lease-a", "attach-a", 7U, "part", "panda_hand",
    {"panda_hand", "panda_leftfinger"}, link_object);
  ASSERT_TRUE(attached.valid) << attached.detail;
  EXPECT_EQ(
    attached.request->resource_keys,
    (std::vector<std::string>{"attached:part", "world:part"}));
  ASSERT_EQ(
    attached.request->scene_diff.robot_state.attached_collision_objects.size(), 1U);
  EXPECT_EQ(
    attached.request->scene_diff.robot_state.attached_collision_objects.front().touch_links.size(),
    2U);

  geometry_msgs::msg::PoseStamped target;
  target.header.frame_id = "world";
  target.header.stamp.sec = 2;
  target.pose = identity_pose();
  const auto detached = make_detach_scene_request(
    "lease-a", "detach-a", 8U, "part", "panda_hand", target, link_object);
  ASSERT_TRUE(detached.valid) << detached.detail;
  ASSERT_EQ(detached.request->scene_diff.world.collision_objects.size(), 1U);
  EXPECT_EQ(
    detached.request->scene_diff.world.collision_objects.front().header.frame_id,
    "world");
  EXPECT_EQ(
    detached.request->scene_diff.robot_state.attached_collision_objects.front().object.operation,
    CollisionObject::REMOVE);
}

TEST(V2SceneAdapterSupport, OperationIdsAreV2AndDelimiterSafe)
{
  const auto first = make_v2_scene_operation_id(
    "lease:a", "part", 7U, V2SceneMutationKind::kStage, 1U, 1U);
  const auto second = make_v2_scene_operation_id(
    "lease", "a:part", 7U, V2SceneMutationKind::kStage, 1U, 1U);
  EXPECT_EQ(first.find("transfer/v2:"), 0U);
  EXPECT_NE(first, second);
}

TEST(V2SceneAdapterSupport, NonPhysicalWaitHasNoElapsedTerminalLimit)
{
  std::size_t probes = 0U;
  const auto result = wait_for_scene_condition(
    [&probes]() {return ++probes > 1000U;},
    []() {return InterruptibleWaitResult::kReady;},
    []() {return true;});
  EXPECT_EQ(result, InterruptibleWaitResult::kReady);
  EXPECT_GT(probes, 1000U);
}

TEST(V2SceneAdapterSupport, ShutdownWinsWhenEndpointIsAlreadyReady)
{
  const auto result = wait_for_scene_condition(
    []() {return true;},
    []() {return InterruptibleWaitResult::kReady;},
    []() {return false;});

  EXPECT_EQ(result, InterruptibleWaitResult::kShutdown);
}

TEST(V2SceneAdapterSupport, StoppedOrPausedSceneDispatchHasNoSideEffect)
{
  for (const auto blocked : {GateResult::kStopped, GateResult::kPaused}) {
    SceneDispatchTestContext context(blocked);
    std::size_t side_effects = 0U;

    const auto result = dispatch_scene_operation(
      context, SceneDispatchClass::kGoalSideEffect,
      [&side_effects]() {++side_effects;});

    EXPECT_EQ(result, blocked);
    EXPECT_EQ(side_effects, 0U);
    EXPECT_EQ(context.goal_gate_calls, 1U);
    EXPECT_EQ(context.maintenance_gate_calls, 0U);
  }
}

TEST(V2SceneAdapterSupport, LeaseMaintenanceUsesStopOnlyGateWhileGoalIsPaused)
{
  SceneDispatchTestContext context(GateResult::kPaused);
  std::size_t renew_dispatches = 0U;

  const auto result = dispatch_scene_operation(
    context, SceneDispatchClass::kLeaseMaintenance,
    [&renew_dispatches]() {++renew_dispatches;});

  EXPECT_EQ(result, GateResult::kExecuted);
  EXPECT_EQ(renew_dispatches, 1U);
  EXPECT_EQ(context.goal_gate_calls, 0U);
  EXPECT_EQ(context.maintenance_gate_calls, 1U);
}

TEST(V2SceneAdapterSupport, CancelInterruptsAnOtherwiseUnboundedWait)
{
  std::size_t slices = 0U;
  const auto result = wait_for_scene_condition(
    []() {return false;},
    [&slices]() {
      ++slices;
      return slices == 5U ? InterruptibleWaitResult::kStopped :
             InterruptibleWaitResult::kReady;
    },
    []() {return true;});
  EXPECT_EQ(result, InterruptibleWaitResult::kStopped);
  EXPECT_EQ(slices, 5U);
}

TEST(V2SceneAdapterSupport, PreviousGoalCancelDoesNotPoisonNextAcquireGeneration)
{
  GoalStopGeneration generation;
  generation.begin_goal();
  const auto previous = generation.capture();
  generation.request_stop();
  EXPECT_TRUE(generation.stopped(previous));

  generation.begin_goal();
  const auto next = generation.capture();
  EXPECT_FALSE(generation.stopped(next));
}

TEST(V2SceneAdapterSupport, MapsPlanningRevisionCurrentRefreshUnavailableAndCancel)
{
  const PlanningSceneRevision expected{"session-a", 7U};
  EXPECT_EQ(
    make_planning_revision_check(expected, stable_state(), false).state,
    SceneRevisionState::kCurrent);
  EXPECT_EQ(
    make_planning_revision_check(
      expected, stable_state("session-a", 8U), false).state,
    SceneRevisionState::kRefreshRequired);

  auto unavailable = stable_state();
  unavailable.valid = false;
  unavailable.detail = "backend unavailable";
  EXPECT_EQ(
    make_planning_revision_check(expected, unavailable, false).state,
    SceneRevisionState::kUnavailable);
  EXPECT_EQ(
    make_planning_revision_check(expected, stable_state(), true).state,
    SceneRevisionState::kUnavailable);
}

}  // namespace
}  // namespace manipulation::moveit::detail
