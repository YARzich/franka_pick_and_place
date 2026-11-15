// Responsibility: Implements pure V2 scene request, state-bracket, and wait logic.

#include "v2_scene_adapter_support.hpp"

#include <algorithm>
#include <cctype>
#include <exception>
#include <memory>
#include <set>
#include <stdexcept>
#include <string_view>
#include <unordered_set>
#include <utility>

#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_msgs/msg/planning_scene_components.hpp"

#include "manipulation/scene/collision_object_validation.hpp"
#include "manipulation/scene/coordinator_utils.hpp"
#include "manipulation/scene/resource_key.hpp"
#include "manipulation/scene/transfer_object_scene.hpp"
#include "manipulation/validation/frame_id.hpp"
#include "manipulation/validation/rigid_pose.hpp"

namespace manipulation::moveit::detail
{
namespace
{

using ApplyDiff = manipulation_msgs::srv::ApplyPlanningSceneDiff;
using CollisionObject = moveit_msgs::msg::CollisionObject;
using OperationResult = manipulation_msgs::msg::PlanningSceneOperationResult;

bool valid_identifier(const std::string & value)
{
  return !value.empty() && std::all_of(
    value.cbegin(), value.cend(), [](const unsigned char character) {
      return std::isspace(character) == 0 && std::iscntrl(character) == 0;
    });
}

void append_length_prefixed(std::string & output, const std::string & value)
{
  output += std::to_string(value.size());
  output += ':';
  output += value;
  output += ':';
}

std::string_view mutation_token(const V2SceneMutationKind kind)
{
  switch (kind) {
    case V2SceneMutationKind::kStage:
      return "stage";
    case V2SceneMutationKind::kAttach:
      return "attach";
    case V2SceneMutationKind::kDetach:
      return "detach";
  }
  throw std::invalid_argument("unknown V2 scene mutation kind");
}

SceneDiffRequestBuildResult invalid_request(std::string detail)
{
  return {false, nullptr, std::move(detail)};
}

ApplyDiff::Request::SharedPtr base_request(
  const std::string & lease_id,
  const std::string & operation_id,
  const std::uint64_t expected_revision)
{
  if (!valid_identifier(lease_id) || !valid_identifier(operation_id)) {
    return nullptr;
  }
  auto request = std::make_shared<ApplyDiff::Request>();
  request->lease_id = lease_id;
  request->operation_id = operation_id;
  request->expected_revision = expected_revision;
  request->scene_diff.is_diff = true;
  return request;
}

bool valid_touch_links(
  const std::string & attachment_link,
  const std::vector<std::string> & touch_links)
{
  if (!manipulation::validation::is_canonical_relative_frame_id(attachment_link) ||
    touch_links.empty())
  {
    return false;
  }
  std::unordered_set<std::string> unique;
  for (const auto & link : touch_links) {
    if (!manipulation::validation::is_canonical_relative_frame_id(link) ||
      !unique.insert(link).second)
    {
      return false;
    }
  }
  return unique.count(attachment_link) == 1U;
}

bool explicit_object(
  const CollisionObject & object,
  const std::string & expected_id)
{
  return object.id == expected_id &&
         manipulation::scene::validate_explicit_collision_object(object).valid;
}

SceneDiffRequestBuildResult validate_built_request(
  ApplyDiff::Request::SharedPtr request)
{
  const auto validation = manipulation::scene::validate_scene_diff_resources(
    request->scene_diff, request->resource_keys);
  if (!validation.valid) {
    return invalid_request("invalid coordinator scene diff: " + validation.message);
  }
  return {true, std::move(request), {}};
}

}  // namespace

manipulation::workflow::TaskOperationGateResult dispatch_scene_operation(
  manipulation::workflow::TransferExecutionContext & context,
  const SceneDispatchClass dispatch_class,
  const std::function<void()> & operation)
{
  if (!operation) {
    throw std::invalid_argument("scene dispatch operation must not be empty");
  }
  switch (dispatch_class) {
    case SceneDispatchClass::kGoalSideEffect:
      return context.run_operation(operation);
    case SceneDispatchClass::kLeaseMaintenance:
      return context.run_maintenance_operation(operation);
  }
  throw std::invalid_argument("unknown scene dispatch class");
}

void GoalStopGeneration::begin_goal() noexcept
{
  goal_generation_.store(
    stop_generation_.load(std::memory_order_acquire),
    std::memory_order_release);
}

void GoalStopGeneration::request_stop() noexcept
{
  stop_generation_.fetch_add(1U, std::memory_order_acq_rel);
}

std::uint64_t GoalStopGeneration::capture() const noexcept
{
  return goal_generation_.load(std::memory_order_acquire);
}

bool GoalStopGeneration::stopped(const std::uint64_t generation) const noexcept
{
  return stop_generation_.load(std::memory_order_acquire) != generation;
}

InterruptibleWaitResult wait_for_scene_condition(
  const std::function<bool()> & ready,
  const std::function<InterruptibleWaitResult()> & wait_one_slice,
  const std::function<bool()> & ros_context_valid)
{
  if (!ready || !wait_one_slice || !ros_context_valid) {
    return InterruptibleWaitResult::kFailed;
  }
  try {
    while (true) {
      if (!ros_context_valid()) {
        return InterruptibleWaitResult::kShutdown;
      }
      if (ready()) {
        return InterruptibleWaitResult::kReady;
      }
      const auto waited = wait_one_slice();
      if (waited != InterruptibleWaitResult::kReady) {
        return waited;
      }
    }
  } catch (...) {
    return InterruptibleWaitResult::kFailed;
  }
}

std::vector<std::string> make_goal_scene_resource_keys(
  const std::vector<manipulation::workflow::ObjectInstance> & instances,
  const std::vector<std::string> & touch_links)
{
  if (instances.empty()) {
    throw std::invalid_argument("scene acquisition requires at least one object instance");
  }
  std::set<std::string> object_ids;
  std::set<std::string> resources;
  for (const auto & instance : instances) {
    if (!object_ids.insert(instance.object_id()).second) {
      throw std::invalid_argument("scene acquisition object instances must be unique");
    }
    const auto object_resources =
      manipulation::scene::make_transfer_object_scene_resources(
      instance.object_id(), touch_links);
    resources.insert(object_resources.all.begin(), object_resources.all.end());
  }
  return {resources.begin(), resources.end()};
}

moveit_msgs::srv::GetPlanningScene::Request::SharedPtr
make_full_planning_scene_request()
{
  using Components = moveit_msgs::msg::PlanningSceneComponents;
  auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  request->components.components =
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
  return request;
}

StableCoordinatorState validate_coordinator_state(
  const manipulation_msgs::msg::PlanningSceneCoordinatorState & state,
  const manipulation_msgs::msg::PlanningSceneOperationResult & result)
{
  if (!result.success || result.error_code != OperationResult::NONE) {
    return {false, state, result.message.empty() ?
      "coordinator state request failed" : result.message};
  }
  if (state.session_id.empty() || state.revision != result.revision) {
    return {false, state, "coordinator state/result identity is inconsistent"};
  }
  if (!state.backend_service_ready) {
    return {false, state, "coordinator PlanningScene backend is unavailable"};
  }
  if (state.operation_in_progress) {
    return {false, state, "coordinator scene operation is in progress"};
  }
  if (state.reconciliation_required) {
    return {false, state, "coordinator requires PlanningScene reconciliation"};
  }
  if (state.operation_capacity_exhausted) {
    return {false, state, "coordinator idempotency capacity is exhausted"};
  }
  return {true, state, {}};
}

SceneRevisionCheck make_planning_revision_check(
  const PlanningSceneRevision & expected,
  const StableCoordinatorState & observed,
  const bool stopped)
{
  PlanningSceneRevision revision{
    observed.state.session_id, observed.state.revision};
  if (stopped) {
    return {
      SceneRevisionState::kUnavailable, std::move(revision),
      "planning scene revision wait was canceled"};
  }
  if (!observed.valid) {
    return {
      SceneRevisionState::kUnavailable, std::move(revision), observed.detail};
  }
  if (revision == expected) {
    return {
      SceneRevisionState::kCurrent, std::move(revision),
      "scene revision is current"};
  }
  return {
    SceneRevisionState::kRefreshRequired, std::move(revision),
    "authoritative coordinator scene revision changed"};
}

FullSceneBracketCheck check_full_scene_bracket(
  const StableCoordinatorState & before,
  const moveit_msgs::msg::PlanningScene & scene,
  const StableCoordinatorState & after,
  const std::string & expected_session_id,
  const std::uint64_t minimum_revision)
{
  if (!before.valid || !after.valid) {
    return {
      FullSceneBracketStatus::kInvalid,
      !before.valid ? before.detail : after.detail};
  }
  if ((!expected_session_id.empty() &&
    (before.state.session_id != expected_session_id ||
    after.state.session_id != expected_session_id)) ||
    before.state.session_id != after.state.session_id)
  {
    return {
      FullSceneBracketStatus::kConflict,
      "coordinator session changed across full PlanningScene read"};
  }
  if (before.state.revision < minimum_revision ||
    after.state.revision < minimum_revision)
  {
    return {
      FullSceneBracketStatus::kConflict,
      "coordinator revision moved behind required scene evidence"};
  }
  if (before.state.revision != after.state.revision) {
    return {
      FullSceneBracketStatus::kRetry,
      "coordinator revision changed across full PlanningScene read"};
  }
  if (scene.is_diff || scene.robot_state.is_diff) {
    return {
      FullSceneBracketStatus::kInvalid,
      "GetPlanningScene returned a diff instead of a complete snapshot"};
  }
  return {FullSceneBracketStatus::kAccepted, {}};
}

std::string make_v2_scene_operation_id(
  const std::string & lease_id,
  const std::string & object_id,
  const std::uint64_t expected_revision,
  const V2SceneMutationKind kind,
  const std::uint64_t goal_generation,
  const std::uint64_t sequence)
{
  if (!valid_identifier(lease_id) || !valid_identifier(object_id)) {
    throw std::invalid_argument("V2 scene operation identity is malformed");
  }
  std::string result{"transfer/v2:"};
  result.reserve(lease_id.size() + object_id.size() + 96U);
  append_length_prefixed(result, lease_id);
  append_length_prefixed(result, object_id);
  result += std::to_string(expected_revision);
  result += ':';
  result += mutation_token(kind);
  result += ':';
  result += std::to_string(goal_generation);
  result += ':';
  result += std::to_string(sequence);
  return result;
}

SceneDiffRequestBuildResult make_stage_scene_request(
  const std::string & lease_id,
  const std::string & operation_id,
  const std::uint64_t expected_revision,
  const CollisionObject & object)
{
  if (!explicit_object(object, object.id) || object.operation != CollisionObject::ADD) {
    return invalid_request("staged world object must be an explicit ADD value");
  }
  const auto world_key = manipulation::scene::make_world_resource_key(object.id);
  auto request = base_request(lease_id, operation_id, expected_revision);
  if (!world_key || !request) {
    return invalid_request("staged world request identity is malformed");
  }
  request->resource_keys = {*world_key};
  request->scene_diff.world.collision_objects.push_back(object);
  return validate_built_request(std::move(request));
}

SceneDiffRequestBuildResult make_attach_scene_request(
  const std::string & lease_id,
  const std::string & operation_id,
  const std::uint64_t expected_revision,
  const std::string & object_id,
  const std::string & attachment_link,
  const std::vector<std::string> & touch_links,
  const CollisionObject & link_frame_object)
{
  if (!valid_touch_links(attachment_link, touch_links) ||
    !explicit_object(link_frame_object, object_id) ||
    link_frame_object.header.frame_id != attachment_link)
  {
    return invalid_request(
      "attach requires explicit link-frame geometry and canonical unique touch links");
  }
  const auto world_key = manipulation::scene::make_world_resource_key(object_id);
  const auto attached_key = manipulation::scene::make_attached_resource_key(object_id);
  auto request = base_request(lease_id, operation_id, expected_revision);
  if (!world_key || !attached_key || !request) {
    return invalid_request("attach request identity is malformed");
  }
  request->resource_keys = {*attached_key, *world_key};
  std::sort(request->resource_keys.begin(), request->resource_keys.end());
  request->scene_diff.robot_state.is_diff = true;
  moveit_msgs::msg::AttachedCollisionObject attached;
  attached.link_name = attachment_link;
  attached.touch_links = touch_links;
  attached.object = link_frame_object;
  attached.object.operation = CollisionObject::ADD;
  request->scene_diff.robot_state.attached_collision_objects.push_back(
    std::move(attached));
  return validate_built_request(std::move(request));
}

SceneDiffRequestBuildResult make_detach_scene_request(
  const std::string & lease_id,
  const std::string & operation_id,
  const std::uint64_t expected_revision,
  const std::string & object_id,
  const std::string & attachment_link,
  const geometry_msgs::msg::PoseStamped & target_object_pose,
  const CollisionObject & attached_geometry)
{
  if (!manipulation::validation::is_canonical_relative_frame_id(attachment_link) ||
    !manipulation::validation::is_valid_pose_stamped(target_object_pose) ||
    !explicit_object(attached_geometry, object_id))
  {
    return invalid_request("detach requires explicit attached geometry and a valid target pose");
  }
  const auto world_key = manipulation::scene::make_world_resource_key(object_id);
  const auto attached_key = manipulation::scene::make_attached_resource_key(object_id);
  auto request = base_request(lease_id, operation_id, expected_revision);
  if (!world_key || !attached_key || !request) {
    return invalid_request("detach request identity is malformed");
  }
  request->resource_keys = {*attached_key, *world_key};
  std::sort(request->resource_keys.begin(), request->resource_keys.end());
  request->scene_diff.robot_state.is_diff = true;

  moveit_msgs::msg::AttachedCollisionObject removed;
  removed.link_name = attachment_link;
  removed.object.id = object_id;
  removed.object.operation = CollisionObject::REMOVE;
  request->scene_diff.robot_state.attached_collision_objects.push_back(
    std::move(removed));

  auto world_object = attached_geometry;
  world_object.header = target_object_pose.header;
  world_object.pose = target_object_pose.pose;
  world_object.operation = CollisionObject::ADD;
  request->scene_diff.world.collision_objects.push_back(std::move(world_object));
  return validate_built_request(std::move(request));
}

const CollisionObject * find_world_object(
  const moveit_msgs::msg::PlanningScene & scene,
  const std::string & object_id) noexcept
{
  const auto found = std::find_if(
    scene.world.collision_objects.cbegin(),
    scene.world.collision_objects.cend(),
    [&object_id](const auto & object) {return object.id == object_id;});
  return found == scene.world.collision_objects.cend() ? nullptr : &*found;
}

const moveit_msgs::msg::AttachedCollisionObject * find_attached_object(
  const moveit_msgs::msg::PlanningScene & scene,
  const std::string & object_id) noexcept
{
  const auto & objects = scene.robot_state.attached_collision_objects;
  const auto found = std::find_if(
    objects.cbegin(), objects.cend(),
    [&object_id](const auto & object) {return object.object.id == object_id;});
  return found == objects.cend() ? nullptr : &*found;
}

}  // namespace manipulation::moveit::detail
