// Responsibility: Implements catalog staging and coordinator-only attach/detach mutations.

#include "ros_v2_scene_adapter_impl.hpp"

#include <exception>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "Eigen/Geometry"
#include "tf2_eigen/tf2_eigen.hpp"

#include "manipulation/workflow/object_geometry_conversion.hpp"
#include "manipulation_msgs/msg/planning_scene_operation_result.hpp"

namespace manipulation::moveit
{
namespace
{

using ErrorCode = manipulation::task::TaskErrorCode;
using OperationResult = manipulation_msgs::msg::PlanningSceneOperationResult;

bool response_may_have_applied(const OperationResult & result) noexcept
{
  return !result.success &&
         (result.error_code == OperationResult::BACKEND_FAILED ||
         result.error_code == OperationResult::TIMEOUT ||
         result.error_code == OperationResult::RECONCILIATION_REQUIRED ||
         result.error_code == OperationResult::OPERATION_IN_PROGRESS ||
         result.error_code == OperationResult::INTERNAL_ERROR);
}

}  // namespace

RosV2SceneAdapter::Impl::MutationResult RosV2SceneAdapter::Impl::apply_mutation(
  Apply::Request::SharedPtr request,
  manipulation::workflow::TransferExecutionContext & context,
  const std::uint64_t generation)
{
  if (!request) {
    return {{}, FailureKind::kInternal, "scene mutation request is null"};
  }
  const auto expected_revision = request->expected_revision;
  const auto call = call_service<Apply>(
    *apply_client_, std::move(request), context_control(context, generation),
    "waiting for coordinator PlanningScene mutation result");
  if (call.status != ServiceStatus::kResponse) {
    if (call.request_sent) {
      return {
        {}, FailureKind::kStateUncertain,
        call.detail + "; dispatched scene mutation outcome is unknown"};
    }
    const auto failure = call.status == ServiceStatus::kStopped ?
      FailureKind::kCanceled :
      call.status == ServiceStatus::kShutdown ?
      FailureKind::kShutdown : FailureKind::kInternal;
    return {{}, failure, call.detail};
  }
  const auto & result = call.response->result;
  if (!result.success) {
    return {
      {}, response_may_have_applied(result) ?
      FailureKind::kStateUncertain : coordinator_failure(result),
      result.message};
  }
  if (result.error_code != OperationResult::NONE ||
    result.revision <= expected_revision)
  {
    return {
      {}, FailureKind::kStateUncertain,
      "coordinator mutation success carried contradictory revision evidence"};
  }
  return {result.revision, FailureKind::kInternal, {}};
}

moveit_msgs::msg::CollisionObject RosV2SceneAdapter::Impl::make_link_frame_object(
  const manipulation::workflow::PlanningSceneSnapshot & snapshot,
  const std::string & object_id,
  const std::string & attachment_link) const
{
  const auto * source = detail::find_world_object(snapshot.scene(), object_id);
  if (!source) {
    throw std::invalid_argument("full scene has no world object to attach");
  }

  ::planning_scene::PlanningScene local_scene(robot_model_);
  if (!local_scene.setPlanningSceneMsg(snapshot.scene())) {
    throw std::runtime_error("MoveIt rejected the immutable full scene snapshot");
  }
  if (!local_scene.knowsFrameTransform(source->header.frame_id) ||
    !local_scene.knowsFrameTransform(attachment_link))
  {
    throw std::invalid_argument(
            "full scene cannot transform staged geometry into the attachment link");
  }

  Eigen::Isometry3d header_to_object;
  tf2::fromMsg(source->pose, header_to_object);
  const Eigen::Isometry3d model_to_object =
    local_scene.getFrameTransform(source->header.frame_id) * header_to_object;
  const Eigen::Isometry3d link_to_object =
    local_scene.getFrameTransform(attachment_link).inverse() * model_to_object;
  if (!link_to_object.matrix().allFinite()) {
    throw std::runtime_error("attachment-link object transform is non-finite");
  }

  auto result = *source;
  result.header.frame_id = attachment_link;
  result.pose = tf2::toMsg(link_to_object);
  result.operation = moveit_msgs::msg::CollisionObject::ADD;
  return result;
}

manipulation::workflow::PlanningSceneLeaseResult
RosV2SceneAdapter::Impl::stage_observation(
  const manipulation::workflow::ObjectInstance & instance,
  const manipulation::workflow::ObjectProfile & profile,
  const manipulation::workflow::ObjectObservation & observation,
  const manipulation::workflow::PlanningSceneAccessProof & proof,
  manipulation::workflow::TransferExecutionContext & context)
{
  if (observation.object_id() != instance.object_id()) {
    return manipulation::workflow::PlanningSceneLeaseResult::failure(
      ErrorCode::kInvalidGoal,
      "catalog instance and stamped observation identify different objects");
  }
  const auto generation = stop_state_.capture();
  std::string detail_text;
  const auto active = active_lease_for(proof, detail_text);
  if (!active) {
    return manipulation::workflow::PlanningSceneLeaseResult::failure(
      ErrorCode::kSceneConflict, std::move(detail_text));
  }

  moveit_msgs::msg::CollisionObject object;
  try {
    object = manipulation::workflow::convert_object_geometry(
      instance, profile, observation.pose(), mesh_loader_);
  } catch (const std::exception & error) {
    return manipulation::workflow::PlanningSceneLeaseResult::failure(
      ErrorCode::kInvalidGoal,
      std::string{"catalog geometry conversion failed: "} + error.what());
  }

  const std::lock_guard<std::mutex> mutation_lock(mutation_mutex_);
  const auto exact = validate_exact_proof(proof, *active, context, generation);
  if (!exact.valid) {
    return manipulation::workflow::PlanningSceneLeaseResult::failure(
      task_error(exact.failure), std::move(exact.detail));
  }

  detail::SceneDiffRequestBuildResult built;
  try {
    built = detail::make_stage_scene_request(
      proof.lease_id(),
      detail::make_v2_scene_operation_id(
        proof.lease_id(), instance.object_id(), proof.revision(),
        detail::V2SceneMutationKind::kStage, generation,
        operation_sequence_.fetch_add(1U, std::memory_order_acq_rel) + 1U),
      proof.revision(), object);
  } catch (const std::exception & error) {
    return manipulation::workflow::PlanningSceneLeaseResult::failure(
      ErrorCode::kInternalError, error.what());
  }
  if (!built.valid) {
    return manipulation::workflow::PlanningSceneLeaseResult::failure(
      ErrorCode::kInvalidGoal, std::move(built.detail));
  }
  const auto mutated = apply_mutation(std::move(built.request), context, generation);
  if (!mutated.succeeded()) {
    return manipulation::workflow::PlanningSceneLeaseResult::failure(
      task_error(mutated.failure), std::move(mutated.detail));
  }
  auto full = read_full_scene(
    *active, proof.coordinator_session_id(), *mutated.revision, context, generation);
  if (!full.succeeded()) {
    return manipulation::workflow::PlanningSceneLeaseResult::failure(
      task_error(full.failure), std::move(full.detail));
  }
  return manipulation::workflow::PlanningSceneLeaseResult::success(
    std::move(*full.lease));
}

CoordinatedSceneResult RosV2SceneAdapter::Impl::attach(
  const StrategySceneAttachRequest & request,
  manipulation::workflow::TransferExecutionContext & context)
{
  if (request.scene_snapshot.coordinator_session_id() !=
    request.access_proof.coordinator_session_id() ||
    request.scene_snapshot.revision() != request.access_proof.revision() ||
    request.scene_snapshot.scene().is_diff ||
    request.scene_snapshot.scene().robot_state.is_diff)
  {
    return CoordinatedSceneResult::failure(
      CoordinatedSceneOutcome::kConflict,
      "attach requires matching complete snapshot and exact proof");
  }
  if (request.attachment_link != config_.attachment_link ||
    request.touch_links != config_.touch_links)
  {
    return CoordinatedSceneResult::failure(
      CoordinatedSceneOutcome::kFailed,
      "attach request differs from leased robot link resources");
  }
  const auto generation = stop_state_.capture();
  std::string detail_text;
  const auto active = active_lease_for(request.access_proof, detail_text);
  if (!active) {
    return CoordinatedSceneResult::failure(
      CoordinatedSceneOutcome::kConflict, std::move(detail_text));
  }

  moveit_msgs::msg::CollisionObject link_object;
  try {
    link_object = make_link_frame_object(
      request.scene_snapshot, request.object_id, request.attachment_link);
  } catch (const std::exception & error) {
    return CoordinatedSceneResult::failure(
      CoordinatedSceneOutcome::kFailed,
      std::string{"attach geometry preparation failed: "} + error.what());
  }

  const std::lock_guard<std::mutex> mutation_lock(mutation_mutex_);
  const auto exact = validate_exact_proof(
    request.access_proof, *active, context, generation);
  if (!exact.valid) {
    return CoordinatedSceneResult::failure(
      coordinated_outcome(exact.failure), std::move(exact.detail));
  }
  detail::SceneDiffRequestBuildResult built;
  try {
    built = detail::make_attach_scene_request(
      request.access_proof.lease_id(),
      detail::make_v2_scene_operation_id(
        request.access_proof.lease_id(), request.object_id,
        request.access_proof.revision(), detail::V2SceneMutationKind::kAttach,
        generation,
        operation_sequence_.fetch_add(1U, std::memory_order_acq_rel) + 1U),
      request.access_proof.revision(), request.object_id,
      request.attachment_link, request.touch_links, link_object);
  } catch (const std::exception & error) {
    return CoordinatedSceneResult::failure(
      CoordinatedSceneOutcome::kInternalError, error.what());
  }
  if (!built.valid) {
    return CoordinatedSceneResult::failure(
      CoordinatedSceneOutcome::kFailed, std::move(built.detail));
  }
  const auto mutated = apply_mutation(std::move(built.request), context, generation);
  if (!mutated.succeeded()) {
    return CoordinatedSceneResult::failure(
      coordinated_outcome(mutated.failure), std::move(mutated.detail));
  }
  auto full = read_full_scene(
    *active, request.access_proof.coordinator_session_id(),
    *mutated.revision, context, generation);
  if (!full.succeeded()) {
    return CoordinatedSceneResult::failure(
      coordinated_outcome(full.failure), std::move(full.detail));
  }
  return CoordinatedSceneResult::success(
    std::move(*full.lease), "object attached through scene coordinator");
}

CoordinatedSceneResult RosV2SceneAdapter::Impl::detach(
  const StrategySceneDetachRequest & request,
  manipulation::workflow::TransferExecutionContext & context)
{
  if (request.scene_snapshot.coordinator_session_id() !=
    request.access_proof.coordinator_session_id() ||
    request.scene_snapshot.revision() != request.access_proof.revision() ||
    request.scene_snapshot.scene().is_diff ||
    request.scene_snapshot.scene().robot_state.is_diff ||
    request.attachment_link != config_.attachment_link)
  {
    return CoordinatedSceneResult::failure(
      CoordinatedSceneOutcome::kConflict,
      "detach requires matching complete scene evidence and leased attachment link");
  }
  const auto * attached = detail::find_attached_object(
    request.scene_snapshot.scene(), request.object_id);
  if (!attached || attached->link_name != request.attachment_link) {
    return CoordinatedSceneResult::failure(
      CoordinatedSceneOutcome::kFailed,
      "full scene has no matching attached object to detach");
  }

  const auto generation = stop_state_.capture();
  std::string detail_text;
  const auto active = active_lease_for(request.access_proof, detail_text);
  if (!active) {
    return CoordinatedSceneResult::failure(
      CoordinatedSceneOutcome::kConflict, std::move(detail_text));
  }
  const std::lock_guard<std::mutex> mutation_lock(mutation_mutex_);
  const auto exact = validate_exact_proof(
    request.access_proof, *active, context, generation);
  if (!exact.valid) {
    return CoordinatedSceneResult::failure(
      coordinated_outcome(exact.failure), std::move(exact.detail));
  }

  detail::SceneDiffRequestBuildResult built;
  try {
    built = detail::make_detach_scene_request(
      request.access_proof.lease_id(),
      detail::make_v2_scene_operation_id(
        request.access_proof.lease_id(), request.object_id,
        request.access_proof.revision(), detail::V2SceneMutationKind::kDetach,
        generation,
        operation_sequence_.fetch_add(1U, std::memory_order_acq_rel) + 1U),
      request.access_proof.revision(), request.object_id,
      request.attachment_link, request.target_object_pose, attached->object);
  } catch (const std::exception & error) {
    return CoordinatedSceneResult::failure(
      CoordinatedSceneOutcome::kInternalError, error.what());
  }
  if (!built.valid) {
    return CoordinatedSceneResult::failure(
      CoordinatedSceneOutcome::kFailed, std::move(built.detail));
  }
  const auto mutated = apply_mutation(std::move(built.request), context, generation);
  if (!mutated.succeeded()) {
    return CoordinatedSceneResult::failure(
      coordinated_outcome(mutated.failure), std::move(mutated.detail));
  }
  auto full = read_full_scene(
    *active, request.access_proof.coordinator_session_id(),
    *mutated.revision, context, generation);
  if (!full.succeeded()) {
    return CoordinatedSceneResult::failure(
      coordinated_outcome(full.failure), std::move(full.detail));
  }
  return CoordinatedSceneResult::success(
    std::move(*full.lease), "object detached through scene coordinator");
}

}  // namespace manipulation::moveit
