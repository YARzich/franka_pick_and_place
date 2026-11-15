// Responsibility: Validates workflow evidence and preserves observation/scene correlations.

#include "manipulation/workflow/transfer_workflow_types.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "manipulation/validation/frame_id.hpp"
#include "manipulation/validation/rigid_pose.hpp"

namespace manipulation::workflow
{
namespace
{

constexpr std::size_t kMaximumIdentityLength = 256U;

bool is_visible_token(const std::string_view value)
{
  return !value.empty() && value.size() <= kMaximumIdentityLength &&
         value.find('|') == std::string_view::npos &&
         std::all_of(value.cbegin(), value.cend(), [](const char character) {
             const auto byte = static_cast<unsigned char>(character);
             return byte >= 0x21U && byte <= 0x7eU;
         });
}

void require_token(const std::string_view value, const char * field)
{
  if (!is_visible_token(value)) {
    throw std::invalid_argument(std::string{field} + " must be a bounded visible token");
  }
}

bool same_stamp(
  const builtin_interfaces::msg::Time & left,
  const builtin_interfaces::msg::Time & right) noexcept
{
  return left.sec == right.sec && left.nanosec == right.nanosec;
}

bool valid_recovery_policy(const TransferRecoveryPolicy policy) noexcept
{
  switch (policy) {
    case TransferRecoveryPolicy::kNone:
    case TransferRecoveryPolicy::kSafeStop:
    case TransferRecoveryPolicy::kRetreat:
    case TransferRecoveryPolicy::kRetreatAndHome:
      return true;
  }
  return false;
}

}  // namespace

ObservationCorrelation ObservationCorrelation::create(
  std::string object_id,
  std::string frame_id,
  builtin_interfaces::msg::Time stamp,
  std::string correlation_id)
{
  require_token(object_id, "observation object ID");
  if (!manipulation::validation::is_canonical_relative_frame_id(frame_id)) {
    throw std::invalid_argument("observation frame must be a canonical relative frame ID");
  }
  if (!manipulation::validation::is_normalized_nonzero_stamp(stamp)) {
    throw std::invalid_argument("observation correlation requires a normalized non-zero stamp");
  }
  require_token(correlation_id, "observation correlation ID");
  return ObservationCorrelation(
    std::move(object_id), std::move(frame_id), stamp, std::move(correlation_id));
}

ObservationCorrelation::ObservationCorrelation(
  std::string object_id,
  std::string frame_id,
  const builtin_interfaces::msg::Time stamp,
  std::string correlation_id)
: object_id_(std::move(object_id)), frame_id_(std::move(frame_id)), stamp_(stamp),
  correlation_id_(std::move(correlation_id))
{
}

const std::string & ObservationCorrelation::object_id() const noexcept
{
  return object_id_;
}

const std::string & ObservationCorrelation::frame_id() const noexcept
{
  return frame_id_;
}

const builtin_interfaces::msg::Time & ObservationCorrelation::stamp() const noexcept
{
  return stamp_;
}

const std::string & ObservationCorrelation::correlation_id() const noexcept
{
  return correlation_id_;
}

bool ObservationCorrelation::operator==(const ObservationCorrelation & other) const noexcept
{
  return object_id_ == other.object_id_ && frame_id_ == other.frame_id_ &&
         same_stamp(stamp_, other.stamp_) && correlation_id_ == other.correlation_id_;
}

bool ObservationCorrelation::operator!=(const ObservationCorrelation & other) const noexcept
{
  return !(*this == other);
}

ObjectObservation ObjectObservation::create(
  std::string object_id,
  geometry_msgs::msg::PoseStamped pose,
  std::string correlation_id,
  const std::optional<double> confidence)
{
  if (!manipulation::validation::is_valid_pose_stamped(pose)) {
    throw std::invalid_argument("object observation requires a valid stamped rigid pose");
  }
  if (confidence &&
    (!std::isfinite(*confidence) || *confidence < 0.0 || *confidence > 1.0))
  {
    throw std::invalid_argument("object observation confidence must be in [0, 1]");
  }
  auto correlation = ObservationCorrelation::create(
    std::move(object_id), pose.header.frame_id, pose.header.stamp,
    std::move(correlation_id));
  return ObjectObservation(std::move(pose), std::move(correlation), confidence);
}

ObjectObservation::ObjectObservation(
  geometry_msgs::msg::PoseStamped pose,
  ObservationCorrelation correlation,
  const std::optional<double> confidence)
: pose_(std::move(pose)), correlation_(std::move(correlation)), confidence_(confidence)
{
}

const std::string & ObjectObservation::object_id() const noexcept
{
  return correlation_.object_id();
}

const geometry_msgs::msg::PoseStamped & ObjectObservation::pose() const noexcept
{
  return pose_;
}

const ObservationCorrelation & ObjectObservation::correlation() const noexcept
{
  return correlation_;
}

const std::optional<double> & ObjectObservation::confidence() const noexcept
{
  return confidence_;
}

PlanningSceneSnapshot PlanningSceneSnapshot::create(
  std::string coordinator_session_id,
  const std::uint64_t revision,
  moveit_msgs::msg::PlanningScene scene)
{
  require_token(coordinator_session_id, "PlanningScene coordinator session ID");
  if (scene.is_diff || scene.robot_state.is_diff) {
    throw std::invalid_argument(
            "PlanningScene snapshot and robot state must both be complete values, not diffs");
  }
  return PlanningSceneSnapshot(
    std::move(coordinator_session_id), revision, std::move(scene));
}

PlanningSceneSnapshot::PlanningSceneSnapshot(
  std::string coordinator_session_id,
  const std::uint64_t revision,
  moveit_msgs::msg::PlanningScene scene)
: coordinator_session_id_(std::move(coordinator_session_id)), revision_(revision),
  scene_(std::move(scene))
{
}

const std::string & PlanningSceneSnapshot::coordinator_session_id() const noexcept
{
  return coordinator_session_id_;
}

std::uint64_t PlanningSceneSnapshot::revision() const noexcept
{
  return revision_;
}

const moveit_msgs::msg::PlanningScene & PlanningSceneSnapshot::scene() const noexcept
{
  return scene_;
}

PlanningSceneAccessProof PlanningSceneAccessProof::create(
  std::string lease_id,
  std::string coordinator_session_id,
  const std::uint64_t revision)
{
  require_token(lease_id, "PlanningScene lease ID");
  require_token(coordinator_session_id, "PlanningScene coordinator session ID");
  return PlanningSceneAccessProof(
    std::move(lease_id), std::move(coordinator_session_id), revision);
}

PlanningSceneAccessProof::PlanningSceneAccessProof(
  std::string lease_id,
  std::string coordinator_session_id,
  const std::uint64_t revision)
: lease_id_(std::move(lease_id)), coordinator_session_id_(std::move(coordinator_session_id)),
  revision_(revision)
{
}

const std::string & PlanningSceneAccessProof::lease_id() const noexcept
{
  return lease_id_;
}

const std::string & PlanningSceneAccessProof::coordinator_session_id() const noexcept
{
  return coordinator_session_id_;
}

std::uint64_t PlanningSceneAccessProof::revision() const noexcept
{
  return revision_;
}

GraspCandidate GraspCandidate::create(
  ObservationCorrelation correlation,
  geometry_msgs::msg::PoseStamped pregrasp_pose,
  geometry_msgs::msg::PoseStamped grasp_pose,
  geometry_msgs::msg::Vector3Stamped approach_direction,
  const double gripper_opening_m,
  const double gripper_force_n,
  const double score)
{
  if (!manipulation::validation::is_valid_pose_stamped(pregrasp_pose) ||
    !manipulation::validation::is_valid_pose_stamped(grasp_pose))
  {
    throw std::invalid_argument("grasp candidate poses must be valid stamped rigid poses");
  }
  if (pregrasp_pose.header.frame_id != grasp_pose.header.frame_id ||
    approach_direction.header.frame_id != grasp_pose.header.frame_id)
  {
    throw std::invalid_argument("grasp, pregrasp, and approach must use one frame");
  }
  if (!same_stamp(pregrasp_pose.header.stamp, correlation.stamp()) ||
    !same_stamp(grasp_pose.header.stamp, correlation.stamp()) ||
    !same_stamp(approach_direction.header.stamp, correlation.stamp()))
  {
    throw std::invalid_argument("grasp candidate stamps must match its observation correlation");
  }

  const auto & vector = approach_direction.vector;
  const double direction_norm_squared =
    vector.x * vector.x + vector.y * vector.y + vector.z * vector.z;
  if (!std::isfinite(direction_norm_squared) ||
    std::abs(direction_norm_squared - 1.0) > manipulation::validation::kRigidPoseTolerance)
  {
    throw std::invalid_argument("grasp approach direction must be a finite unit vector");
  }
  if (!std::isfinite(gripper_opening_m) || gripper_opening_m < 0.0 ||
    !std::isfinite(gripper_force_n) || gripper_force_n < 0.0 || !std::isfinite(score))
  {
    throw std::invalid_argument("grasp opening, force, and score values are invalid");
  }
  return GraspCandidate(
    std::move(correlation), std::move(pregrasp_pose), std::move(grasp_pose),
    std::move(approach_direction), gripper_opening_m, gripper_force_n, score);
}

GraspCandidate::GraspCandidate(
  ObservationCorrelation correlation,
  geometry_msgs::msg::PoseStamped pregrasp_pose,
  geometry_msgs::msg::PoseStamped grasp_pose,
  geometry_msgs::msg::Vector3Stamped approach_direction,
  const double gripper_opening_m,
  const double gripper_force_n,
  const double score)
: correlation_(std::move(correlation)), pregrasp_pose_(std::move(pregrasp_pose)),
  grasp_pose_(std::move(grasp_pose)), approach_direction_(std::move(approach_direction)),
  gripper_opening_m_(gripper_opening_m), gripper_force_n_(gripper_force_n), score_(score)
{
}

const ObservationCorrelation & GraspCandidate::correlation() const noexcept
{
  return correlation_;
}

const geometry_msgs::msg::PoseStamped & GraspCandidate::pregrasp_pose() const noexcept
{
  return pregrasp_pose_;
}

const geometry_msgs::msg::PoseStamped & GraspCandidate::grasp_pose() const noexcept
{
  return grasp_pose_;
}

const geometry_msgs::msg::Vector3Stamped & GraspCandidate::approach_direction() const noexcept
{
  return approach_direction_;
}

double GraspCandidate::gripper_opening_m() const noexcept
{
  return gripper_opening_m_;
}

double GraspCandidate::gripper_force_n() const noexcept
{
  return gripper_force_n_;
}

double GraspCandidate::score() const noexcept
{
  return score_;
}

std::vector<GraspCandidate> rank_grasp_candidates(
  const ObjectObservation & observation,
  std::vector<GraspCandidate> candidates)
{
  if (candidates.empty()) {
    throw std::invalid_argument("grasp provider returned no candidates");
  }
  if (!std::all_of(candidates.cbegin(), candidates.cend(), [&observation](const auto & candidate) {
      return candidate.correlation() == observation.correlation();
    }))
  {
    throw std::invalid_argument("grasp candidate is not correlated to the requested observation");
  }
  std::stable_sort(
    candidates.begin(), candidates.end(), [](const auto & left, const auto & right) {
      return left.score() > right.score();
    });
  return candidates;
}

ResolvedTransferTask ResolvedTransferTask::create(
  ObjectInstance instance,
  ObjectProfile profile,
  ObjectObservation observation,
  GraspCandidate grasp,
  PlanningSceneSnapshot scene_snapshot,
  PlanningSceneAccessProof scene_access_proof,
  geometry_msgs::msg::PoseStamped target_object_pose,
  const bool return_home,
  const TransferRecoveryPolicy recovery_policy)
{
  if (instance.profile_id() != profile.profile_id()) {
    throw std::invalid_argument("resolved transfer instance and profile IDs do not match");
  }
  if (instance.object_id() != observation.object_id()) {
    throw std::invalid_argument("resolved transfer observation targets another object instance");
  }
  if (grasp.correlation() != observation.correlation()) {
    throw std::invalid_argument("resolved transfer grasp targets another observation");
  }
  if (scene_access_proof.coordinator_session_id() !=
    scene_snapshot.coordinator_session_id() ||
    scene_access_proof.revision() != scene_snapshot.revision())
  {
    throw std::invalid_argument(
            "resolved transfer scene snapshot and access proof do not identify one revision");
  }
  if (!manipulation::validation::is_valid_pose_stamped(target_object_pose)) {
    throw std::invalid_argument("resolved transfer target must be a valid stamped rigid pose");
  }
  if (!valid_recovery_policy(recovery_policy)) {
    throw std::invalid_argument("resolved transfer uses an unknown recovery policy");
  }
  return ResolvedTransferTask(
    std::move(instance), std::move(profile), std::move(observation), std::move(grasp),
    std::move(scene_snapshot), std::move(scene_access_proof),
    std::move(target_object_pose), return_home, recovery_policy);
}

ResolvedTransferTask::ResolvedTransferTask(
  ObjectInstance instance,
  ObjectProfile profile,
  ObjectObservation observation,
  GraspCandidate grasp,
  PlanningSceneSnapshot scene_snapshot,
  PlanningSceneAccessProof scene_access_proof,
  geometry_msgs::msg::PoseStamped target_object_pose,
  const bool return_home,
  const TransferRecoveryPolicy recovery_policy)
: instance_(std::move(instance)), profile_(std::move(profile)),
  observation_(std::move(observation)), grasp_(std::move(grasp)),
  scene_snapshot_(std::move(scene_snapshot)), scene_access_proof_(std::move(scene_access_proof)),
  target_object_pose_(std::move(target_object_pose)),
  return_home_(return_home), recovery_policy_(recovery_policy)
{
}

const ObjectInstance & ResolvedTransferTask::instance() const noexcept
{
  return instance_;
}

const ObjectProfile & ResolvedTransferTask::profile() const noexcept
{
  return profile_;
}

const ObjectObservation & ResolvedTransferTask::observation() const noexcept
{
  return observation_;
}

const GraspCandidate & ResolvedTransferTask::grasp() const noexcept
{
  return grasp_;
}

const PlanningSceneSnapshot & ResolvedTransferTask::scene_snapshot() const noexcept
{
  return scene_snapshot_;
}

const PlanningSceneAccessProof & ResolvedTransferTask::scene_access_proof() const noexcept
{
  return scene_access_proof_;
}

const geometry_msgs::msg::PoseStamped & ResolvedTransferTask::target_object_pose() const noexcept
{
  return target_object_pose_;
}

bool ResolvedTransferTask::return_home() const noexcept
{
  return return_home_;
}

TransferRecoveryPolicy ResolvedTransferTask::recovery_policy() const noexcept
{
  return recovery_policy_;
}

}  // namespace manipulation::workflow
