// Responsibility: Defines immutable observations, scene evidence, grasps, and resolved transfers.

#ifndef MANIPULATION__WORKFLOW__TRANSFER_WORKFLOW_TYPES_HPP_
#define MANIPULATION__WORKFLOW__TRANSFER_WORKFLOW_TYPES_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "moveit_msgs/msg/planning_scene.hpp"

#include "manipulation/workflow/object_catalog.hpp"

namespace manipulation::workflow
{

/** Stable identity tying downstream values to one stamped object observation. */
class ObservationCorrelation final
{
public:
  [[nodiscard]] static ObservationCorrelation create(
    std::string object_id,
    std::string frame_id,
    builtin_interfaces::msg::Time stamp,
    std::string correlation_id);

  [[nodiscard]] const std::string & object_id() const noexcept;
  [[nodiscard]] const std::string & frame_id() const noexcept;
  [[nodiscard]] const builtin_interfaces::msg::Time & stamp() const noexcept;
  [[nodiscard]] const std::string & correlation_id() const noexcept;

  [[nodiscard]] bool operator==(const ObservationCorrelation & other) const noexcept;
  [[nodiscard]] bool operator!=(const ObservationCorrelation & other) const noexcept;

private:
  ObservationCorrelation(
    std::string object_id,
    std::string frame_id,
    builtin_interfaces::msg::Time stamp,
    std::string correlation_id);

  std::string object_id_;
  std::string frame_id_;
  builtin_interfaces::msg::Time stamp_;
  std::string correlation_id_;
};

/** Immutable stamped pose and optional confidence for one physical object. */
class ObjectObservation final
{
public:
  [[nodiscard]] static ObjectObservation create(
    std::string object_id,
    geometry_msgs::msg::PoseStamped pose,
    std::string correlation_id,
    std::optional<double> confidence = std::nullopt);

  [[nodiscard]] const std::string & object_id() const noexcept;
  [[nodiscard]] const geometry_msgs::msg::PoseStamped & pose() const noexcept;
  [[nodiscard]] const ObservationCorrelation & correlation() const noexcept;
  [[nodiscard]] const std::optional<double> & confidence() const noexcept;

private:
  ObjectObservation(
    geometry_msgs::msg::PoseStamped pose,
    ObservationCorrelation correlation,
    std::optional<double> confidence);

  geometry_msgs::msg::PoseStamped pose_;
  ObservationCorrelation correlation_;
  std::optional<double> confidence_;
};

/** Immutable full-scene value authored by one coordinator session and revision. */
class PlanningSceneSnapshot final
{
public:
  /** The supplied PlanningScene must be a complete value, never a diff. */
  [[nodiscard]] static PlanningSceneSnapshot create(
    std::string coordinator_session_id,
    std::uint64_t revision,
    moveit_msgs::msg::PlanningScene scene);

  [[nodiscard]] const std::string & coordinator_session_id() const noexcept;
  [[nodiscard]] std::uint64_t revision() const noexcept;
  [[nodiscard]] const moveit_msgs::msg::PlanningScene & scene() const noexcept;

private:
  PlanningSceneSnapshot(
    std::string coordinator_session_id,
    std::uint64_t revision,
    moveit_msgs::msg::PlanningScene scene);

  std::string coordinator_session_id_;
  std::uint64_t revision_{0U};
  moveit_msgs::msg::PlanningScene scene_;
};

/** Immutable coordinator authorization evidence for later scene mutations. */
class PlanningSceneAccessProof final
{
public:
  [[nodiscard]] static PlanningSceneAccessProof create(
    std::string lease_id,
    std::string coordinator_session_id,
    std::uint64_t revision);

  [[nodiscard]] const std::string & lease_id() const noexcept;
  [[nodiscard]] const std::string & coordinator_session_id() const noexcept;
  [[nodiscard]] std::uint64_t revision() const noexcept;

private:
  PlanningSceneAccessProof(
    std::string lease_id,
    std::string coordinator_session_id,
    std::uint64_t revision);

  std::string lease_id_;
  std::string coordinator_session_id_;
  std::uint64_t revision_{0U};
};

/** One provider-ranked grasp correlated to its exact stamped observation. */
class GraspCandidate final
{
public:
  [[nodiscard]] static GraspCandidate create(
    ObservationCorrelation correlation,
    geometry_msgs::msg::PoseStamped pregrasp_pose,
    geometry_msgs::msg::PoseStamped grasp_pose,
    geometry_msgs::msg::Vector3Stamped approach_direction,
    double gripper_opening_m,
    double gripper_force_n,
    double score);

  [[nodiscard]] const ObservationCorrelation & correlation() const noexcept;
  [[nodiscard]] const geometry_msgs::msg::PoseStamped & pregrasp_pose() const noexcept;
  [[nodiscard]] const geometry_msgs::msg::PoseStamped & grasp_pose() const noexcept;
  [[nodiscard]] const geometry_msgs::msg::Vector3Stamped & approach_direction() const noexcept;
  [[nodiscard]] double gripper_opening_m() const noexcept;
  [[nodiscard]] double gripper_force_n() const noexcept;
  [[nodiscard]] double score() const noexcept;

private:
  GraspCandidate(
    ObservationCorrelation correlation,
    geometry_msgs::msg::PoseStamped pregrasp_pose,
    geometry_msgs::msg::PoseStamped grasp_pose,
    geometry_msgs::msg::Vector3Stamped approach_direction,
    double gripper_opening_m,
    double gripper_force_n,
    double score);

  ObservationCorrelation correlation_;
  geometry_msgs::msg::PoseStamped pregrasp_pose_;
  geometry_msgs::msg::PoseStamped grasp_pose_;
  geometry_msgs::msg::Vector3Stamped approach_direction_;
  double gripper_opening_m_{0.0};
  double gripper_force_n_{0.0};
  double score_{0.0};
};

/**
 * Validate correlation and return candidates in stable descending score order.
 *
 * An empty set and any candidate not tied to `observation` are contract errors
 * reported as `std::invalid_argument`.
 */
[[nodiscard]] std::vector<GraspCandidate> rank_grasp_candidates(
  const ObjectObservation & observation,
  std::vector<GraspCandidate> candidates);

/// Recovery intent independent from any generated action definition.
enum class TransferRecoveryPolicy : std::uint8_t
{
  kNone,
  kSafeStop,
  kRetreat,
  kRetreatAndHome,
};

/** Complete immutable input consumed by one execution strategy. */
class ResolvedTransferTask final
{
public:
  [[nodiscard]] static ResolvedTransferTask create(
    ObjectInstance instance,
    ObjectProfile profile,
    ObjectObservation observation,
    GraspCandidate grasp,
    PlanningSceneSnapshot scene_snapshot,
    PlanningSceneAccessProof scene_access_proof,
    geometry_msgs::msg::PoseStamped target_object_pose,
    bool return_home,
    TransferRecoveryPolicy recovery_policy);

  [[nodiscard]] const ObjectInstance & instance() const noexcept;
  [[nodiscard]] const ObjectProfile & profile() const noexcept;
  [[nodiscard]] const ObjectObservation & observation() const noexcept;
  [[nodiscard]] const GraspCandidate & grasp() const noexcept;
  [[nodiscard]] const PlanningSceneSnapshot & scene_snapshot() const noexcept;
  [[nodiscard]] const PlanningSceneAccessProof & scene_access_proof() const noexcept;
  [[nodiscard]] const geometry_msgs::msg::PoseStamped & target_object_pose() const noexcept;
  [[nodiscard]] bool return_home() const noexcept;
  [[nodiscard]] TransferRecoveryPolicy recovery_policy() const noexcept;

private:
  ResolvedTransferTask(
    ObjectInstance instance,
    ObjectProfile profile,
    ObjectObservation observation,
    GraspCandidate grasp,
    PlanningSceneSnapshot scene_snapshot,
    PlanningSceneAccessProof scene_access_proof,
    geometry_msgs::msg::PoseStamped target_object_pose,
    bool return_home,
    TransferRecoveryPolicy recovery_policy);

  ObjectInstance instance_;
  ObjectProfile profile_;
  ObjectObservation observation_;
  GraspCandidate grasp_;
  PlanningSceneSnapshot scene_snapshot_;
  PlanningSceneAccessProof scene_access_proof_;
  geometry_msgs::msg::PoseStamped target_object_pose_;
  bool return_home_{false};
  TransferRecoveryPolicy recovery_policy_{TransferRecoveryPolicy::kNone};
};

}  // namespace manipulation::workflow

#endif  // MANIPULATION__WORKFLOW__TRANSFER_WORKFLOW_TYPES_HPP_
