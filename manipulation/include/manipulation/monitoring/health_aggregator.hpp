// Responsibility: Defines health evidence and deterministically aggregates it into system status.

#ifndef MANIPULATION__MONITORING__HEALTH_AGGREGATOR_HPP_
#define MANIPULATION__MONITORING__HEALTH_AGGREGATOR_HPP_

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "manipulation/monitoring/freshness_monitor.hpp"

namespace manipulation::monitoring
{

/// Stable identities of dependencies checked before manipulation starts.
enum class HealthComponentId : std::uint8_t
{
  kJointStates = 1,
  kSimulationClock = 2,
  kControllers = 3,
  kRobotTf = 4,
  // Value 5 preserves the existing ROS wire contract during the rename.
  kMotionBackend = 5,
  kGripper = 6,
  kPlanningScene = 7,
  kStereoPerception = 8,
  kObjectDetection = 9,
  kPoseEstimation = 10,
};

/// Machine-readable reasons for a component's non-OK state.
enum class HealthIssueCode : std::uint16_t
{
  kNone = 0,
  kJointStatesUnavailable = 100,
  kJointStatesDelayed = 101,
  kJointStatesStale = 102,
  kRequiredJointsMissing = 103,
  kJointStampMissing = 104,
  kJointStampOutsideTolerance = 105,
  kSimulationClockUnavailable = 200,
  kSimulationClockDelayed = 201,
  kSimulationClockStalled = 202,
  kControllerStateUnavailable = 300,
  kControllerStateDelayed = 301,
  kControllerStateStale = 302,
  kRequiredControllersInactive = 303,
  kRobotTfUnavailable = 400,
  // Value 500 preserves the existing ROS wire contract during the rename.
  kMotionBackendUnavailable = 500,
  kGripperUnavailable = 600,
  kPlanningSceneCoordinatorUnavailable = 700,
  kPlanningSceneStateUnavailable = 701,
  kPlanningSceneStateDelayed = 702,
  kPlanningSceneStateStale = 703,
  kPlanningSceneOperationInProgress = 704,
  kPlanningSceneOperationStuck = 705,
  kPlanningSceneBackendUnavailable = 706,
  kPlanningSceneBackendError = 707,
  kPlanningSceneReconciliationRequired = 708,
  kPlanningSceneLeaseExpired = 709,
  kPlanningSceneOperationCapacityExhausted = 710,
  kPerceptionDiagnosticUnavailable = 800,
  kPerceptionDiagnosticDelayed = 801,
  kPerceptionDiagnosticStale = 802,
  kPerceptionPipelineReportedWarn = 803,
  kPerceptionPipelineReportedError = 804,
};

/// Defines which steady-clock interval the age field represents.
enum class ObservationAgeSemantics : std::uint8_t
{
  kNotApplicable = 0,
  kSinceLastObservation = 1,
  kSinceMonitorStart = 2,
};

/// Latest allowlisted diagnostic state for one perception pipeline.
/**
 * The ROS adapter timestamps receipt with a steady clock and fills this value.
 * `diagnostic_msgs/DiagnosticArray.header.stamp` is intentionally absent: it is
 * ROS time and must never participate in freshness classification.
 */
struct PerceptionPipelineInput
{
  HealthComponentId id{HealthComponentId::kStereoPerception};
  std::string task_name;
  bool required{false};
  FreshnessStatus freshness;
  HealthLevel reported_level{HealthLevel::kStale};
  std::string reported_detail;
};

/// Dependency observations captured atomically enough for one health sample.
/**
 * The ROS adapter fills this structure without blocking on external services.
 * The aggregator is pure: it owns all readiness and severity policy, while the
 * adapter owns subscriptions, clients and clocks.
 */
struct HealthInputs
{
  std::uint64_t sequence{0};
  /// Elapsed process lifetime measured by the adapter's steady clock.
  std::chrono::nanoseconds steady_uptime{};
  bool startup_grace_active{false};

  FreshnessStatus joint_states;
  /// False when the robot profile omitted required_joint_names.
  bool joint_profile_configured{false};
  std::vector<std::string> missing_joints;
  /// Original JointState ROS timestamp and its age against the current ROS clock.
  std::int64_t joint_stamp_ns{0};
  std::optional<double> joint_stamp_age_sec;
  bool joint_stamp_outside_tolerance{false};

  bool simulation_clock_required{false};
  FreshnessStatus simulation_clock;
  std::uint64_t clock_reset_count{0};
  /// Last observed simulation-clock value in ROS nanoseconds.
  std::optional<std::int64_t> last_clock_ns;

  FreshnessStatus controllers;
  /// False when the robot profile omitted required_controllers.
  bool controller_profile_configured{false};
  std::vector<std::string> inactive_controllers;

  bool robot_tf_available{false};
  std::string robot_tf_error;
  bool motion_backend_available{false};
  bool gripper_available{false};

  bool planning_scene_service_available{false};
  FreshnessStatus planning_scene_state;
  std::string planning_scene_session_id;
  std::uint64_t planning_scene_revision{0};
  bool planning_scene_operation_in_progress{false};
  bool planning_scene_backend_ready{false};
  bool planning_scene_reconciliation_required{false};
  std::uint16_t planning_scene_backend_error_code{0};
  std::uint64_t planning_scene_expired_lease_count{0};
  std::uint64_t planning_scene_unreconciled_lease_expiry_count{0};
  bool planning_scene_operation_capacity_exhausted{false};
  /// Steady elapsed time since the coordinator first reported an active operation.
  std::optional<std::chrono::nanoseconds> planning_scene_operation_age;
  std::chrono::nanoseconds planning_scene_operation_stuck_after{};

  std::vector<PerceptionPipelineInput> perception_pipelines;
};

/// Complete classification of one component for diagnostics and typed output.
struct ComponentHealth
{
  HealthComponentId id{HealthComponentId::kJointStates};
  bool required{true};
  HealthLevel level{HealthLevel::kStale};
  std::vector<HealthIssueCode> issues;
  bool observed{false};
  bool startup_grace_active{false};
  ObservationAgeSemantics age_semantics{ObservationAgeSemantics::kNotApplicable};
  std::chrono::nanoseconds observation_age{};
  std::string detail;
};

/// Immutable result shared by all external representations of system health.
struct SystemHealth
{
  std::uint64_t sequence{0};
  std::chrono::nanoseconds steady_uptime{};
  bool motion_ready{false};
  HealthLevel overall_level{HealthLevel::kStale};
  std::vector<ComponentHealth> components;
};

/// Pure readiness policy for the manipulation dependency set.
class HealthAggregator
{
public:
  /// Classify one input sample without reading clocks or performing I/O.
  [[nodiscard]] SystemHealth aggregate(const HealthInputs & inputs) const;
};

/// Find a component in a complete aggregate or throw logic_error.
[[nodiscard]] const ComponentHealth & findComponent(
  const SystemHealth & health, HealthComponentId id);

}  // namespace manipulation::monitoring

#endif  // MANIPULATION__MONITORING__HEALTH_AGGREGATOR_HPP_
