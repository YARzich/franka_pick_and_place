// Responsibility: Converts internal health snapshots and enums to stable ROS message contracts.

#include "health_message_adapter.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iterator>
#include <limits>
#include <stdexcept>

#include "builtin_interfaces/msg/duration.hpp"
#include "manipulation_msgs/msg/health_component_status.hpp"

namespace manipulation::monitoring::detail
{
namespace
{

using HealthComponentStatus = manipulation_msgs::msg::HealthComponentStatus;

builtin_interfaces::msg::Duration toWireDuration(std::chrono::nanoseconds duration)
{
  constexpr auto kMaxDuration = std::chrono::seconds{std::numeric_limits<std::int32_t>::max()} +
  std::chrono::nanoseconds{999999999};
  const auto bounded = std::clamp(duration, std::chrono::nanoseconds::zero(), kMaxDuration);
  const auto whole_seconds = std::chrono::duration_cast<std::chrono::seconds>(bounded);

  builtin_interfaces::msg::Duration result;
  result.sec = static_cast<std::int32_t>(whole_seconds.count());
  result.nanosec = static_cast<std::uint32_t>((bounded - whole_seconds).count());
  return result;
}

std::uint8_t toWireLevel(HealthLevel level)
{
  switch (level) {
    case HealthLevel::kOk:
      return HealthComponentStatus::LEVEL_OK;
    case HealthLevel::kWarn:
      return HealthComponentStatus::LEVEL_WARN;
    case HealthLevel::kError:
      return HealthComponentStatus::LEVEL_ERROR;
    case HealthLevel::kStale:
      return HealthComponentStatus::LEVEL_STALE;
  }
  throw std::logic_error("unsupported health level");
}

std::uint8_t toWireComponent(HealthComponentId id)
{
  switch (id) {
    case HealthComponentId::kJointStates:
      return HealthComponentStatus::COMPONENT_JOINT_STATES;
    case HealthComponentId::kSimulationClock:
      return HealthComponentStatus::COMPONENT_SIMULATION_CLOCK;
    case HealthComponentId::kControllers:
      return HealthComponentStatus::COMPONENT_CONTROLLERS;
    case HealthComponentId::kRobotTf:
      return HealthComponentStatus::COMPONENT_ROBOT_TF;
    case HealthComponentId::kMotionBackend:
      // The renamed internal concept keeps wire value 5 until the message
      // contract performs its own coordinated compatibility migration.
      return HealthComponentStatus::COMPONENT_MOVE_GROUP;
    case HealthComponentId::kGripper:
      return HealthComponentStatus::COMPONENT_GRIPPER;
    case HealthComponentId::kPlanningScene:
      return HealthComponentStatus::COMPONENT_PLANNING_SCENE;
    case HealthComponentId::kStereoPerception:
      return HealthComponentStatus::COMPONENT_STEREO_PERCEPTION;
    case HealthComponentId::kObjectDetection:
      return HealthComponentStatus::COMPONENT_OBJECT_DETECTION;
    case HealthComponentId::kPoseEstimation:
      return HealthComponentStatus::COMPONENT_POSE_ESTIMATION;
  }
  throw std::logic_error("unsupported health component");
}

std::uint8_t toWireAgeSemantics(ObservationAgeSemantics semantics)
{
  switch (semantics) {
    case ObservationAgeSemantics::kNotApplicable:
      return HealthComponentStatus::AGE_NOT_APPLICABLE;
    case ObservationAgeSemantics::kSinceLastObservation:
      return HealthComponentStatus::AGE_SINCE_LAST_OBSERVATION;
    case ObservationAgeSemantics::kSinceMonitorStart:
      return HealthComponentStatus::AGE_SINCE_MONITOR_START;
  }
  throw std::logic_error("unsupported observation-age semantics");
}

std::uint16_t toWireIssue(HealthIssueCode issue)
{
  // Internal values never leak accidentally into the stable ROS interface.
  switch (issue) {
    case HealthIssueCode::kNone:
      return HealthComponentStatus::ISSUE_NONE;
    case HealthIssueCode::kJointStatesUnavailable:
      return HealthComponentStatus::ISSUE_JOINT_STATES_UNAVAILABLE;
    case HealthIssueCode::kJointStatesDelayed:
      return HealthComponentStatus::ISSUE_JOINT_STATES_DELAYED;
    case HealthIssueCode::kJointStatesStale:
      return HealthComponentStatus::ISSUE_JOINT_STATES_STALE;
    case HealthIssueCode::kRequiredJointsMissing:
      return HealthComponentStatus::ISSUE_REQUIRED_JOINTS_MISSING;
    case HealthIssueCode::kJointStampMissing:
      return HealthComponentStatus::ISSUE_JOINT_STAMP_MISSING;
    case HealthIssueCode::kJointStampOutsideTolerance:
      return HealthComponentStatus::ISSUE_JOINT_STAMP_OUTSIDE_TOLERANCE;
    case HealthIssueCode::kSimulationClockUnavailable:
      return HealthComponentStatus::ISSUE_SIMULATION_CLOCK_UNAVAILABLE;
    case HealthIssueCode::kSimulationClockDelayed:
      return HealthComponentStatus::ISSUE_SIMULATION_CLOCK_DELAYED;
    case HealthIssueCode::kSimulationClockStalled:
      return HealthComponentStatus::ISSUE_SIMULATION_CLOCK_STALLED;
    case HealthIssueCode::kControllerStateUnavailable:
      return HealthComponentStatus::ISSUE_CONTROLLER_STATE_UNAVAILABLE;
    case HealthIssueCode::kControllerStateDelayed:
      return HealthComponentStatus::ISSUE_CONTROLLER_STATE_DELAYED;
    case HealthIssueCode::kControllerStateStale:
      return HealthComponentStatus::ISSUE_CONTROLLER_STATE_STALE;
    case HealthIssueCode::kRequiredControllersInactive:
      return HealthComponentStatus::ISSUE_REQUIRED_CONTROLLERS_INACTIVE;
    case HealthIssueCode::kRobotTfUnavailable:
      return HealthComponentStatus::ISSUE_ROBOT_TF_UNAVAILABLE;
    case HealthIssueCode::kMotionBackendUnavailable:
      // Preserve wire value 500 while old clients still use the MoveGroup name.
      return HealthComponentStatus::ISSUE_MOVE_GROUP_UNAVAILABLE;
    case HealthIssueCode::kGripperUnavailable:
      return HealthComponentStatus::ISSUE_GRIPPER_UNAVAILABLE;
    case HealthIssueCode::kPlanningSceneCoordinatorUnavailable:
      return HealthComponentStatus::ISSUE_PLANNING_SCENE_COORDINATOR_UNAVAILABLE;
    case HealthIssueCode::kPlanningSceneStateUnavailable:
      return HealthComponentStatus::ISSUE_PLANNING_SCENE_STATE_UNAVAILABLE;
    case HealthIssueCode::kPlanningSceneStateDelayed:
      return HealthComponentStatus::ISSUE_PLANNING_SCENE_STATE_DELAYED;
    case HealthIssueCode::kPlanningSceneStateStale:
      return HealthComponentStatus::ISSUE_PLANNING_SCENE_STATE_STALE;
    case HealthIssueCode::kPlanningSceneOperationInProgress:
      return HealthComponentStatus::ISSUE_PLANNING_SCENE_OPERATION_IN_PROGRESS;
    case HealthIssueCode::kPlanningSceneOperationStuck:
      return HealthComponentStatus::ISSUE_PLANNING_SCENE_OPERATION_STUCK;
    case HealthIssueCode::kPlanningSceneBackendUnavailable:
      return HealthComponentStatus::ISSUE_PLANNING_SCENE_BACKEND_UNAVAILABLE;
    case HealthIssueCode::kPlanningSceneBackendError:
      return HealthComponentStatus::ISSUE_PLANNING_SCENE_BACKEND_ERROR;
    case HealthIssueCode::kPlanningSceneReconciliationRequired:
      return HealthComponentStatus::ISSUE_PLANNING_SCENE_RECONCILIATION_REQUIRED;
    case HealthIssueCode::kPlanningSceneLeaseExpired:
      return HealthComponentStatus::ISSUE_PLANNING_SCENE_LEASE_EXPIRED;
    case HealthIssueCode::kPlanningSceneOperationCapacityExhausted:
      return HealthComponentStatus::ISSUE_PLANNING_SCENE_OPERATION_CAPACITY_EXHAUSTED;
    case HealthIssueCode::kPerceptionDiagnosticUnavailable:
      return HealthComponentStatus::ISSUE_PERCEPTION_DIAGNOSTIC_UNAVAILABLE;
    case HealthIssueCode::kPerceptionDiagnosticDelayed:
      return HealthComponentStatus::ISSUE_PERCEPTION_DIAGNOSTIC_DELAYED;
    case HealthIssueCode::kPerceptionDiagnosticStale:
      return HealthComponentStatus::ISSUE_PERCEPTION_DIAGNOSTIC_STALE;
    case HealthIssueCode::kPerceptionPipelineReportedWarn:
      return HealthComponentStatus::ISSUE_PERCEPTION_PIPELINE_REPORTED_WARN;
    case HealthIssueCode::kPerceptionPipelineReportedError:
      return HealthComponentStatus::ISSUE_PERCEPTION_PIPELINE_REPORTED_ERROR;
  }
  throw std::logic_error("unsupported health issue");
}

HealthComponentStatus toWireComponentStatus(const ComponentHealth & component)
{
  HealthComponentStatus result;
  result.component_id = toWireComponent(component.id);
  result.required = component.required;
  result.level = toWireLevel(component.level);
  result.issue_codes.reserve(component.issues.size());
  std::transform(
    component.issues.begin(), component.issues.end(),
    std::back_inserter(result.issue_codes), toWireIssue);
  result.observed = component.observed;
  result.startup_grace_active = component.startup_grace_active;
  result.age_semantics = toWireAgeSemantics(component.age_semantics);
  result.steady_observation_age = toWireDuration(component.observation_age);
  result.detail = component.detail;
  return result;
}

}  // namespace

manipulation_msgs::msg::SystemHealthSnapshot toHealthMessage(
  const SystemHealth & health,
  const builtin_interfaces::msg::Time & sample_stamp)
{
  manipulation_msgs::msg::SystemHealthSnapshot message;
  message.sample_stamp = sample_stamp;
  message.sequence = health.sequence;
  message.steady_uptime = toWireDuration(health.steady_uptime);
  message.motion_ready = health.motion_ready;
  message.overall_level = toWireLevel(health.overall_level);
  message.components.reserve(health.components.size());
  std::transform(
    health.components.begin(), health.components.end(),
    std::back_inserter(message.components), toWireComponentStatus);
  return message;
}

}  // namespace manipulation::monitoring::detail
