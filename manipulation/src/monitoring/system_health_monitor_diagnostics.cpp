// Responsibility: Registers diagnostic_updater tasks and renders component evidence for operators.

#include "system_health_monitor.hpp"

#include <chrono>
#include <cstdint>
#include <sstream>
#include <string>
#include <vector>

namespace manipulation::monitoring
{
namespace
{

double seconds(std::chrono::nanoseconds duration)
{
  return std::chrono::duration<double>(duration).count();
}

std::string join(const std::vector<std::string> & values)
{
  std::ostringstream stream;
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index != 0U) {
      stream << ", ";
    }
    stream << values[index];
  }
  return stream.str();
}

unsigned char diagnosticLevel(HealthLevel level)
{
  return static_cast<unsigned char>(level);
}

std::string joinIssues(const std::vector<HealthIssueCode> & issues)
{
  std::ostringstream stream;
  for (std::size_t index = 0; index < issues.size(); ++index) {
    if (index != 0U) {
      stream << ", ";
    }
    stream << static_cast<std::uint16_t>(issues[index]);
  }
  return stream.str();
}

void summarizeComponent(
  diagnostic_updater::DiagnosticStatusWrapper & status,
  const ComponentHealth & component)
{
  status.summary(diagnosticLevel(component.level), component.detail);
  status.add("component_id", static_cast<unsigned int>(component.id));
  status.add("issue_codes", joinIssues(component.issues));
  status.add("observed", component.observed);
  status.add("steady_age_sec", seconds(component.observation_age));
  status.add("startup_grace_active", component.startup_grace_active);
}

void addServiceRequestState(
  diagnostic_updater::DiagnosticStatusWrapper & status,
  const detail::ServiceRequestSnapshot & request,
  const std::chrono::nanoseconds timeout)
{
  status.add("request_in_flight", request.in_flight);
  status.add("request_generation", request.generation);
  status.add("request_id", request.request_id.value_or(0));
  status.add("request_steady_age_sec", seconds(request.age));
  status.add("request_timeout_sec", seconds(timeout));
  status.add("request_timeout_count", request.timeout_count);
}

}  // namespace

void SystemHealthMonitor::configureDiagnostics()
{
  diagnostics_ = std::make_unique<diagnostic_updater::Updater>(this);
  diagnostics_->setHardwareID("manipulation_system");
  diagnostics_->add("Joint states", this, &SystemHealthMonitor::diagnoseJointStates);
  diagnostics_->add("Simulation clock", this, &SystemHealthMonitor::diagnoseClock);
  diagnostics_->add("Controllers", this, &SystemHealthMonitor::diagnoseControllers);
  diagnostics_->add("Robot TF", this, &SystemHealthMonitor::diagnoseTf);
  diagnostics_->add(
    "Motion backend action", this,
    &SystemHealthMonitor::diagnoseMotionBackend);
  diagnostics_->add("Gripper action", this, &SystemHealthMonitor::diagnoseGripper);
  diagnostics_->add(
    "PlanningScene coordinator", this,
    &SystemHealthMonitor::diagnosePlanningScene);
}

void SystemHealthMonitor::diagnoseJointStates(
  diagnostic_updater::DiagnosticStatusWrapper & status)
{
  const auto report = latestHealthReport();
  const auto & component = findComponent(
    report.health, HealthComponentId::kJointStates);
  summarizeComponent(status, component);
  status.add("topic", joint_states_topic_);
  status.add("missing_joints", join(report.inputs.missing_joints));
  status.add("stamp_ns", report.inputs.joint_stamp_ns);
  if (report.inputs.joint_stamp_age_sec) {
    status.add("stamp_age_sec", *report.inputs.joint_stamp_age_sec);
  }
}

void SystemHealthMonitor::diagnoseClock(diagnostic_updater::DiagnosticStatusWrapper & status)
{
  const auto report = latestHealthReport();
  const auto & component = findComponent(
    report.health, HealthComponentId::kSimulationClock);
  summarizeComponent(status, component);
  status.add("required", report.inputs.simulation_clock_required);
  status.add("reset_count", report.inputs.clock_reset_count);
  status.add("last_clock_ns", report.inputs.last_clock_ns.value_or(0));
}

void SystemHealthMonitor::diagnoseControllers(
  diagnostic_updater::DiagnosticStatusWrapper & status)
{
  const auto report = latestHealthReport();
  const auto & component = findComponent(
    report.health, HealthComponentId::kControllers);
  summarizeComponent(status, component);
  status.add("service", controller_service_);
  status.add("inactive_or_missing", join(report.inputs.inactive_controllers));
  addServiceRequestState(
    status,
    controller_request_tracker_.snapshot(FreshnessMonitor::Clock::now()),
    service_request_timeout_);
}

void SystemHealthMonitor::diagnoseTf(diagnostic_updater::DiagnosticStatusWrapper & status)
{
  const auto report = latestHealthReport();
  const auto & component = findComponent(report.health, HealthComponentId::kRobotTf);
  summarizeComponent(status, component);
  status.add("available", report.inputs.robot_tf_available);
  status.add("base_frame", base_frame_);
  status.add("end_effector_frame", end_effector_frame_);
  if (!report.inputs.robot_tf_available && !report.inputs.robot_tf_error.empty()) {
    status.add("tf_error", report.inputs.robot_tf_error);
  }
}

void SystemHealthMonitor::diagnoseMotionBackend(
  diagnostic_updater::DiagnosticStatusWrapper & status)
{
  const auto report = latestHealthReport();
  const auto & component = findComponent(report.health, HealthComponentId::kMotionBackend);
  summarizeComponent(status, component);
  status.add("available", report.inputs.motion_backend_available);
  status.add("action", motion_backend_action_);
  status.add("action_type", motion_backend_action_type_);
}

void SystemHealthMonitor::diagnoseGripper(
  diagnostic_updater::DiagnosticStatusWrapper & status)
{
  const auto report = latestHealthReport();
  const auto & component = findComponent(report.health, HealthComponentId::kGripper);
  summarizeComponent(status, component);
  status.add("available", report.inputs.gripper_available);
  status.add("action", gripper_action_);
}

void SystemHealthMonitor::diagnosePlanningScene(
  diagnostic_updater::DiagnosticStatusWrapper & status)
{
  const auto report = latestHealthReport();
  const auto & component = findComponent(
    report.health, HealthComponentId::kPlanningScene);
  summarizeComponent(status, component);
  status.add("service", planning_scene_state_service_);
  addServiceRequestState(
    status,
    planning_scene_request_tracker_.snapshot(FreshnessMonitor::Clock::now()),
    service_request_timeout_);
  status.add(
    "service_available", report.inputs.planning_scene_service_available);
  status.add("session_id", report.inputs.planning_scene_session_id);
  status.add("revision", report.inputs.planning_scene_revision);
  status.add("backend_ready", report.inputs.planning_scene_backend_ready);
  status.add(
    "backend_error_code", report.inputs.planning_scene_backend_error_code);
  status.add(
    "reconciliation_required",
    report.inputs.planning_scene_reconciliation_required);
  status.add(
    "expired_leases_total",
    report.inputs.planning_scene_expired_lease_count);
  status.add(
    "unreconciled_lease_expiries",
    report.inputs.planning_scene_unreconciled_lease_expiry_count);
  status.add(
    "operation_capacity_exhausted",
    report.inputs.planning_scene_operation_capacity_exhausted);
  status.add(
    "operation_in_progress",
    report.inputs.planning_scene_operation_in_progress);
  if (report.inputs.planning_scene_operation_age) {
    status.add(
      "operation_steady_age_sec",
      seconds(*report.inputs.planning_scene_operation_age));
  }
}

}  // namespace manipulation::monitoring
