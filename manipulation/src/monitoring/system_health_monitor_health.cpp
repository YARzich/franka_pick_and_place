// Responsibility: Collects cached observations, applies fault injection and publishes health
//   snapshots.

#include "system_health_monitor.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

#include "manipulation_msgs/msg/planning_scene_operation_result.hpp"
#include "health_message_adapter.hpp"
#include "tf2/time.hpp"

namespace manipulation::monitoring
{
using SystemHealthSnapshot = manipulation_msgs::msg::SystemHealthSnapshot;

bool SystemHealthMonitor::faultEnabled(const std::string & name) const
{
  return get_parameter("fault_injection." + name).as_bool();
}

HealthInputs SystemHealthMonitor::collectHealthInputs()
{
  const auto steady_now = FreshnessMonitor::Clock::now();
  HealthInputs inputs;
  inputs.steady_uptime = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::max(
      steady_now - started_at_, FreshnessMonitor::Clock::duration::zero()));
  inputs.startup_grace_active = inputs.steady_uptime < startup_grace_;
  inputs.joint_states = joint_states_monitor_->status(steady_now);
  inputs.joint_profile_configured = !required_joint_names_.empty() &&
    std::all_of(
    required_joint_names_.begin(), required_joint_names_.end(),
    [](const std::string & name) {return !name.empty();}) &&
    std::set<std::string>(
    required_joint_names_.begin(), required_joint_names_.end()).size() ==
    required_joint_names_.size();
  inputs.simulation_clock_required = require_sim_clock_;
  inputs.simulation_clock = clock_monitor_->status(steady_now);
  inputs.controllers = controller_monitor_->status(steady_now);
  inputs.controller_profile_configured = !required_controllers_.empty() &&
    std::all_of(
    required_controllers_.begin(), required_controllers_.end(),
    [](const std::string & name) {return !name.empty();}) &&
    std::set<std::string>(
    required_controllers_.begin(), required_controllers_.end()).size() ==
    required_controllers_.size();
  inputs.planning_scene_state = planning_scene_monitor_->status(steady_now);
  inputs.perception_pipelines = perception_diagnostics_adapter_->snapshot(steady_now);

  if (faultEnabled("force_joint_states_stale")) {
    inputs.joint_states.level = HealthLevel::kStale;
  }
  if (faultEnabled("force_clock_stale")) {
    inputs.simulation_clock.level = HealthLevel::kStale;
  }
  if (faultEnabled("force_controllers_unavailable")) {
    inputs.controllers.level = HealthLevel::kStale;
  }
  if (faultEnabled("force_perception_unavailable")) {
    for (auto & pipeline : inputs.perception_pipelines) {
      pipeline.freshness.level = HealthLevel::kStale;
    }
  }
  const bool force_planning_scene_unavailable =
    faultEnabled("force_planning_scene_unavailable");
  if (force_planning_scene_unavailable) {
    inputs.planning_scene_state.level = HealthLevel::kStale;
  }

  std::set<std::string> joint_names;
  std::unordered_map<std::string, std::string> controller_states;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    joint_names = joint_names_;
    inputs.joint_stamp_ns = joint_stamp_ns_;
    inputs.clock_reset_count = clock_reset_count_;
    inputs.last_clock_ns = last_clock_ns_;
    controller_states = controller_states_;
    inputs.planning_scene_session_id = planning_scene_session_id_;
    inputs.planning_scene_revision = planning_scene_revision_;
    inputs.planning_scene_operation_in_progress =
      planning_scene_operation_in_progress_;
    inputs.planning_scene_backend_ready = planning_scene_backend_ready_;
    inputs.planning_scene_reconciliation_required =
      planning_scene_reconciliation_required_;
    inputs.planning_scene_backend_error_code = planning_scene_backend_error_code_;
    inputs.planning_scene_expired_lease_count =
      planning_scene_expired_lease_count_;
    inputs.planning_scene_unreconciled_lease_expiry_count =
      planning_scene_unreconciled_lease_expiry_count_;
    inputs.planning_scene_operation_capacity_exhausted =
      planning_scene_operation_capacity_exhausted_;
    if (planning_scene_operation_started_at_) {
      inputs.planning_scene_operation_age =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::max(
          steady_now - *planning_scene_operation_started_at_,
          FreshnessMonitor::Clock::duration::zero()));
    }
  }

  for (const auto & required : required_joint_names_) {
    if (joint_names.count(required) == 0U) {
      inputs.missing_joints.push_back(required);
    }
  }
  if (inputs.joint_states.observed && inputs.joint_stamp_ns != 0) {
    const long double stamp_age_ns =
      static_cast<long double>(get_clock()->now().nanoseconds()) -
      static_cast<long double>(inputs.joint_stamp_ns);
    inputs.joint_stamp_age_sec = static_cast<double>(stamp_age_ns / 1.0e9L);
    inputs.joint_stamp_outside_tolerance =
      std::fabs(stamp_age_ns) > static_cast<long double>(joint_stamp_tolerance_.count());
  }

  for (const auto & required : required_controllers_) {
    const auto found = controller_states.find(required);
    if (found == controller_states.end() || found->second != "active") {
      inputs.inactive_controllers.push_back(required);
    }
  }

  if (faultEnabled("force_planning_scene_backend_unavailable")) {
    inputs.planning_scene_backend_ready = false;
  }
  if (faultEnabled("force_planning_scene_backend_error")) {
    inputs.planning_scene_backend_error_code =
      manipulation_msgs::msg::PlanningSceneOperationResult::BACKEND_FAILED;
  }
  if (faultEnabled("force_planning_scene_reconciliation_required")) {
    inputs.planning_scene_reconciliation_required = true;
  }
  if (faultEnabled("force_planning_scene_lease_expired")) {
    inputs.planning_scene_unreconciled_lease_expiry_count = 1U;
  }
  if (faultEnabled("force_planning_scene_capacity_exhausted")) {
    inputs.planning_scene_operation_capacity_exhausted = true;
  }

  const bool tf_profile_configured = !base_frame_.empty() &&
    !end_effector_frame_.empty() && base_frame_ != end_effector_frame_;
  if (!tf_profile_configured) {
    inputs.robot_tf_error =
      "base_frame and end_effector_frame must be configured and distinct";
  } else {
    inputs.robot_tf_available = !faultEnabled("force_tf_unavailable") &&
      tf_buffer_->canTransform(
      base_frame_, end_effector_frame_, tf2::TimePointZero,
      tf2::durationFromSec(0.0), &inputs.robot_tf_error);
  }
  inputs.motion_backend_available = !faultEnabled("force_motion_backend_unavailable") &&
    motion_backend_client_ && motion_backend_client_->action_server_is_ready();
  inputs.gripper_available = !faultEnabled("force_gripper_unavailable") &&
    gripper_client_ && gripper_client_->action_server_is_ready();
  inputs.planning_scene_service_available =
    !force_planning_scene_unavailable && planning_scene_client_->service_is_ready();
  inputs.planning_scene_operation_stuck_after =
    planning_scene_operation_stuck_after_;
  return inputs;
}

void SystemHealthMonitor::configureHealthContract()
{
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  health_publisher_ = create_publisher<SystemHealthSnapshot>("~/health", qos);
  health_service_ = create_service<GetSystemHealth>(
    "~/get_health",
    [this](
      const std::shared_ptr<GetSystemHealth::Request>,
      std::shared_ptr<GetSystemHealth::Response> response)
    {
      const auto report = latestHealthReport();
      response->snapshot = detail::toHealthMessage(report.health, report.sample_stamp);
    });

  // Publish once immediately; transient-local QoS then serves late joiners.
  refreshHealthReport();
  health_publish_timer_ = create_wall_timer(
    health_publish_period_, [this]() {refreshHealthReport();});
}

void SystemHealthMonitor::refreshHealthReport()
{
  auto inputs = collectHealthInputs();
  {
    std::lock_guard<std::mutex> lock(health_mutex_);
    inputs.sequence = ++health_sequence_;
  }

  CachedHealthReport report;
  report.inputs = std::move(inputs);
  report.health = health_aggregator_.aggregate(report.inputs);
  report.sample_stamp = get_clock()->now();
  {
    std::lock_guard<std::mutex> lock(health_mutex_);
    latest_health_report_ = report;
  }
  health_publisher_->publish(detail::toHealthMessage(report.health, report.sample_stamp));
}

SystemHealthMonitor::CachedHealthReport SystemHealthMonitor::latestHealthReport() const
{
  std::lock_guard<std::mutex> lock(health_mutex_);
  if (!latest_health_report_) {
    throw std::logic_error("health snapshot requested before monitor initialization");
  }
  return *latest_health_report_;
}

}  // namespace manipulation::monitoring
