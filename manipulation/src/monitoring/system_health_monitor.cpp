// Responsibility: Builds the health monitor node, parameters, subscriptions, timers and service
//   clients.

#include "system_health_monitor.hpp"

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "duration_conversion.hpp"
#include "manipulation/validation/rigid_pose.hpp"
#include "rclcpp_action/create_generic_client.hpp"

namespace manipulation::monitoring
{

using namespace std::chrono_literals;

SystemHealthMonitor::SystemHealthMonitor()
: Node("system_health_monitor"), started_at_(FreshnessMonitor::Clock::now())
{
  startup_grace_ = detail::secondsToDuration(
    declare_parameter("startup_grace_sec", 10.0), "startup_grace_sec");
  joint_states_monitor_ = makeFreshnessMonitor(
    startup_grace_, "joint_states_warn_after_sec", 0.5,
    "joint_states_stale_after_sec", 2.0);
  clock_monitor_ = makeFreshnessMonitor(
    startup_grace_, "clock_warn_after_sec", 0.5, "clock_stale_after_sec", 2.0);
  controller_monitor_ = makeFreshnessMonitor(
    startup_grace_, "controllers_warn_after_sec", 2.0,
    "controllers_stale_after_sec", 5.0);
  planning_scene_monitor_ = makeFreshnessMonitor(
    startup_grace_, "planning_scene_warn_after_sec", 2.0,
    "planning_scene_stale_after_sec", 5.0);

  require_sim_clock_ = declare_parameter("require_sim_clock", false);
  joint_states_topic_ = declare_parameter("joint_states_topic", std::string("/joint_states"));
  controller_service_ = declare_parameter(
    "controller_manager_service", std::string("/controller_manager/list_controllers"));
  motion_backend_action_ = declare_parameter("motion_backend_action", std::string{});
  motion_backend_action_type_ = declare_parameter(
    "motion_backend_action_type", std::string{});
  gripper_action_ = declare_parameter("gripper_action", std::string{});
  planning_scene_state_service_ = declare_parameter(
    "planning_scene_state_service",
    std::string("/planning_scene_coordinator/get_state"));
  base_frame_ = declare_parameter("base_frame", std::string{});
  end_effector_frame_ = declare_parameter("end_effector_frame", std::string{});
  required_joint_names_ = declare_parameter(
    "required_joint_names", std::vector<std::string>{});
  required_controllers_ = declare_parameter(
    "required_controllers", std::vector<std::string>{});
  joint_stamp_tolerance_ = detail::secondsToDuration(
    declare_parameter("joint_stamp_tolerance_sec", 2.0), "joint_stamp_tolerance_sec");
  health_publish_period_ = detail::secondsToDuration(
    declare_parameter("health_publish_period_sec", 1.0), "health_publish_period_sec");
  if (health_publish_period_ < 100ms) {
    throw std::invalid_argument(
            "health_publish_period_sec must be at least 0.1 to bound publication rate");
  }
  if (health_publish_period_ > 1s) {
    throw std::invalid_argument(
            "health_publish_period_sec must not exceed 1.0 to bound readiness latency");
  }
  service_request_timeout_ = detail::secondsToDuration(
    declare_parameter("service_request_timeout_sec", 2.0),
    "service_request_timeout_sec");
  if (service_request_timeout_ == std::chrono::nanoseconds::zero()) {
    throw std::invalid_argument("service_request_timeout_sec must be positive");
  }
  planning_scene_operation_stuck_after_ = detail::secondsToDuration(
    declare_parameter("planning_scene_operation_stuck_after_sec", 30.0),
    "planning_scene_operation_stuck_after_sec");
  if (planning_scene_operation_stuck_after_ == std::chrono::nanoseconds::zero()) {
    throw std::invalid_argument(
            "planning_scene_operation_stuck_after_sec must be positive");
  }

  perception_diagnostics_topic_ = declare_parameter(
    "perception_diagnostics.topic", std::string("/diagnostics"));
  const auto perception_task_names = declare_parameter(
    "perception_diagnostics.allowed_tasks", std::vector<std::string>{});
  const auto perception_component_ids = declare_parameter(
    "perception_diagnostics.component_ids", std::vector<std::int64_t>{});
  perception_pipelines_ = makePerceptionPipelineDescriptors(
    perception_task_names, perception_component_ids);
  perception_status_names_ = declare_parameter(
    "perception_diagnostics.status_names", std::vector<std::string>{});
  required_perception_tasks_ = declare_parameter(
    "perception_diagnostics.required_tasks", std::vector<std::string>{});
  perception_freshness_policy_ = FreshnessPolicy{
    startup_grace_,
    detail::secondsToDuration(
      declare_parameter("perception_diagnostics.warn_after_sec", 2.0),
      "perception_diagnostics.warn_after_sec"),
    detail::secondsToDuration(
      declare_parameter("perception_diagnostics.stale_after_sec", 5.0),
      "perception_diagnostics.stale_after_sec")};

  declareFaultInjectionParameters();
  createRosInterfaces();
  configureHealthContract();
  configureDiagnostics();
}

std::unique_ptr<FreshnessMonitor> SystemHealthMonitor::makeFreshnessMonitor(
  std::chrono::nanoseconds startup_grace,
  const std::string & warn_parameter,
  double warn_default,
  const std::string & stale_parameter,
  double stale_default)
{
  const auto warn_after = detail::secondsToDuration(
    declare_parameter(warn_parameter, warn_default), warn_parameter);
  const auto stale_after = detail::secondsToDuration(
    declare_parameter(stale_parameter, stale_default), stale_parameter);
  return std::make_unique<FreshnessMonitor>(
    FreshnessPolicy{startup_grace, warn_after, stale_after}, started_at_);
}

void SystemHealthMonitor::declareFaultInjectionParameters()
{
  declare_parameter("fault_injection.force_joint_states_stale", false);
  declare_parameter("fault_injection.force_clock_stale", false);
  declare_parameter("fault_injection.force_controllers_unavailable", false);
  declare_parameter("fault_injection.force_tf_unavailable", false);
  declare_parameter("fault_injection.force_motion_backend_unavailable", false);
  declare_parameter("fault_injection.force_gripper_unavailable", false);
  declare_parameter("fault_injection.force_planning_scene_unavailable", false);
  declare_parameter("fault_injection.force_planning_scene_backend_unavailable", false);
  declare_parameter("fault_injection.force_planning_scene_backend_error", false);
  declare_parameter("fault_injection.force_planning_scene_reconciliation_required", false);
  declare_parameter("fault_injection.force_planning_scene_lease_expired", false);
  declare_parameter("fault_injection.force_planning_scene_capacity_exhausted", false);
  declare_parameter("fault_injection.force_perception_unavailable", false);
}

void SystemHealthMonitor::createRosInterfaces()
{
  joint_states_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
    joint_states_topic_, rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::JointState::ConstSharedPtr message) {
      if (!manipulation::validation::is_normalized_stamp(message->header.stamp)) {
        return;
      }
      const auto now = FreshnessMonitor::Clock::now();
      joint_states_monitor_->observe(now);
      std::lock_guard<std::mutex> lock(data_mutex_);
      joint_names_.clear();
      joint_names_.insert(message->name.begin(), message->name.end());
      joint_stamp_ns_ = rclcpp::Time(message->header.stamp).nanoseconds();
    });

  if (require_sim_clock_) {
    clock_subscription_ = create_subscription<rosgraph_msgs::msg::Clock>(
      "/clock", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
      [this](rosgraph_msgs::msg::Clock::ConstSharedPtr message) {
        if (!manipulation::validation::is_normalized_stamp(message->clock)) {
          return;
        }
        const auto now = FreshnessMonitor::Clock::now();
        const auto stamp_ns = rclcpp::Time(message->clock).nanoseconds();
        bool progressed = false;
        {
          std::lock_guard<std::mutex> lock(data_mutex_);
          if (!last_clock_ns_ || stamp_ns > *last_clock_ns_) {
            progressed = true;
          } else if (stamp_ns < *last_clock_ns_) {
            ++clock_reset_count_;
            progressed = true;
          }
          last_clock_ns_ = stamp_ns;
        }
        if (progressed) {
          clock_monitor_->observe(now);
        }
      });
  }

  controller_client_ = create_client<ListControllers>(controller_service_);
  controller_poll_timer_ = create_wall_timer(1s, [this]() {pollControllers();});
  planning_scene_client_ = create_client<GetPlanningSceneState>(
    planning_scene_state_service_);
  planning_scene_poll_timer_ = create_wall_timer(1s, [this]() {pollPlanningScene();});
  service_request_watchdog_timer_ = create_wall_timer(
    100ms, [this]() {expireServiceRequests();});

  if (motion_backend_action_.empty() || motion_backend_action_type_.empty()) {
    RCLCPP_ERROR(
      get_logger(),
      "Robot profile must configure motion_backend_action and "
      "motion_backend_action_type; motion readiness will fail closed");
  } else {
    try {
      motion_backend_client_ = rclcpp_action::create_generic_client(
        this, motion_backend_action_, motion_backend_action_type_);
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(
        get_logger(), "Cannot create motion backend action probe: %s",
        exception.what());
    }
  }
  if (gripper_action_.empty()) {
    RCLCPP_ERROR(
      get_logger(),
      "Robot profile must configure gripper_action; gripper readiness will fail closed");
  } else {
    gripper_client_ = rclcpp_action::create_client<GripperCommand>(this, gripper_action_);
  }

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this, false);

  perception_diagnostics_adapter_ = std::make_unique<PerceptionDiagnosticsAdapter>(
    *this,
    PerceptionDiagnosticsAdapter::Config{
      perception_diagnostics_topic_,
      perception_pipelines_,
      perception_status_names_,
      required_perception_tasks_,
      perception_freshness_policy_,
      started_at_});
}

}  // namespace manipulation::monitoring
