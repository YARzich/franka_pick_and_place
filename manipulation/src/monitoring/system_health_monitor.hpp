// Responsibility: Owns ROS health observations and coordinates polling, aggregation and
//   diagnostics.

#ifndef MONITORING__SYSTEM_HEALTH_MONITOR_HPP_
#define MONITORING__SYSTEM_HEALTH_MONITOR_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "control_msgs/action/gripper_command.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "manipulation/monitoring/freshness_monitor.hpp"
#include "manipulation/monitoring/health_aggregator.hpp"
#include "manipulation_msgs/msg/system_health_snapshot.hpp"
#include "manipulation_msgs/srv/get_planning_scene_state.hpp"
#include "manipulation_msgs/srv/get_system_health.hpp"
#include "perception_diagnostics_adapter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/generic_client.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "service_request_tracker.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace manipulation::monitoring
{

/**
 * Read-only ROS adapter that reports manipulation-system health.
 *
 * Recovery and motion side effects belong to the task engine. This node only
 * observes dependencies and publishes standard and typed health snapshots.
 * Freshness and service-response deadlines use a steady clock; message
 * timestamps are checked against the node's ROS clock.
 *
 * The node owns subscriptions, clients, timers, TF and diagnostics. Callbacks
 * may execute concurrently in a MultiThreadedExecutor: message-derived state is
 * protected by `data_mutex_`, the published aggregate by `health_mutex_`, and
 * each freshness/request tracker owns its own lock. Service callbacks never
 * hold `data_mutex_` while updating freshness, preserving the lock order. A
 * missing, late or malformed dependency fails readiness closed; this node does
 * not attempt controller, TF, sensor or PlanningScene recovery.
 */
class SystemHealthMonitor final : public rclcpp::Node
{
public:
  SystemHealthMonitor();

private:
  using ListControllers = controller_manager_msgs::srv::ListControllers;
  using GripperCommand = control_msgs::action::GripperCommand;
  using GetSystemHealth = manipulation_msgs::srv::GetSystemHealth;
  using GetPlanningSceneState = manipulation_msgs::srv::GetPlanningSceneState;

  struct CachedHealthReport
  {
    HealthInputs inputs;
    SystemHealth health;
    builtin_interfaces::msg::Time sample_stamp;
  };

  std::unique_ptr<FreshnessMonitor> makeFreshnessMonitor(
    std::chrono::nanoseconds startup_grace,
    const std::string & warn_parameter,
    double warn_default,
    const std::string & stale_parameter,
    double stale_default);
  void declareFaultInjectionParameters();
  void createRosInterfaces();
  void configureHealthContract();
  void configureDiagnostics();
  void pollControllers();
  void pollPlanningScene();
  void expireServiceRequests();

  [[nodiscard]] bool faultEnabled(const std::string & name) const;
  [[nodiscard]] HealthInputs collectHealthInputs();
  void refreshHealthReport();
  [[nodiscard]] CachedHealthReport latestHealthReport() const;
  void diagnoseJointStates(diagnostic_updater::DiagnosticStatusWrapper & status);
  void diagnoseClock(diagnostic_updater::DiagnosticStatusWrapper & status);
  void diagnoseControllers(diagnostic_updater::DiagnosticStatusWrapper & status);
  void diagnoseTf(diagnostic_updater::DiagnosticStatusWrapper & status);
  void diagnoseMotionBackend(diagnostic_updater::DiagnosticStatusWrapper & status);
  void diagnoseGripper(diagnostic_updater::DiagnosticStatusWrapper & status);
  void diagnosePlanningScene(diagnostic_updater::DiagnosticStatusWrapper & status);

  const FreshnessMonitor::TimePoint started_at_;
  std::unique_ptr<FreshnessMonitor> joint_states_monitor_;
  std::unique_ptr<FreshnessMonitor> clock_monitor_;
  std::unique_ptr<FreshnessMonitor> controller_monitor_;
  std::unique_ptr<FreshnessMonitor> planning_scene_monitor_;

  bool require_sim_clock_{false};
  std::string joint_states_topic_;
  std::string controller_service_;
  std::string motion_backend_action_;
  std::string motion_backend_action_type_;
  std::string gripper_action_;
  std::string planning_scene_state_service_;
  std::string perception_diagnostics_topic_;
  std::string base_frame_;
  std::string end_effector_frame_;
  std::vector<std::string> required_joint_names_;
  std::vector<std::string> required_controllers_;
  std::vector<PerceptionPipelineDescriptor> perception_pipelines_;
  std::vector<std::string> perception_status_names_;
  std::vector<std::string> required_perception_tasks_;
  std::chrono::nanoseconds startup_grace_{};
  std::chrono::nanoseconds joint_stamp_tolerance_{};
  std::chrono::nanoseconds health_publish_period_{};
  std::chrono::nanoseconds service_request_timeout_{};
  std::chrono::nanoseconds planning_scene_operation_stuck_after_{};
  FreshnessPolicy perception_freshness_policy_;

  // Protects only message-derived snapshots; FreshnessMonitor has its own lock.
  mutable std::mutex data_mutex_;
  std::set<std::string> joint_names_;
  std::int64_t joint_stamp_ns_{0};
  std::optional<std::int64_t> last_clock_ns_;
  std::uint64_t clock_reset_count_{0};
  std::unordered_map<std::string, std::string> controller_states_;
  std::string planning_scene_session_id_;
  std::uint64_t planning_scene_revision_{0};
  bool planning_scene_operation_in_progress_{false};
  bool planning_scene_backend_ready_{false};
  bool planning_scene_reconciliation_required_{false};
  std::uint16_t planning_scene_backend_error_code_{0};
  std::uint64_t planning_scene_expired_lease_count_{0};
  std::uint64_t planning_scene_unreconciled_lease_expiry_count_{0};
  bool planning_scene_operation_capacity_exhausted_{false};
  std::optional<FreshnessMonitor::TimePoint> planning_scene_operation_started_at_;

  // Generation tokens prevent a late response from completing a newer poll.
  detail::ServiceRequestTracker controller_request_tracker_;
  detail::ServiceRequestTracker planning_scene_request_tracker_;

  HealthAggregator health_aggregator_;
  mutable std::mutex health_mutex_;
  std::optional<CachedHealthReport> latest_health_report_;
  std::uint64_t health_sequence_{0};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscription_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_subscription_;
  rclcpp::Client<ListControllers>::SharedPtr controller_client_;
  rclcpp::TimerBase::SharedPtr controller_poll_timer_;
  rclcpp_action::GenericClient::SharedPtr motion_backend_client_;
  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_client_;
  rclcpp::Client<GetPlanningSceneState>::SharedPtr planning_scene_client_;
  rclcpp::TimerBase::SharedPtr planning_scene_poll_timer_;
  rclcpp::TimerBase::SharedPtr service_request_watchdog_timer_;
  std::unique_ptr<PerceptionDiagnosticsAdapter> perception_diagnostics_adapter_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<manipulation_msgs::msg::SystemHealthSnapshot>::SharedPtr
    health_publisher_;
  rclcpp::Service<GetSystemHealth>::SharedPtr health_service_;
  rclcpp::TimerBase::SharedPtr health_publish_timer_;
  std::unique_ptr<diagnostic_updater::Updater> diagnostics_;
};

}  // namespace manipulation::monitoring

#endif  // MONITORING__SYSTEM_HEALTH_MONITOR_HPP_
