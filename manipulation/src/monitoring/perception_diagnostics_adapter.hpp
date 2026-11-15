// Responsibility: Declares the thread-safe boundary from DiagnosticArray traffic to perception
//   evidence.

#ifndef MONITORING__PERCEPTION_DIAGNOSTICS_ADAPTER_HPP_
#define MONITORING__PERCEPTION_DIAGNOSTICS_ADAPTER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "manipulation/monitoring/health_aggregator.hpp"
#include "manipulation/monitoring/perception_health_policy.hpp"
#include "rclcpp/rclcpp.hpp"

namespace manipulation::monitoring
{

/// Bind one or more exact wire-status names to stable perception tasks.
/**
 * Receipt time comes exclusively from `std::chrono::steady_clock`. The adapter
 * ignores DiagnosticArray.header.stamp, performs no recovery, and exposes a
 * copy-only snapshot for the pure health policy. Alternative producers for a
 * stable task must be mutually exclusive; the latest exact observation updates
 * their shared state. The adapter owns its subscription and cache but borrows
 * the node used to create the subscription. Callback and snapshot calls may run
 * concurrently; `mutex_` protects both pipeline state and the wire-name index.
 */
class PerceptionDiagnosticsAdapter final
{
public:
  struct Config
  {
    std::string topic{"/diagnostics"};
    std::vector<PerceptionPipelineDescriptor> pipelines;
    std::vector<std::string> status_names;
    std::vector<std::string> required_tasks;
    FreshnessPolicy freshness;
    FreshnessMonitor::TimePoint started_at;
  };

  PerceptionDiagnosticsAdapter(rclcpp::Node & node, Config config);
  ~PerceptionDiagnosticsAdapter();

  PerceptionDiagnosticsAdapter(const PerceptionDiagnosticsAdapter &) = delete;
  PerceptionDiagnosticsAdapter & operator=(const PerceptionDiagnosticsAdapter &) = delete;

  /// Return one state per stable task, ordered by its first binding row.
  [[nodiscard]] std::vector<PerceptionPipelineInput> snapshot(
    FreshnessMonitor::TimePoint now) const;

private:
  struct PipelineState;

  void onDiagnostics(const diagnostic_msgs::msg::DiagnosticArray & message);

  mutable std::mutex mutex_;
  std::vector<std::unique_ptr<PipelineState>> pipelines_;
  std::unordered_map<std::string, PipelineState *> pipeline_by_status_name_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr subscription_;
};

}  // namespace manipulation::monitoring

#endif  // MONITORING__PERCEPTION_DIAGNOSTICS_ADAPTER_HPP_
