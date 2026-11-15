// Responsibility: Maintains per-pipeline health evidence from stamped ROS diagnostic updates.

#include "perception_diagnostics_adapter.hpp"

#include <cstddef>
#include <cstdint>
#include <set>
#include <stdexcept>
#include <utility>

#include "manipulation/monitoring/perception_health_policy.hpp"

namespace manipulation::monitoring
{
namespace
{

constexpr std::size_t kMaxCachedDetailLength = 512U;

std::string boundedDetail(const std::string & detail)
{
  if (detail.size() <= kMaxCachedDetailLength) {
    return detail;
  }
  return detail.substr(0U, kMaxCachedDetailLength) + "...";
}

}  // namespace

struct PerceptionDiagnosticsAdapter::PipelineState
{
  PipelineState(
    std::string task,
    HealthComponentId component_id,
    bool is_required,
    FreshnessPolicy policy,
    FreshnessMonitor::TimePoint started_at)
  : task_name(std::move(task)), id(component_id), required(is_required),
    freshness(policy, started_at)
  {
  }

  std::string task_name;
  HealthComponentId id;
  bool required;
  FreshnessMonitor freshness;
  HealthLevel reported_level{HealthLevel::kStale};
  std::string reported_detail;
};

PerceptionDiagnosticsAdapter::PerceptionDiagnosticsAdapter(
  rclcpp::Node & node,
  Config config)
{
  if (config.topic.empty()) {
    throw std::invalid_argument("perception diagnostics topic must not be empty");
  }
  const FreshnessMonitor policy_validator(config.freshness, config.started_at);
  (void)policy_validator;
  validatePerceptionDiagnosticBindings(
    config.pipelines, config.status_names, config.required_tasks);
  const std::set<std::string> required(
    config.required_tasks.begin(), config.required_tasks.end());
  // Alternative wire names share the state owned by their stable task.
  std::unordered_map<std::string, PipelineState *> pipeline_by_stable_task;

  pipelines_.reserve(config.pipelines.size());
  pipeline_by_stable_task.reserve(config.pipelines.size());
  pipeline_by_status_name_.reserve(config.pipelines.size());
  for (std::size_t index = 0U; index < config.pipelines.size(); ++index) {
    const auto & descriptor = config.pipelines[index];
    const auto & task_name = descriptor.task_name;
    const auto & status_name = config.status_names[index];
    PipelineState * pipeline{nullptr};
    const auto existing = pipeline_by_stable_task.find(task_name);
    if (existing == pipeline_by_stable_task.end()) {
      auto owned_pipeline = std::make_unique<PipelineState>(
        task_name, descriptor.component_id, required.count(task_name) != 0U,
        config.freshness, config.started_at);
      pipeline = owned_pipeline.get();
      pipeline_by_stable_task.emplace(task_name, pipeline);
      pipelines_.push_back(std::move(owned_pipeline));
    } else {
      pipeline = existing->second;
    }
    pipeline_by_status_name_.emplace(status_name, pipeline);
  }

  if (pipelines_.empty()) {
    return;
  }
  subscription_ = node.create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    config.topic, rclcpp::QoS(rclcpp::KeepLast(20)).reliable(),
    [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr message) {
      onDiagnostics(*message);
    });
}

PerceptionDiagnosticsAdapter::~PerceptionDiagnosticsAdapter() = default;

std::vector<PerceptionPipelineInput> PerceptionDiagnosticsAdapter::snapshot(
  FreshnessMonitor::TimePoint now) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<PerceptionPipelineInput> result;
  result.reserve(pipelines_.size());
  for (const auto & pipeline : pipelines_) {
    result.push_back(PerceptionPipelineInput{
        pipeline->id,
        pipeline->task_name,
        pipeline->required,
        pipeline->freshness.status(now),
        pipeline->reported_level,
        pipeline->reported_detail});
  }
  return result;
}

void PerceptionDiagnosticsAdapter::onDiagnostics(
  const diagnostic_msgs::msg::DiagnosticArray & message)
{
  const auto received_at = FreshnessMonitor::Clock::now();
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & status : message.status) {
    const auto found = pipeline_by_status_name_.find(status.name);
    if (found == pipeline_by_status_name_.end()) {
      continue;
    }
    auto & pipeline = *found->second;
    pipeline.freshness.observe(received_at);
    pipeline.reported_level = healthLevelFromDiagnostic(status.level);
    if (status.level > 3U) {
      pipeline.reported_detail =
        "Diagnostic task reported unsupported level " + std::to_string(status.level);
    } else {
      pipeline.reported_detail = boundedDetail(status.message);
    }
  }
}

}  // namespace manipulation::monitoring
