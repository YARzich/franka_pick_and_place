// Responsibility: Classifies observation freshness against startup, warning and error deadlines.

#ifndef MANIPULATION__MONITORING__FRESHNESS_MONITOR_HPP_
#define MANIPULATION__MONITORING__FRESHNESS_MONITOR_HPP_

#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>

namespace manipulation::monitoring
{

/// Severity values intentionally match diagnostic_msgs/DiagnosticStatus.
enum class HealthLevel : std::uint8_t
{
  kOk = 0,
  kWarn = 1,
  kError = 2,
  kStale = 3,
};

/// Steady-clock thresholds used to classify an observed data source.
struct FreshnessPolicy
{
  std::chrono::nanoseconds startup_grace{};
  std::chrono::nanoseconds warn_after{};
  std::chrono::nanoseconds stale_after{};
};

/// Immutable result of one freshness evaluation.
struct FreshnessStatus
{
  HealthLevel level{HealthLevel::kStale};
  bool observed{false};
  bool startup_grace_active{false};
  std::chrono::nanoseconds age{};
};

/// Thread-safe freshness tracker for ROS callbacks and diagnostic callbacks.
/**
 * The tracker uses only std::chrono::steady_clock. It therefore remains valid
 * across simulation-time pauses and resets. Callers provide time points so
 * unit tests and fault-injection tests never need real sleeps.
 */
class FreshnessMonitor
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  /// Construct a tracker and validate all threshold invariants.
  explicit FreshnessMonitor(FreshnessPolicy policy, TimePoint started_at = Clock::now());

  /// Record successful receipt or refresh of the monitored resource.
  void observe(TimePoint observed_at = Clock::now());

  /// Forget the last observation and begin a new startup-grace interval.
  void reset(TimePoint started_at = Clock::now());

  /// Return a race-free snapshot classified against the configured policy.
  [[nodiscard]] FreshnessStatus status(TimePoint now = Clock::now()) const;

private:
  FreshnessPolicy policy_;
  mutable std::mutex mutex_;
  TimePoint started_at_;
  std::optional<TimePoint> last_observed_;
};

}  // namespace manipulation::monitoring

#endif  // MANIPULATION__MONITORING__FRESHNESS_MONITOR_HPP_
