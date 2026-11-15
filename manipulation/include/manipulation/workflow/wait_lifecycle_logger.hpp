// Responsibility: Emits throttled ROS-free lifecycle records for non-physical waits.

#ifndef MANIPULATION__WORKFLOW__WAIT_LIFECYCLE_LOGGER_HPP_
#define MANIPULATION__WORKFLOW__WAIT_LIFECYCLE_LOGGER_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "manipulation/workflow/transfer_execution_context.hpp"

namespace manipulation::workflow
{

enum class WaitLogSeverity : std::uint8_t
{
  kInfo,
};

enum class WaitLifecycleEvent : std::uint8_t
{
  kEnter,
  kWaiting,
  kExit,
};

/// Typed terminal reason for one non-physical wait scope.
enum class WaitLifecycleOutcome : std::uint8_t
{
  kReady,
  kCanceled,
  kShutdown,
  kError,
};

struct WaitLifecycleScope
{
  std::string goal_id;
  std::size_t step_index{0U};
  std::size_t step_count{0U};
  std::string object_id;
  TransferWaitingReason reason{TransferWaitingReason::kNone};
  std::string diagnostic;
};

struct WaitLifecycleRecord
{
  WaitLogSeverity severity{WaitLogSeverity::kInfo};
  WaitLifecycleEvent event{WaitLifecycleEvent::kEnter};
  WaitLifecycleScope scope;
  std::chrono::steady_clock::duration elapsed{};
  std::optional<WaitLifecycleOutcome> outcome;
};

/**
 * Single-execution wait logger with one internal, non-configurable throttle.
 *
 * Normal enter, periodic waiting, and exit records are INFO. Deployments inject
 * a sink that maps these neutral values to their logger of choice; the workflow
 * exposes no YAML/parameter surface for the anti-spam interval.
 */
class WaitLifecycleLogger final
{
public:
  using Clock = std::chrono::steady_clock;
  using Sink = std::function<void(WaitLifecycleRecord)>;
  using NowFunction = std::function<Clock::time_point()>;

  explicit WaitLifecycleLogger(
    Sink sink,
    NowFunction now = [] () {return Clock::now();});
  ~WaitLifecycleLogger();

  WaitLifecycleLogger(const WaitLifecycleLogger &) = delete;
  WaitLifecycleLogger & operator=(const WaitLifecycleLogger &) = delete;
  WaitLifecycleLogger(WaitLifecycleLogger &&) = delete;
  WaitLifecycleLogger & operator=(WaitLifecycleLogger &&) = delete;

  void enter(WaitLifecycleScope scope);
  void waiting();
  void exit(WaitLifecycleOutcome outcome, std::string diagnostic = {});

private:
  static constexpr auto kWaitingLogThrottle = std::chrono::seconds{5};

  void emit(
    WaitLifecycleEvent event,
    const WaitLifecycleScope & scope,
    Clock::time_point timestamp,
    std::optional<WaitLifecycleOutcome> outcome = std::nullopt);

  Sink sink_;
  NowFunction now_;
  std::optional<WaitLifecycleScope> active_;
  Clock::time_point entered_at_{};
  Clock::time_point last_waiting_log_{};
};

}  // namespace manipulation::workflow

#endif  // MANIPULATION__WORKFLOW__WAIT_LIFECYCLE_LOGGER_HPP_
