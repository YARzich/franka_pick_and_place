// Responsibility: Applies deterministic enter/wait/exit logging and anti-spam throttling.

#include "manipulation/workflow/wait_lifecycle_logger.hpp"

#include <stdexcept>
#include <utility>

namespace manipulation::workflow
{

WaitLifecycleLogger::WaitLifecycleLogger(Sink sink, NowFunction now)
: sink_(std::move(sink)), now_(std::move(now))
{
  if (!sink_) {
    throw std::invalid_argument("wait lifecycle logger requires a sink");
  }
  if (!now_) {
    throw std::invalid_argument("wait lifecycle logger requires a clock function");
  }
}

WaitLifecycleLogger::~WaitLifecycleLogger() = default;

void WaitLifecycleLogger::enter(WaitLifecycleScope scope)
{
  if (scope.reason == TransferWaitingReason::kNone) {
    throw std::invalid_argument("wait lifecycle enter requires a typed waiting reason");
  }
  if (active_) {
    exit(WaitLifecycleOutcome::kError);
  }
  active_ = std::move(scope);
  entered_at_ = now_();
  last_waiting_log_ = entered_at_;
  emit(WaitLifecycleEvent::kEnter, *active_, entered_at_);
}

void WaitLifecycleLogger::waiting()
{
  if (!active_) {
    return;
  }
  const auto now = now_();
  if (now - last_waiting_log_ < kWaitingLogThrottle) {
    return;
  }
  last_waiting_log_ = now;
  emit(WaitLifecycleEvent::kWaiting, *active_, now);
}

void WaitLifecycleLogger::exit(
  const WaitLifecycleOutcome outcome,
  std::string diagnostic)
{
  if (!active_) {
    return;
  }
  auto completed = *active_;
  if (!diagnostic.empty()) {
    completed.diagnostic = std::move(diagnostic);
  }
  emit(WaitLifecycleEvent::kExit, completed, now_(), outcome);
  active_.reset();
}

void WaitLifecycleLogger::emit(
  const WaitLifecycleEvent event,
  const WaitLifecycleScope & scope,
  const Clock::time_point timestamp,
  const std::optional<WaitLifecycleOutcome> outcome)
{
  const auto elapsed = timestamp < entered_at_ ? Clock::duration::zero() :
    timestamp - entered_at_;
  sink_({WaitLogSeverity::kInfo, event, scope, elapsed, outcome});
}

}  // namespace manipulation::workflow
