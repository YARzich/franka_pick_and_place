// Responsibility: Decides when one-shot helper output is ready, failed or no longer worth waiting
//   for.

#ifndef ONE_SHOT_PROCESS_RESULT_WAIT_POLICY_HPP_
#define ONE_SHOT_PROCESS_RESULT_WAIT_POLICY_HPP_

#include <algorithm>
#include <chrono>
#include <optional>

namespace franka_motion
{

/// Latest observable lifecycle state of an owned one-shot child process.
enum class OneShotProcessState
{
  kRunning,
  kSucceeded,
  kFailed,
};

/**
 * Steady-clock grace policy for DDS delivery after a one-shot child exits.
 *
 * Process failure stops waiting immediately. A successful exit starts one
 * bounded grace interval because reliable DDS acknowledgement can precede the
 * parent's subscription callback. The policy is stateful and not thread-safe;
 * create one instance per child invocation.
 */
class OneShotProcessResultWaitPolicy
{
public:
  using Clock = std::chrono::steady_clock;

  explicit OneShotProcessResultWaitPolicy(Clock::duration successful_exit_delivery_grace)
  : successful_exit_delivery_grace_(std::max(
        Clock::duration::zero(), successful_exit_delivery_grace))
  {
  }

  /// Return whether the parent should continue waiting at the supplied steady time.
  bool shouldKeepWaiting(const OneShotProcessState state, const Clock::time_point now)
  {
    if (state == OneShotProcessState::kFailed) {
      return false;
    }
    if (state == OneShotProcessState::kRunning) {
      successful_exit_observed_at_.reset();
      return true;
    }

    if (!successful_exit_observed_at_.has_value()) {
      successful_exit_observed_at_ = now;
    }
    return now - *successful_exit_observed_at_ < successful_exit_delivery_grace_;
  }

private:
  Clock::duration successful_exit_delivery_grace_;
  std::optional<Clock::time_point> successful_exit_observed_at_;
};

}  // namespace franka_motion

#endif  // ONE_SHOT_PROCESS_RESULT_WAIT_POLICY_HPP_
