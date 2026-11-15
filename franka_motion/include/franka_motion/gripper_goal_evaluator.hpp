// Responsibility: Evaluates gripper feedback into deterministic running, success or failure
//   transitions.

#pragma once

#include <chrono>
#include <cmath>
#include <optional>
#include <stdexcept>

#include "franka_motion/gripper_utils.hpp"

namespace franka_motion
{
/// One finger-state snapshot timestamped with the local steady clock.
struct GripperFingerState
{
  /// Left finger-joint position in metres.
  double left_joint{0.0};
  /// Right finger-joint position in metres.
  double right_joint{0.0};
  /// Optional controller-reported effort in its configured hardware units.
  std::optional<double> effort;
  /// Local receipt time; ROS message time is not used for watchdog freshness.
  std::chrono::steady_clock::time_point received_at{};

  /// Return the measured full gripper width, in metres.
  double width() const
  {
    return left_joint + right_joint;
  }
};

/// Return whether both finger positions and their sum are finite.
inline bool isValidGripperFingerState(const GripperFingerState & state)
{
  return std::isfinite(state.left_joint) && std::isfinite(state.right_joint) &&
         std::isfinite(state.width());
}

/// Metre thresholds and steady-clock deadlines for one GripperCommand goal.
struct GripperGoalPolicy
{
  double goal_tolerance{0.004};
  double movement_threshold{0.0005};
  double finger_symmetry_tolerance{0.005};
  double maximum_reverse_motion{0.002};
  bool allow_stalling{true};
  std::chrono::steady_clock::duration state_timeout{std::chrono::milliseconds(500)};
  std::chrono::steady_clock::duration stall_timeout{std::chrono::milliseconds(500)};
  std::chrono::steady_clock::duration goal_timeout{std::chrono::seconds(3)};
};

/// Terminal and non-terminal outcomes understood by the ROS action adapter.
enum class GripperGoalTransition
{
  kInactive,
  kContinue,
  kCanceled,
  kStateTimeout,
  kReached,
  kStallSucceeded,
  kStallAborted,
  kGoalTimeout,
};

/// Pure decision plus side effects the owning ROS adapter must perform.
///
/// `request_hold` does not publish by itself. `start_width` is full fingertip
/// width in metres and is present after a valid post-goal sample is established.
struct GripperGoalEvaluation
{
  GripperGoalTransition transition{GripperGoalTransition::kInactive};
  bool publish_feedback{false};
  bool request_hold{false};
  bool feedback_reached{false};
  bool feedback_stalled{false};
  bool result_reached{false};
  bool result_stalled{false};
  bool closing_progress{false};
  bool finger_motion_supports_contact{false};
  std::optional<double> start_width;
};

///
/// Deterministic steady-clock state machine for a single gripper goal.
///
/// The class owns evaluation state only; ROS goal handles, publications, and
/// logging remain in the node adapter. It is intentionally not thread-safe: one
/// node callback group owns it. `startGoal()` replaces prior state. Cancel and
/// feedback timeout deactivate the goal and request a hold command from the
/// adapter; the evaluator itself performs no physical side effect.
class GripperGoalEvaluator
{
public:
  using SteadyClock = std::chrono::steady_clock;

  /// Construct after validating all finite thresholds and positive deadlines.
  explicit GripperGoalEvaluator(const GripperGoalPolicy & policy)
  : policy_(policy), progress_monitor_(policy.movement_threshold)
  {
    if (!std::isfinite(policy_.goal_tolerance) || policy_.goal_tolerance < 0.0 ||
      !std::isfinite(policy_.movement_threshold) || policy_.movement_threshold < 0.0 ||
      !std::isfinite(policy_.finger_symmetry_tolerance) ||
      policy_.finger_symmetry_tolerance < 0.0 ||
      !std::isfinite(policy_.maximum_reverse_motion) ||
      policy_.maximum_reverse_motion < 0.0 ||
      policy_.state_timeout <= SteadyClock::duration::zero() ||
      policy_.stall_timeout <= SteadyClock::duration::zero() ||
      policy_.goal_timeout <= SteadyClock::duration::zero())
    {
      throw std::invalid_argument("Invalid gripper goal evaluation policy");
    }
  }

  /// Start evaluating a full fingertip width in metres.
  ///
  /// A fresh initial snapshot becomes the progress baseline. Invalid width
  /// throws `std::invalid_argument`; any prior evaluation is replaced.
  void startGoal(
    double target_width,
    SteadyClock::time_point now,
    const std::optional<GripperFingerState> & initial_state = std::nullopt)
  {
    if (!std::isfinite(target_width) || target_width < 0.0 ||
      target_width > kPandaGripperMaxWidth)
    {
      throw std::invalid_argument("Invalid gripper target width");
    }

    active_ = true;
    target_width_ = target_width;
    goal_started_at_ = now;
    start_state_.reset();
    if (initial_state.has_value() && isValidGripperFingerState(initial_state.value()) &&
      initial_state->received_at <= now &&
      now - initial_state->received_at <= policy_.state_timeout)
    {
      start_state_ = initial_state;
    }
    progress_monitor_.reset(now);
  }

  /// Evaluate one steady-clock tick; terminal transitions deactivate the goal.
  GripperGoalEvaluation evaluate(
    SteadyClock::time_point now,
    const std::optional<GripperFingerState> & latest_state,
    bool cancel_requested = false)
  {
    GripperGoalEvaluation evaluation;
    if (!active_) {
      return evaluation;
    }
    if (cancel_requested) {
      evaluation.transition = GripperGoalTransition::kCanceled;
      evaluation.request_hold = true;
      active_ = false;
      return evaluation;
    }

    const bool has_post_goal_state = latest_state.has_value() &&
      isValidGripperFingerState(latest_state.value()) &&
      latest_state->received_at >= goal_started_at_ && latest_state->received_at <= now;
    const bool state_fresh = has_post_goal_state &&
      now - latest_state->received_at <= policy_.state_timeout;
    if (!state_fresh) {
      const auto freshness_reference =
        has_post_goal_state ? latest_state->received_at : goal_started_at_;
      if (now - freshness_reference >= policy_.state_timeout) {
        evaluation.transition = GripperGoalTransition::kStateTimeout;
        evaluation.request_hold = true;
        active_ = false;
      } else {
        evaluation.transition = GripperGoalTransition::kContinue;
      }
      return evaluation;
    }

    if (!start_state_.has_value()) {
      start_state_ = latest_state;
    }
    evaluation.start_width = start_state_->width();

    const double current_width = latest_state->width();
    const bool fingers_synchronized =
      std::abs(latest_state->left_joint - latest_state->right_joint) <=
      policy_.finger_symmetry_tolerance;
    evaluation.feedback_reached = fingers_synchronized &&
      gripperWidthReached(current_width, target_width_, policy_.goal_tolerance);

    // Observe first so resumed motion cannot emit stale stalled=true feedback.
    progress_monitor_.observe(current_width, now);
    evaluation.feedback_stalled = progress_monitor_.hasStalled(now, policy_.stall_timeout);
    evaluation.publish_feedback = true;

    if (evaluation.feedback_reached) {
      evaluation.feedback_stalled = false;
      evaluation.transition = GripperGoalTransition::kReached;
      evaluation.result_reached = true;
      active_ = false;
      return evaluation;
    }

    if (evaluation.feedback_stalled) {
      evaluation.finger_motion_supports_contact =
        gripperFingerMotionSupportsClosingContact(
        start_state_->left_joint,
        start_state_->right_joint,
        latest_state->left_joint,
        latest_state->right_joint,
        progress_monitor_.movementThreshold(),
        policy_.maximum_reverse_motion);
      evaluation.closing_progress = evaluation.finger_motion_supports_contact &&
        gripperStallHasClosingProgress(
        start_state_->width(),
        current_width,
        target_width_,
        progress_monitor_.movementThreshold(),
        policy_.goal_tolerance);
      evaluation.result_stalled = evaluation.closing_progress;
      evaluation.transition = policy_.allow_stalling && evaluation.closing_progress ?
        GripperGoalTransition::kStallSucceeded : GripperGoalTransition::kStallAborted;
      active_ = false;
      return evaluation;
    }

    if (now - goal_started_at_ >= policy_.goal_timeout) {
      evaluation.transition = GripperGoalTransition::kGoalTimeout;
      evaluation.request_hold = true;
      active_ = false;
      return evaluation;
    }

    evaluation.transition = GripperGoalTransition::kContinue;
    return evaluation;
  }

  /// Return whether evaluate still expects samples for the current goal.
  bool hasActiveGoal() const
  {
    return active_;
  }

private:
  GripperGoalPolicy policy_;
  GripperProgressMonitor progress_monitor_;
  bool active_{false};
  double target_width_{0.0};
  SteadyClock::time_point goal_started_at_{};
  std::optional<GripperFingerState> start_state_;
};

}  // namespace franka_motion
