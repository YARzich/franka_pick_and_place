// Responsibility: Defines pure Panda gripper geometry, validity and motion-progress predicates.
// Boundary: This policy layer has no ROS communication or mutable driver state.

#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>

namespace franka_motion
{
/// Maximum distance between the two Panda fingertips, in metres.
constexpr double kPandaGripperMaxWidth = 0.08;

/// Clamp a requested full gripper width to Panda's physical range.
inline double clampGripperWidth(double width)
{
  return std::clamp(width, 0.0, kPandaGripperMaxWidth);
}

/// Convert full fingertip width to one symmetric finger-joint position.
inline double widthToJoint(double width)
{
  return std::clamp(width * 0.5, 0.0, 0.04);
}

/// Return whether a GripperCommand request contains finite, valid values.
inline bool isValidGripperCommand(double width, double max_effort)
{
  return std::isfinite(width) && std::isfinite(max_effort) && max_effort >= 0.0;
}

/// Return whether this position-only adapter can honor the effort field.
inline bool positionOnlyGripperAcceptsMaxEffort(double max_effort)
{
  // Within this adapter contract, zero means no requested effort limit. A
  // positive limit requires an effort-capable controller and action adapter.
  return std::isfinite(max_effort) && max_effort == 0.0;
}

/// Test a measured full width against a target and non-negative tolerance.
inline bool gripperWidthReached(double current_width, double target_width, double tolerance)
{
  return std::isfinite(current_width) && std::isfinite(target_width) &&
         std::isfinite(tolerance) && tolerance >= 0.0 &&
         std::abs(current_width - target_width) <= tolerance;
}

/// Return whether the target requests closing from the measured width.
inline bool isClosingGripperCommand(
  double current_width,
  double target_width,
  double tolerance)
{
  return std::isfinite(current_width) && std::isfinite(target_width) &&
         std::isfinite(tolerance) && tolerance >= 0.0 &&
         target_width < current_width - tolerance;
}

/// Validate that a stalled closing command made measurable progress toward its target.
inline bool gripperStallHasClosingProgress(
  double start_width,
  double stalled_width,
  double target_width,
  double minimum_progress,
  double tolerance)
{
  return std::isfinite(start_width) && std::isfinite(stalled_width) &&
         std::isfinite(target_width) && std::isfinite(minimum_progress) &&
         std::isfinite(tolerance) && minimum_progress >= 0.0 && tolerance >= 0.0 &&
         isClosingGripperCommand(start_width, target_width, tolerance) &&
         stalled_width < start_width - minimum_progress &&
         stalled_width >= target_width - tolerance;
}

///
/// Validate per-finger motion for an off-centre contact. One finger may remain
/// at contact while the other closes, but neither may reverse beyond the
/// configured tolerance.
inline bool gripperFingerMotionSupportsClosingContact(
  double start_left,
  double start_right,
  double current_left,
  double current_right,
  double minimum_progress,
  double maximum_reverse_motion)
{
  if (!std::isfinite(start_left) || !std::isfinite(start_right) ||
    !std::isfinite(current_left) || !std::isfinite(current_right) ||
    !std::isfinite(minimum_progress) || !std::isfinite(maximum_reverse_motion) ||
    minimum_progress < 0.0 || maximum_reverse_motion < 0.0)
  {
    return false;
  }

  const double left_progress = start_left - current_left;
  const double right_progress = start_right - current_right;
  return std::max(left_progress, right_progress) > minimum_progress &&
         left_progress >= -maximum_reverse_motion &&
         right_progress >= -maximum_reverse_motion;
}

/// Steady-clock detector for lack of measurable full-width progress.
class GripperProgressMonitor
{
public:
  /// Construct with the minimum width delta that counts as progress.
  explicit GripperProgressMonitor(double movement_threshold)
  : movement_threshold_(std::max(0.0, movement_threshold))
  {
  }

  /// Start a new observation interval and discard the preceding sample.
  void reset(std::chrono::steady_clock::time_point now)
  {
    last_progress_width_.reset();
    last_progress_time_ = now;
  }

  /// Record a finite width; returns false without changing state for non-finite input.
  bool observe(double width, std::chrono::steady_clock::time_point now)
  {
    if (!std::isfinite(width)) {
      return false;
    }

    if (!last_progress_width_.has_value() ||
      std::abs(width - last_progress_width_.value()) > movement_threshold_)
    {
      last_progress_width_ = width;
      last_progress_time_ = now;
    }
    return true;
  }

  /// Return true once a valid sample has not changed for at least timeout.
  bool hasStalled(
    std::chrono::steady_clock::time_point now,
    std::chrono::steady_clock::duration timeout) const
  {
    return last_progress_width_.has_value() && now - last_progress_time_ >= timeout;
  }

  /// Return the configured full-width progress threshold, in metres.
  double movementThreshold() const
  {
    return movement_threshold_;
  }

private:
  double movement_threshold_;
  std::optional<double> last_progress_width_;
  std::chrono::steady_clock::time_point last_progress_time_{};
};
}  // namespace franka_motion
