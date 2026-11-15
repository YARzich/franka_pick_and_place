// Responsibility: Serializes gripper goal admission and transfers ownership of the single active
//   goal.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace franka_motion
{

/// Result of binding an accepted ROS goal handle to its earlier reservation.
enum class GripperGoalActivation
{
  kStart,
  kCancelBeforeStart,
  kInvalidReservation,
};

/**
 * Serial reservation state for the Panda gripper action adapter.
 *
 * The standalone driver invokes this class only from one SingleThreadedExecutor;
 * it is intentionally not thread-safe. Reserving in the action goal callback
 * closes the gap before the accepted callback. A cancel received in that gap
 * is sticky, so activation never publishes a physical command for that goal.
 */
class GripperGoalReservation final
{
public:
  static constexpr std::size_t kGoalIdSize = 16U;
  using GoalId = std::array<std::uint8_t, kGoalIdSize>;

  /// Reserve an idle driver for one goal; reject while any goal owns it.
  [[nodiscard]] bool reserve(const GoalId & goal_id)
  {
    if (phase_ != Phase::kIdle) {
      return false;
    }
    goal_id_ = goal_id;
    phase_ = Phase::kReserved;
    cancel_requested_ = false;
    return true;
  }

  /// Record cancel for the matching reserved or active goal.
  [[nodiscard]] bool request_cancel(const GoalId & goal_id)
  {
    if (phase_ == Phase::kIdle || !goal_id_ || *goal_id_ != goal_id) {
      return false;
    }
    cancel_requested_ = true;
    return true;
  }

  /// Activate the matching reservation and replay any pre-start cancel.
  [[nodiscard]] GripperGoalActivation activate(const GoalId & goal_id)
  {
    if (phase_ != Phase::kReserved || !goal_id_ || *goal_id_ != goal_id) {
      return GripperGoalActivation::kInvalidReservation;
    }
    phase_ = Phase::kActive;
    return cancel_requested_ ? GripperGoalActivation::kCancelBeforeStart :
           GripperGoalActivation::kStart;
  }

  /// Release ownership after the action handle reaches a terminal state.
  void release()
  {
    phase_ = Phase::kIdle;
    goal_id_.reset();
    cancel_requested_ = false;
  }

  /// Return whether a reserved or active goal owns the driver.
  [[nodiscard]] bool occupied() const
  {
    return phase_ != Phase::kIdle;
  }

private:
  enum class Phase
  {
    kIdle,
    kReserved,
    kActive,
  };

  Phase phase_{Phase::kIdle};
  std::optional<GoalId> goal_id_;
  bool cancel_requested_{false};
};

}  // namespace franka_motion
