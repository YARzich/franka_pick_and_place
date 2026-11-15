// Responsibility: Owns the gripper action server, feedback state and ros2_control publisher.
// Boundary: Construction is exposed only through makePandaGripperDriverNode().

#pragma once

#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include "control_msgs/action/gripper_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "franka_motion/gripper_goal_evaluator.hpp"
#include "franka_motion/gripper_goal_reservation.hpp"

namespace franka_motion::detail
{

/**
 * Position-only GripperCommand adapter owned by one SingleThreadedExecutor.
 *
 * The node owns its publishers, subscription, action server, wall timer, and
 * one active goal reservation. Widths are metres; feedback freshness and goal
 * deadlines use the steady clock. Positive max_effort is rejected because the
 * downstream JointGroupPositionController cannot enforce effort.
 *
 * Callback state is intentionally lock-free and non-composable. Supporting a
 * multi-threaded executor requires an explicit callback-group/locking policy.
 */
class PandaGripperDriver final : public rclcpp::Node
{
public:
  using GripperAction = control_msgs::action::GripperCommand;
  using GoalHandle = rclcpp_action::ServerGoalHandle<GripperAction>;
  using SteadyClock = std::chrono::steady_clock;

  PandaGripperDriver();

  /// Finalize an active action handle before process-owned node destruction.
  ///
  /// This publishes no new controller command; the last target may remain
  /// active, so terminal action state does not prove a physical hold/release.
  void shutdownActiveGoal() noexcept;

private:
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & goal_id,
    std::shared_ptr<const GripperAction::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandle> & goal_handle);
  void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle);

  void handleJointState(const sensor_msgs::msg::JointState::SharedPtr msg);
  void monitorActiveGoal();
  void applyGoalEvaluation(const GripperGoalEvaluation & evaluation);

  void abortActiveGoal(const char * reason);
  std::shared_ptr<GripperAction::Result> makeResult(
    bool reached_goal,
    bool stalled) const;
  void publishFeedback(const GripperGoalEvaluation & evaluation);
  void publishWidthCommand(double width);
  void publishHoldPosition();

  std::string command_topic_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp_action::Server<GripperAction>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr monitor_timer_;

  std::shared_ptr<GoalHandle> active_goal_;
  GripperGoalReservation goal_reservation_;
  int command_joint_count_{2};
  double command_joint_min_position_{0.0};
  double command_joint_max_position_{0.04};
  SteadyClock::duration state_timeout_{std::chrono::milliseconds(500)};
  std::optional<GripperGoalEvaluator> goal_evaluator_;
  std::optional<GripperFingerState> last_state_;
};

}  // namespace franka_motion::detail
